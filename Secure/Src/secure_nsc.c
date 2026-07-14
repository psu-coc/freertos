/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Secure/Src/secure_nsc.c
  * @author  MCD Application Team
  * @brief   This file contains the non-secure callable APIs (secure world)
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN Non_Secure_CallLib */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "secure_nsc.h"
#include <stdio.h>
#include "secure_port_macros.h"
#include <string.h>
#include "Crypto/hmac-sha256/hmac-sha256.h"
//#include "aes-gcm/aes.h"   // รวม AES CBC, CTR, ECB ไว้หมด
#include "Aesnew/aes.h"
#include "arm_cmse.h"
//#include "Speck/speck.h"
//#include "Speck/ff1_speck.h"

#define USE_SAU_APPROACH        1

#define SHA256_DIGEST_SIZE 32
#define BLOCK_SIZE 4096         // // <--- แก้ตัวเลขตรงนี้ครับ (256, 512, 1024, 2048, 4096)
/*
 * Keep attestation image inside Secure flash only (0x08000000..0x0803FFFF).
 * TOTAL_SIZE=0x80000 spills into NS flash (@0x08040000) where NS code lives.
 * With USE_SAU_APPROACH + IRQ on, marking those pages Secure causes NS SecureFault → reset loop.
 */
#define TOTAL_SIZE 0x40000
#define BLOCKS (TOTAL_SIZE / BLOCK_SIZE)

#define SAU_DYNAMIC_REGION      7U

static void sau_enable_dwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t sau_rbar_aligned(const uint8_t *block_ptr)
{
    return ((uint32_t)block_ptr & ~0x1FU);
}

static uint32_t sau_rlar_secure(const uint8_t *block_ptr, size_t b)
{
    return (((uint32_t)block_ptr + (uint32_t)b - 1U) | 0x1FU) | 0x1U;
}

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */
static void (*ns_print_cb)(const char *) = NULL;

CMSE_NS_ENTRY void SECURE_RegisterPrintCallback(void *callback)
{
    ns_print_cb = (void (*)(const char *))cmse_nsfptr_create(callback);
}

CMSE_NS_ENTRY void SECURE_Print(const char *msg)
{
    if (ns_print_cb)
    {
        ns_print_cb(msg);
    }
}

/**
  * @brief  Secure registration of non-secure callback.
  * @param  CallbackId  callback identifier
  * @param  func        pointer to non-secure function
  * @retval None
  */
CMSE_NS_ENTRY void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func)
{
  if(func != NULL)
  {
    switch(CallbackId)
    {
      case SECURE_FAULT_CB_ID:           /* SecureFault Interrupt occurred */
        pSecureFaultCallback = func;
        break;
      case GTZC_ERROR_CB_ID:             /* GTZC Interrupt occurred */
        pSecureErrorCallback = func;
        break;
      default:
        /* unknown */
        break;
    }
  }
}

/*CMSE_NS_ENTRY*/secureportNON_SECURE_CALLABLE void SECURE_LEDToggle(void)
{
  BSP_LED_Toggle(LED1);
}

uint8_t *real_memory = (uint8_t *)0x8000000; // อันเก่าใช้ 0x8040000
static const uint8_t key[] = "MySecureKey123"; // Example key
static hmac_sha256 hmac;

// ---- Key/IV derivation: HMAC(secret, challenge) -> 32B -> 16B key + 16B iv
static void derive_aes_key_iv_from_challenge(uint8_t key16[16],
                                             uint8_t iv16[16],
                                             const uint8_t *challenge,
                                             size_t clen)
{
    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen((const char *)key));
    if (challenge && clen) {
        hmac_sha256_update(&hmac, challenge, clen);
    } else {
        // fallback entropy so it’s never constant
        uint32_t tick = (uint32_t)SysTick->VAL;
        hmac_sha256_update(&hmac, (uint8_t*)&tick, sizeof(tick));
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(key16, hmac.digest, 16);
    memcpy(iv16,  hmac.digest + 16, 16);
}

// ---- PRNG: AES-CTR keystream -> 32-bit samples
// keep a small keystream buffer so we don’t re-encrypt every time
typedef struct {
    struct AES_ctx ctx;
    uint8_t  buf[16];
    int      idx;   // next unread byte in buf (0..16)
} ctr_prng_t;

static void prng_init(ctr_prng_t *p, const uint8_t key16[16], const uint8_t iv16[16])
{
    AES_init_ctx_iv(&p->ctx, key16, iv16);
    memset(p->buf, 0, sizeof(p->buf));
    p->idx = 16; // force refill on first use
}

static void prng_refill_block(ctr_prng_t *p)
{
    // encrypt zero block → keystream, tiny-AES increments IV internally
    uint8_t zero[16] = {0};
    memcpy(p->buf, zero, 16);
    AES_CTR_xcrypt_buffer(&p->ctx, p->buf, 16); // p->ctx.Iv auto-increments
    p->idx = 0;
}

static uint32_t prng_next_u32(ctr_prng_t *p)
{
    if (p->idx > 12) {           // not enough bytes left → refill
        prng_refill_block(p);
    }
    uint32_t v;
    memcpy(&v, &p->buf[p->idx], 4);
    p->idx += 4;
    return v;
}

// unbiased integer in [0, n)
static int prng_uniform_u32(ctr_prng_t *p, int n)
{
    // rejection sampling to avoid modulo bias
    const uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
    for (;;) {
        uint32_t r = prng_next_u32(p);
        if (r < lim) return (int)(r % (uint32_t)n);
    }
}

// ---- Fisher–Yates using the PRNG above
static void shuffle_secure_aes_ctr(int *arr, int n,
                                   const uint8_t key16[16],
                                   const uint8_t iv16[16])
{
    ctr_prng_t prng;
    prng_init(&prng, key16, iv16);

    for (int i = n - 1; i > 0; i--) {
        int j = prng_uniform_u32(&prng, i + 1);
        int tmp = arr[i]; arr[i] = arr[j]; arr[j] = tmp;
    }
}

// ---- Non-secure callable: secure shuffle + HMAC over blocks
__attribute__((cmse_nonsecure_entry))
void SECURE_ShuffledHMAC_secure(uint8_t *out_digest,
                                const uint8_t *challenge, size_t challenge_len,
                                uint32_t *out_sau_avg_cycles)
{
    /* Validate NS pointers before writing across the security boundary. */
    out_digest = cmse_check_address_range(out_digest, SHA256_DIGEST_SIZE, CMSE_NONSECURE);
    if (!out_digest) return;
    if (out_sau_avg_cycles) {
        out_sau_avg_cycles = cmse_check_pointed_object(out_sau_avg_cycles, CMSE_NONSECURE);
        if (!out_sau_avg_cycles) return;
    }
    if (challenge && challenge_len) {
        challenge = cmse_check_address_range((void *)challenge, challenge_len, CMSE_NONSECURE);
        if (!challenge) return;
    }

    // 1) indices = 0..BLOCKS-1
    static int indices[BLOCKS];
    for (int i = 0; i < BLOCKS; i++) indices[i] = i;

    // 2) derive AES key/IV from challenge
    uint8_t key16[16], iv16[16];
    derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);

    // 3) secure shuffle
    shuffle_secure_aes_ctr(indices, BLOCKS, key16, iv16);

#if defined(USE_SAU_APPROACH)
    sau_enable_dwt();
#endif

    // 4) HMAC over shuffled blocks
    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen((const char *)key));
#if defined(USE_SAU_APPROACH)
    uint64_t sau_cycles_total = 0U;
#endif
    for (int i = 0; i < BLOCKS; i++) {
        const uint8_t *blk = &real_memory[(size_t)indices[i] * BLOCK_SIZE];
#if defined(USE_SAU_APPROACH)
        uint32_t t_sau_start = DWT->CYCCNT;

        SAU->RNR  = SAU_DYNAMIC_REGION;
        SAU->RBAR = sau_rbar_aligned(blk);
        SAU->RLAR = sau_rlar_secure(blk, BLOCK_SIZE);
        __DSB();
        __ISB();

        uint32_t t_sau_end = DWT->CYCCNT;
        sau_cycles_total += (uint64_t)(t_sau_end - t_sau_start);

        hmac_sha256_update(&hmac, blk, BLOCK_SIZE);

        SAU->RNR  = SAU_DYNAMIC_REGION;
        SAU->RLAR = 0U;
        __DSB();
        __ISB();
#else
        __disable_irq();
        hmac_sha256_update(&hmac, blk, BLOCK_SIZE);
        __enable_irq();
#endif
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
#if defined(USE_SAU_APPROACH)
    if (out_sau_avg_cycles) {
        *out_sau_avg_cycles = (uint32_t)(sau_cycles_total / (uint64_t)BLOCKS);
    }
#else
    if (out_sau_avg_cycles) {
        *out_sau_avg_cycles = 0U;
    }
#endif
}

/* USER CODE END Non_Secure_CallLib */
