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
#define BLOCK_SIZE 64        /* change for sweeps: 64,128,256,512,1024,2048,4096 */
/*
 * Dedicated NS **data** window (must not overlap NS app code at 0x08040000..0x0804FFFF):
 *   code/rodata : 0x08040000..0x0804FFFF  (64 KiB; SAU region 1 — not attested here)
 *   data window : 0x08060000..0x0807FFFF  (128 KiB; SAU regions 6/7 at runtime)
 *   gap         : 0x08050000..0x0805FFFF  (64 KiB — not attested; keeps |M| off NS ROM tail)
 * SAU hole-remap over the running NS image (256 KiB from 0x08040000) hangs the MCU.
 * |M| for memory locking on this board is 128 KiB in bank-2 tail (not full 256 KiB NS flash).
 *
 * Armv8-M SAU regions are only Non-Secure or Secure+NSC when ENABLE=1.
 * Ordinary Secure attribution is the default for addresses not covered by any
 * enabled SAU region. Per-block memory locking (professor sequence):
 *   disable IRQ; DSB; SAU->CTRL disable; configure regions 6/7;
 *   SAU->CTRL enable; DSB; ISB; enable IRQ — then HMAC with IRQ on.
 */
#define ATTEST_DATA_BASE   0x08060000U
#define TOTAL_SIZE         0x20000U      /* 128 KiB */
#define BLOCKS             (TOTAL_SIZE / BLOCK_SIZE)

#define SAU_DATA_REGION_A  6U
#define SAU_DATA_REGION_B  7U

#if (BLOCK_SIZE < 32U) || ((BLOCK_SIZE & 31U) != 0U)
#error BLOCK_SIZE must be a multiple of the SAU 32-byte granule
#endif
#if ((ATTEST_DATA_BASE & 31U) != 0U) || ((TOTAL_SIZE & 31U) != 0U)
#error attestation window must be 32-byte aligned
#endif
#if (TOTAL_SIZE % BLOCK_SIZE) != 0U
#error TOTAL_SIZE must be divisible by BLOCK_SIZE
#endif

static void sau_enable_dwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Program one enabled Non-Secure SAU region [start, end_inclusive]. */
static void sau_program_ns_region(uint32_t rnr, uint32_t start, uint32_t end_inclusive)
{
    SAU->RNR  = rnr;
    SAU->RBAR = start & SAU_RBAR_BADDR_Msk;
    SAU->RLAR = (end_inclusive & SAU_RLAR_LADDR_Msk) | SAU_RLAR_ENABLE_Msk;
}

static void sau_disable_region(uint32_t rnr)
{
    SAU->RNR  = rnr;
    SAU->RBAR = 0U;
    SAU->RLAR = 0U;
}

/* Program R6/R7 only (SAU may be disabled — registers still writable). */
static void sau_program_cover_window_ns(void)
{
    sau_program_ns_region(SAU_DATA_REGION_A,
                          ATTEST_DATA_BASE,
                          ATTEST_DATA_BASE + TOTAL_SIZE - 1U);
    sau_disable_region(SAU_DATA_REGION_B);
}

/*
 * Leave [blk, blk+BLOCK_SIZE) uncovered → Secure by default; rest of window NS via R6/R7.
 */
static void sau_program_hole_for_block(const uint8_t *blk)
{
    const uint32_t hole_start = (uint32_t)blk;
    const uint32_t hole_end   = hole_start + (uint32_t)BLOCK_SIZE;
    const uint32_t win_start  = ATTEST_DATA_BASE;
    const uint32_t win_end    = ATTEST_DATA_BASE + TOTAL_SIZE;

    const uint32_t left_start  = win_start;
    const uint32_t left_end_ex = hole_start;
    const uint32_t right_start = hole_end;
    const uint32_t right_end_ex = win_end;

    const int have_left  = (left_end_ex > left_start);
    const int have_right = (right_end_ex > right_start);

    if (have_left && have_right) {
        sau_program_ns_region(SAU_DATA_REGION_A, left_start, left_end_ex - 1U);
        sau_program_ns_region(SAU_DATA_REGION_B, right_start, right_end_ex - 1U);
    } else if (have_left) {
        sau_program_ns_region(SAU_DATA_REGION_A, left_start, left_end_ex - 1U);
        sau_disable_region(SAU_DATA_REGION_B);
    } else if (have_right) {
        sau_program_ns_region(SAU_DATA_REGION_A, right_start, right_end_ex - 1U);
        sau_disable_region(SAU_DATA_REGION_B);
    } else {
        sau_disable_region(SAU_DATA_REGION_A);
        sau_disable_region(SAU_DATA_REGION_B);
    }
}

/*
 * Professor / AN lock sequence: finish prior accesses, toggle SAU, program, barriers.
 * Returns DWT cycle span of the full sequence (for SAU_config_avg_cycles).
 */
static uint32_t sau_apply_configuration_locked(int lock_block, const uint8_t *blk)
{
    uint32_t t0;
    uint32_t t1;

    __disable_irq();
    t0 = DWT->CYCCNT;
    __DSB();
    SAU->CTRL &= ~SAU_CTRL_ENABLE_Msk;
    __DSB();
    if (lock_block) {
        sau_program_hole_for_block(blk);
    } else {
        sau_program_cover_window_ns();
    }
    SAU->CTRL |= SAU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();
    t1 = DWT->CYCCNT;
    __enable_irq();
    return (t1 - t0);
}

static uint32_t sau_memory_lock_block(const uint8_t *blk)
{
    return sau_apply_configuration_locked(1, blk);
}

static uint32_t sau_memory_unlock_window(void)
{
    return sau_apply_configuration_locked(0, NULL);
}

static void sau_cover_data_window_ns(void)
{
    (void)sau_memory_unlock_window();
}

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */
static volatile uint32_t *s_ns_phase_for_call = NULL;

#define ATTEST_PHASE_ENTER        1U
#define ATTEST_PHASE_SHUFFLE      2U
#define ATTEST_PHASE_SAU          3U
#define ATTEST_PHASE_BLOCK_BASE   100U
#define ATTEST_PHASE_DONE         9000U

static void attest_phase_set(uint32_t v)
{
    if (s_ns_phase_for_call != NULL) {
        *s_ns_phase_for_call = v;
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

uint8_t *real_memory = (uint8_t *)ATTEST_DATA_BASE;
static const uint8_t key[] = "MySecureKey123"; /* Example key */
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
                                SECURE_AttestReport_t *report)
{
    s_ns_phase_for_call = NULL;
    if (report != NULL) {
        report = (SECURE_AttestReport_t *)cmse_check_pointed_object(report, CMSE_NONSECURE);
        if (report != NULL && report->attest_phase_ptr != NULL) {
            s_ns_phase_for_call = (volatile uint32_t *)cmse_check_pointed_object(
                (void *)report->attest_phase_ptr, CMSE_NONSECURE);
        }
    }

    /* Validate NS pointers before writing across the security boundary. */
    out_digest = cmse_check_address_range(out_digest, SHA256_DIGEST_SIZE, CMSE_NONSECURE);
    if (!out_digest) {
        if (report != NULL) {
            report->sau_config_avg_cycles = 0xBAD00001U;
        }
        return;
    }
    if (challenge && challenge_len) {
        challenge = cmse_check_address_range((void *)challenge, challenge_len, CMSE_NONSECURE);
        if (!challenge) {
            if (report != NULL) {
                report->sau_config_avg_cycles = 0xBAD00002U;
            }
            return;
        }
    }

    BSP_LED_Toggle(LED1);
    attest_phase_set(ATTEST_PHASE_ENTER);

    // 1) indices = 0..BLOCKS-1
    static int indices[BLOCKS];
    for (int i = 0; i < BLOCKS; i++) indices[i] = i;

    // 2) derive AES key/IV from challenge
    uint8_t key16[16], iv16[16];
    derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);

    // 3) secure shuffle
    shuffle_secure_aes_ctr(indices, BLOCKS, key16, iv16);
    attest_phase_set(ATTEST_PHASE_SHUFFLE);

#if defined(USE_SAU_APPROACH)
    sau_enable_dwt();
    /* Ensure data window is Non-Secure before punching holes. */
    sau_cover_data_window_ns();
    attest_phase_set(ATTEST_PHASE_SAU);
#endif

    // 4) HMAC over shuffled blocks
    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen((const char *)key));
#if defined(USE_SAU_APPROACH)
    uint64_t sau_cycles_total = 0U;
#endif
    for (int i = 0; i < BLOCKS; i++) {
        const uint8_t *blk = &real_memory[(size_t)indices[i] * BLOCK_SIZE];
#if defined(USE_SAU_APPROACH)
        /* Guard: block must lie wholly inside the dedicated data window. */
        if (((uint32_t)blk < ATTEST_DATA_BASE) ||
            (((uint32_t)blk + (uint32_t)BLOCK_SIZE) > (ATTEST_DATA_BASE + TOTAL_SIZE)) ||
            (((uint32_t)blk & 31U) != 0U)) {
            sau_cover_data_window_ns();
            if (report != NULL) {
                report->sau_config_avg_cycles = 0U;
            }
            return;
        }

        if (i == 0U || ((i & 3U) == 0U) || (i + 1) == BLOCKS) {
            attest_phase_set(ATTEST_PHASE_BLOCK_BASE + (uint32_t)i);
            if ((i & 7U) == 7U) {
                BSP_LED_Toggle(LED1);
            }
        }

        sau_cycles_total += (uint64_t)sau_memory_lock_block(blk);

        /* Hash in place with interrupts enabled (block is Secure by default). */
        hmac_sha256_update(&hmac, blk, BLOCK_SIZE);

        sau_cycles_total += (uint64_t)sau_memory_unlock_window();
#else
        __disable_irq();
        hmac_sha256_update(&hmac, blk, BLOCK_SIZE);
        __enable_irq();
#endif
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
#if defined(USE_SAU_APPROACH)
    /* Leave the data window Non-Secure after attestation. */
    sau_cover_data_window_ns();
    if (report != NULL) {
        report->sau_config_avg_cycles = (uint32_t)(sau_cycles_total / (uint64_t)BLOCKS);
    }
    attest_phase_set(ATTEST_PHASE_DONE);
#else
    if (report != NULL) {
        report->sau_config_avg_cycles = 0U;
    }
#endif
}

/* USER CODE END Non_Secure_CallLib */
