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

#include "main.h"
#include "secure_nsc.h"
#include <stdio.h>
#include "secure_port_macros.h"
#include <string.h>
#include "Crypto/hmac-sha256/hmac-sha256.h"
#include "aes-gcm/aes.h"
#include "Aesnew/aes.h"
#include "arm_cmse.h"
#include "Speck/speck.h"
#include "Speck/ff1_speck.h"

#define SHA256_DIGEST_SIZE 32
#define BLOCK_SIZE 64
#define TOTAL_SIZE 0x80000
#define BLOCKS (TOTAL_SIZE / BLOCK_SIZE)

__attribute__((section(".gnu.linkonce.b._ns_work_buffer")))
static uint8_t ns_work_buffer[256] __attribute__((aligned(8))) __attribute__((unused));

void *pSecureFaultCallback = NULL;
void *pSecureErrorCallback = NULL;

CMSE_NS_ENTRY void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func)
{
  if(func != NULL)
  {
    switch(CallbackId)
    {
      case SECURE_FAULT_CB_ID:
        pSecureFaultCallback = func;
        break;
      case GTZC_ERROR_CB_ID:
        pSecureErrorCallback = func;
        break;
      default:
        break;
    }
  }
}

uint8_t *real_memory = (uint8_t *)0x8000000;

static const char key[] = "MySecureKey123";
static hmac_sha256 hmac;

static void derive_aes_key_iv_from_challenge(uint8_t key16[16],
                                             uint8_t iv16[16],
                                             const uint8_t *challenge,
                                             size_t clen)
{
    hmac_sha256_initialize(&hmac, (const uint8_t *)key, strlen(key));
    if (challenge && clen) {
        hmac_sha256_update(&hmac, challenge, clen);
    } else {
        uint32_t tick = (uint32_t)SysTick->VAL;
        hmac_sha256_update(&hmac, (uint8_t *)&tick, sizeof(tick));
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(key16, hmac.digest, 16);
    memcpy(iv16,  hmac.digest + 16, 16);
}

typedef struct {
    struct AES_ctx ctx;
    uint8_t  buf[16];
    int      idx;
} ctr_prng_t;

__attribute__((unused)) static void prng_init(ctr_prng_t *p, const uint8_t key16[16], const uint8_t iv16[16])
{
    AES_init_ctx_iv(&p->ctx, key16, iv16);
    memset(p->buf, 0, sizeof(p->buf));
    p->idx = 16;
}

__attribute__((unused)) static void prng_refill_block(ctr_prng_t *p)
{
    uint8_t zero[16] = {0};
    memcpy(p->buf, zero, 16);
    AES_CTR_xcrypt_buffer(&p->ctx, p->buf, 16);
    p->idx = 0;
}

__attribute__((unused)) static uint32_t prng_next_u32(ctr_prng_t *p)
{
    if (p->idx > 12) {
        prng_refill_block(p);
    }
    uint32_t v;
    memcpy(&v, &p->buf[p->idx], 4);
    p->idx += 4;
    return v;
}

__attribute__((unused)) static int prng_uniform_u32(ctr_prng_t *p, int n)
{
    const uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
    for (;;) {
        uint32_t r = prng_next_u32(p);
        if (r < lim) return (int)(r % (uint32_t)n);
    }
}

static FF1_Key_Speck s_ff1_speck_key;

__attribute__((cmse_nonsecure_entry))
void SECURE_RTSMARM_FF1_Speck_ShuffledHMAC_secure(uint8_t *out_digest, size_t out_len,
                                                  const uint8_t *challenge, size_t challenge_len)
{
    if (!out_digest || out_len < SHA256_DIGEST_SIZE) return;

    uint8_t key16[16], iv16[16];
    derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);

    FF1_SetKey_Speck(&s_ff1_speck_key, key16);
    uint32_t tweak = 0u;
    if (challenge && challenge_len >= 4u)
        tweak = ((uint32_t)challenge[0] << 24) | ((uint32_t)challenge[1] << 16)
              | ((uint32_t)challenge[2] << 8) | (uint32_t)challenge[3];

    hmac_sha256_initialize(&hmac, (const uint8_t *)key, strlen(key));

    for (uint32_t i = 0u; i < (uint32_t)BLOCKS; i++) {
        uint32_t idx = FF1Permute_Speck(i, (uint32_t)BLOCKS, &s_ff1_speck_key, tweak);
        if (idx >= (uint32_t)BLOCKS) continue;
//        __disable_irq();
        hmac_sha256_update(&hmac, &real_memory[(size_t)idx * BLOCK_SIZE], BLOCK_SIZE);
//        __enable_irq();
    }

    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
}

/* USER CODE END Non_Secure_CallLib */
