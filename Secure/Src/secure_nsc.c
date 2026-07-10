/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Secure/Src/secure_nsc.c
  * @author  MCD Application Team
  * @brief   This file contains the non-secure callable APIs (secure world)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include "aes-gcm/aes.h"
#include "Aesnew/aes.h"
#include "arm_cmse.h"

#define SHA256_DIGEST_SIZE 32
#define BLOCK_SIZE 256
#define TOTAL_SIZE 0x80000
#define BLOCKS (TOTAL_SIZE / BLOCK_SIZE)

__attribute__((section(".gnu.linkonce.b._ns_work_buffer")))
static uint8_t ns_work_buffer[256] __attribute__((aligned(8))) __attribute__((unused));

/** @addtogroup STM32L5xx_HAL_Examples
  * @{
  */

/** @addtogroup FreeRTOS_SecureIOToggle_TrustZone
  * @{
  */

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */

/* Private functions ---------------------------------------------------------*/

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
        hmac_sha256_update(&hmac, (uint8_t*)&tick, sizeof(tick));
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

static void prng_init(ctr_prng_t *p, const uint8_t key16[16], const uint8_t iv16[16])
{
    AES_init_ctx_iv(&p->ctx, key16, iv16);
    memset(p->buf, 0, sizeof(p->buf));
    p->idx = 16;
}

static void prng_refill_block(ctr_prng_t *p)
{
    uint8_t zero[16] = {0};
    memcpy(p->buf, zero, 16);
    AES_CTR_xcrypt_buffer(&p->ctx, p->buf, 16);
    p->idx = 0;
}

static uint32_t prng_next_u32(ctr_prng_t *p)
{
    if (p->idx > 12) {
        prng_refill_block(p);
    }
    uint32_t v;
    memcpy(&v, &p->buf[p->idx], 4);
    p->idx += 4;
    return v;
}

static int prng_uniform_u32(ctr_prng_t *p, int n)
{
    const uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
    for (;;) {
        uint32_t r = prng_next_u32(p);
        if (r < lim) return (int)(r % (uint32_t)n);
    }
}

#define RTSMARM_BITARRAY_BYTES  ((BLOCKS + 7) / 8)
static uint8_t rtsmarm_used_bits[RTSMARM_BITARRAY_BYTES];

static int rtsmarm_get_bit(int i)
{
    return (rtsmarm_used_bits[i / 8] >> (i % 8)) & 1;
}

static void rtsmarm_set_bit(int i)
{
    rtsmarm_used_bits[i / 8] |= (1U << (i % 8));
}

static void rtsmarm_clear_bits(int n)
{
    memset(rtsmarm_used_bits, 0, (size_t)((n + 7) / 8));
}

static int rtsmarm_find_kth_zero(int n, int k)
{
    int count = 0;
    int result = 0;
    for (int i = 0; i < n; i++) {
        if (rtsmarm_get_bit(i) == 0) {
            if (count == k) {
                result = i;
            }
            count++;
        }
    }
    return result;
}

__attribute__((cmse_nonsecure_entry))
void SECURE_RTSMARM_ShuffledHMAC_secure(uint8_t *out_digest, size_t out_len,
                                        const uint8_t *challenge, size_t challenge_len)
{
    if (!out_digest || out_len < SHA256_DIGEST_SIZE) return;

    uint8_t key16[16], iv16[16];
    derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);

    ctr_prng_t prng;
    prng_init(&prng, key16, iv16);

    rtsmarm_clear_bits(BLOCKS);

    hmac_sha256_initialize(&hmac, (const uint8_t *)key, strlen(key));

    for (int round = 0; round < BLOCKS; round++) {
        int remaining = BLOCKS - round;
        int k = prng_uniform_u32(&prng, remaining);
        int idx = rtsmarm_find_kth_zero(BLOCKS, k);
        rtsmarm_set_bit(idx);
        __disable_irq();
        hmac_sha256_update(&hmac, &real_memory[(size_t)idx * BLOCK_SIZE], BLOCK_SIZE);
        __enable_irq();
    }

    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
}

/**
  * @}
  */

/**
  * @}
  */
/* USER CODE END Non_Secure_CallLib */
