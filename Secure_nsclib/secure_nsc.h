/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Secure_nsclib/secure_nsc.h
  * @author  MCD Application Team
  * @brief   Header for secure non-secure callable APIs list
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN Non_Secure_CallLib_h */
#ifndef SECURE_NSC_H
#define SECURE_NSC_H

#include <stdint.h>
#include <stddef.h>
#include "attest_overhead_cfg.h"

extern void *pSecureFaultCallback;
extern void *pSecureErrorCallback;

typedef enum
{
  SECURE_FAULT_CB_ID     = 0x00U,
  GTZC_ERROR_CB_ID       = 0x01U
} SECURE_CallbackIDTypeDef;

/**
 * Written by Secure into NS RAM.
 * t_disabled_*: DWT CYCCNT span of each IRQ-off window (PRIMASK) in this round.
 *   Baseline = HMAC-update per block; Snap = memcpy per block;
 *   Guard = each SAU lock and each SAU unlock (hash runs with IRQs on).
 * sau_config_avg_cycles: Guard only — mean of (lock+unlock) cycles per block
 *   (legacy FAR log field; 0 for Baseline/Snap).
 * t_disabled_std_cycles: population stddev of all IRQ-off windows this round
 *   (from sum-of-squares; no per-window UART dump needed).
 * Optional raw dump (TDIS_RAW_DUMP): Secure fills raw_cycles[0..raw_count)
 *   and raw_tag[] with 'L'/'U' (Guard) or 'W' (Baseline/Snap window).
 */
typedef struct {
  uint32_t sau_config_avg_cycles;
  uint32_t t_disabled_avg_cycles;
  uint32_t t_disabled_max_cycles;
  uint32_t t_disabled_min_cycles;
  uint32_t t_disabled_std_cycles;
  volatile uint32_t *attest_phase_ptr;
#if TDIS_RAW_DUMP
  uint32_t *raw_cycles;   /* NS buffer, length raw_cap */
  uint8_t  *raw_tag;      /* NS buffer, length raw_cap */
  uint32_t  raw_cap;
  uint32_t  raw_count;    /* filled by Secure */
#endif
} SECURE_AttestReport_t;

void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func);
void SECURE_LEDToggle(void);
/* NSC entry: at most 4 arguments (Arm CMSE — no stack args). */
void SECURE_ShuffledHMAC_secure(uint8_t *out_digest,
                                const uint8_t *challenge, size_t challenge_len,
                                SECURE_AttestReport_t *report);

#endif /* SECURE_NSC_H */
/* USER CODE END Non_Secure_CallLib_h */
