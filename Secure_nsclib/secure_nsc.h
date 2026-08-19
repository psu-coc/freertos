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

extern void *pSecureFaultCallback;
extern void *pSecureErrorCallback;

typedef enum
{
  SECURE_FAULT_CB_ID     = 0x00U,
  GTZC_ERROR_CB_ID       = 0x01U
} SECURE_CallbackIDTypeDef;

/** Written by Secure (NS RAM). attest_phase_ptr may point at g_attest_phase for live progress. */
typedef struct {
  uint32_t sau_config_avg_cycles;
  volatile uint32_t *attest_phase_ptr;
} SECURE_AttestReport_t;

void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func);
void SECURE_LEDToggle(void);
/* NSC entry: at most 4 arguments (Arm CMSE — no stack args). */
void SECURE_ShuffledHMAC_secure(uint8_t *out_digest,
                                const uint8_t *challenge, size_t challenge_len,
                                SECURE_AttestReport_t *report);

#endif /* SECURE_NSC_H */
/* USER CODE END Non_Secure_CallLib_h */
