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

void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func);
void SECURE_RegisterPrintCallback(void *callback);
void SECURE_Print(const char *msg);
void SECURE_LEDToggle(void);
void SECURE_ShuffledHMAC_secure(uint8_t *out_digest,
                                const uint8_t *challenge, size_t challenge_len,
                                uint32_t *out_sau_avg_cycles);

#endif /* SECURE_NSC_H */
/* USER CODE END Non_Secure_CallLib_h */
