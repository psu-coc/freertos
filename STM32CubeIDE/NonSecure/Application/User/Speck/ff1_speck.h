// ff1_speck.h
#ifndef FF1_SPECK_H
#define FF1_SPECK_H

#include <stdint.h>
#include <stddef.h>
#include "speck.h"

// ใช้ Speck เป็น block cipher ใน FF1
typedef struct {
    SimSpk_Cipher cipher;
} FF1_Key_Speck;

// เซ็ตคีย์ 128 บิตสำหรับ Speck128/128
void FF1_SetKey_Speck(FF1_Key_Speck *k, const uint8_t raw_key[16]);

// FF1-based permutation (FPE) ใช้ Speck เป็น core
uint32_t FF1Permute_Speck(uint32_t index,
                          uint32_t maxblocks,
                          const FF1_Key_Speck *key,
                          uint32_t tweak);

#endif
