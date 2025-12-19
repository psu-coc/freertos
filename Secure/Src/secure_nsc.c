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
#include "aes-gcm/aes.h"   // รวม AES CBC, CTR, ECB ไว้หมด
#include "Aesnew/aes.h"
#include "arm_cmse.h"

#define SHA256_DIGEST_SIZE 32
#define BLOCK_SIZE 4096 // // <--- แก้ตัวเลขตรงนี้ครับ (256, 512, 1024, 2048, 4096)
#define TOTAL_SIZE 0x40000
#define BLOCKS (TOTAL_SIZE / BLOCK_SIZE)

#define SAFE_FLASH_PAGE   128
#define TARGET_FLASH_ADDR 0x08068000 // Page 128 Bank 2


#define FLASH_TEST_ADDR_SECURE  0x0C0BE000  // Secure view
#define FLASH_TEST_ADDR_NS      0x080BE000  // Non-Secure view
#define FLASH_TEST_PAGE         120         // Bank 2, Page 120
#define FLASH_TEST_BANK         FLASH_BANK_2
#define FLASH_PAGE_SIZE         0x800       // 2KB per page
#define FLASH_TEST_DATA         0x1234567812345678ULL


// ⭐ เพิ่ม helper function ที่ top ของไฟล์
static inline void InvalidateICache(void)
{
    #if (__ICACHE_PRESENT == 1U)
    SCB->ICIALLU = 0UL;  // Invalidate entire I-Cache
    __DSB();
    __ISB();
    #endif
}

__attribute__((section(".gnu.linkonce.b._ns_work_buffer")))
static uint8_t ns_work_buffer[256] __attribute__((aligned(8)));

/** @addtogroup STM32L5xx_HAL_Examples
  * @{
  */

/** @addtogroup FreeRTOS_SecureIOToggle_TrustZone
  * @{
  */

/* Global variables ----------------------------------------------------------*/
void *pSecureFaultCallback = NULL;   /* Pointer to secure fault callback in Non-secure */
void *pSecureErrorCallback = NULL;   /* Pointer to secure error callback in Non-secure */
static void (*ns_print_cb)(const char *) = NULL;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



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

/**
  * @brief  Secure treatment of non-secure push button interrupt.
  * @retval None
  */
/*CMSE_NS_ENTRY*/secureportNON_SECURE_CALLABLE void SECURE_LEDToggle(void)
{
  BSP_LED_Toggle(LED1);
}

//CMSE_NS_ENTRY void SECURE_send(uint8_t* message, int size){
//	HAL_UART_Transmit(&hlpuart1, message, size, HAL_MAX_DELAY);
//};


/*CMSE_NS_ENTRY*/
//secureportNON_SECURE_CALLABLE const char* SECURE_GetMessage(void)
//{
//
//	return "AAA test in secure";
//}







//__attribute__((cmse_nonsecure_entry))
//int32_t SECURE_EraseProgramFlash(SecureFlashParams_t *params)
//{
//    HAL_StatusTypeDef status;
//    FLASH_EraseInitTypeDef EraseInitStruct = {0};
//    uint32_t PageError = 0;
//
//    // Safety: check pointer is from non-secure
//    if (!CMSE_IS_NONSECURE_ADDRESS(params)) return -100;
//
//    status = HAL_FLASH_Unlock();
//    if (status != HAL_OK) return -1;
//
//    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//    EraseInitStruct.Page = params->page;
//    EraseInitStruct.NbPages = 1;
//    EraseInitStruct.Banks = params->bank;
//
//    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
//    if (status != HAL_OK || PageError != 0xFFFFFFFF) {
//        HAL_FLASH_Lock();
//        return -2;
//    }
//    __DSB();
//    __ISB();
//
//    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, params->target_addr, params->value);
//
//    HAL_FLASH_Lock();
//
//    if (status == HAL_OK) return 0;
//    else return -3;
//}




__attribute__((cmse_nonsecure_entry))
void SECURE_CopyMessage(char* buffer, size_t maxlen)
{
    if (!buffer || maxlen == 0) return;
    const char* secure_msg = "AAA test in secure";
    strncpy(buffer, secure_msg, maxlen - 1);
    buffer[maxlen - 1] = '\0';
}


static const uint8_t secure_key[] = "MySecretKey";  // Fixed test key
static hmac_sha256 secure_hmac_ctx;
static uint8_t secure_digest[SHA256_DIGEST_SIZE];


__attribute__((cmse_nonsecure_entry))
void SECURE_ComputeHMAC(uint8_t *output_digest, size_t maxlen)
{

    if (!output_digest || maxlen < SHA256_DIGEST_SIZE) return;

    __disable_irq();

    for (volatile uint32_t i = 0; i < 10000000; i++);
	__enable_irq();

    const uint8_t message[] = "Temp Temp from Secure World!";

    hmac_sha256_initialize(&secure_hmac_ctx, secure_key, strlen((const char*)secure_key));
//    hmac_sha256_update(&secure_hmac_ctx, message, strlen((const char*)message));
    hmac_sha256_finalize(&secure_hmac_ctx, NULL, 0);

    for (volatile uint32_t i = 0; i < 10000000; i++);

    memcpy(secure_digest, secure_hmac_ctx.digest, SHA256_DIGEST_SIZE);


	memcpy(output_digest, secure_digest, SHA256_DIGEST_SIZE);


}

static uint8_t secure_digest_2[SHA256_DIGEST_SIZE];
uint8_t *real_memory = (uint8_t *)0x8040000;
uint8_t seed[32] = {0};
//uint8_t real_memory[TOTAL_SIZE];  // This is safe, real RAM

static const uint8_t key[] = "MySecureKey123"; // Example key
static hmac_sha256 hmac;
static uint8_t memory_region[1024];     // Simulate memory to attest
static uint8_t secure_digest_1[SHA256_DIGEST_SIZE];



__attribute__((cmse_nonsecure_entry))
void SECURE_LinearHMAC(uint8_t *output_digest, size_t maxlen)
{


//    __disable_irq();
    if (!output_digest || maxlen < SHA256_DIGEST_SIZE) {
        return;
    }

    // Optional: Fill real_memory with known data for consistent testing
//    for (int i = 0; i < TOTAL_SIZE; i++) {
//        real_memory[i] = i & 0xFF;  // test pattern
//    }

//    hmac_sha256_initialize(&hmac, key, strlen((const char *)key));
    hmac_sha256_initialize(&hmac, key, sizeof(key));


    // Process blocks sequentially (not shuffled)
    hmac_sha256_update(&hmac, real_memory, TOTAL_SIZE);

//    for (int i = 0; i < BLOCKS; i++) {
//        const uint8_t *block = &real_memory[i * BLOCK_SIZE];
//        hmac_sha256_update(&hmac, block, BLOCK_SIZE);
//    }

    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(output_digest, hmac.digest, SHA256_DIGEST_SIZE);


}


__attribute__((cmse_nonsecure_entry))
void SECURE_ShuffledHMAC(uint8_t *output_digest, size_t maxlen)
{

	  if (!output_digest || maxlen < SHA256_DIGEST_SIZE) return;

	    static int indices[BLOCKS];
	    for (int i = 0; i < BLOCKS; i++) indices[i] = i;
	    // Simple shuffle without srand for now


	      srand(42);

	      for (int i = BLOCKS - 1; i > 0; i--) {
	          int j = rand() % (i + 1);
	          int tmp = indices[i];
	          indices[i] = indices[j];
	          indices[j] = tmp;
	      }

	    hmac_sha256_initialize(&hmac, key, strlen((const char *)key));
	    for (int i = 0; i < BLOCKS; i++) {
	        const uint8_t *block = &real_memory[indices[i] * BLOCK_SIZE];
	        __disable_irq();
	        hmac_sha256_update(&hmac, block, BLOCK_SIZE);
	        __enable_irq();
	    }
	    hmac_sha256_finalize(&hmac, NULL, 0);
	    memcpy(output_digest, hmac.digest, SHA256_DIGEST_SIZE);



}

//static void derive_key16_from_challenge(uint8_t out16[16],
//                                        const uint8_t *challenge, size_t clen)
//{
//    // K = HMAC_SHA256(secret_key, challenge || fallback)
//    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen(key));
//    if (challenge && clen) {
//        hmac_sha256_update(&hmac, challenge, clen);
//    } else {
//        // fallback: add some device-side entropy so it's never all-zero
//        uint32_t tick = (uint32_t)SysTick->VAL;
//        hmac_sha256_update(&hmac, (uint8_t*)&tick, sizeof(tick));
//    }
//    hmac_sha256_finalize(&hmac, NULL, 0);
//
//    // Use first 16 bytes as AES-128 key
//    memcpy(out16, hmac.digest, 16);
//}
//
//static int prng_uniform_int_ctr(int n,
//                                const uint8_t aes_key16[16],
//                                uint32_t *counter_io)
//{
//    // rejection sampling to avoid modulo bias
//    const uint32_t limit = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
//
//    for (;;) {
//        uint8_t nonce[16] = {0};    // CTR nonce/IV
//        uint8_t block[16] = {0};    // plaintext buffer (zero) -> keystream
//
//        // put the counter into the first 4 bytes of nonce (little-endian is fine)
//        memcpy(nonce, counter_io, sizeof(*counter_io));
//
//        // aes_128_ctr_encrypt encrypts 'block' in place using key+nonce
//        // We only need 16 bytes of keystream for one sample
//        uint8_t key_local[16];
//        memcpy(key_local, aes_key16, 16);  // API expects non-const in some builds
//        // encrypt 16 bytes -> block now contains keystream
//        if (aes_128_ctr_encrypt(key_local, nonce, block, sizeof(block)) != 0) {
//            // if your AES returns non-zero on error, decide how to handle
//            return 0;
//        }
//
//        // interpret first 4 bytes as random 32-bit
//        uint32_t rnd;
//        memcpy(&rnd, block, sizeof(rnd));
//        (*counter_io)++;
//
//        if (rnd < limit) {
//            return (int)(rnd % (uint32_t)n);
//        }
//        // else: try again
//    }
//}
//
//static void shuffle_secure_aes_ctr(int *arr, int n,
//                                   const uint8_t aes_key16[16])
//{
//    uint32_t ctr = 0;  // start counter at 0; increments each sample
//    for (int i = n - 1; i > 0; i--) {
//        int j = prng_uniform_int_ctr(i + 1, aes_key16, &ctr);
//        int tmp = arr[i];
//        arr[i] = arr[j];
//        arr[j] = tmp;
//    }
//}
//
//
//__attribute__((cmse_nonsecure_entry))
//void SECURE_ShuffledHMAC_secure(uint8_t *out_digest, size_t out_len,
//                                const uint8_t *challenge, size_t challenge_len)
//{
//
//    if (!out_digest || out_len < SHA256_DIGEST_SIZE) return;
//
//    // 1) indices = 0..BLOCKS-1
//    static int indices[BLOCKS];
//    for (int i = 0; i < BLOCKS; i++) indices[i] = i;
//
//    // 2) derive AES-128 key from HMAC(secret, challenge)
//    uint8_t aes_key16[16];
//    derive_key16_from_challenge(aes_key16, challenge, challenge_len);
//
//    // 3) secure shuffle (unpredictable without secret key)
//    shuffle_secure_aes_ctr(indices, BLOCKS, aes_key16);
//
//    // 4) HMAC over shuffled blocks
//    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen(key));
//    for (int i = 0; i < BLOCKS; i++) {
//        const uint8_t *block = &real_memory[(size_t)indices[i] * BLOCK_SIZE];
////        __disable_irq();
//        hmac_sha256_update(&hmac, block, BLOCK_SIZE);
////        __enable_irq();
//    }
//    hmac_sha256_finalize(&hmac, NULL, 0);
//    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
//
//}

// ---- Key/IV derivation: HMAC(secret, challenge) -> 32B -> 16B key + 16B iv
static void derive_aes_key_iv_from_challenge(uint8_t key16[16],
                                             uint8_t iv16[16],
                                             const uint8_t *challenge,
                                             size_t clen)
{
    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen(key));
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
void SECURE_ShuffledHMAC_secure(uint8_t *out_digest, size_t out_len,
                                const uint8_t *challenge, size_t challenge_len)
{
    if (!out_digest || out_len < SHA256_DIGEST_SIZE) return;

    // 1) indices = 0..BLOCKS-1
    static int indices[BLOCKS];
    for (int i = 0; i < BLOCKS; i++) indices[i] = i;

    // 2) derive AES key/IV from challenge
    uint8_t key16[16], iv16[16];
    derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);

    // 3) secure shuffle
    shuffle_secure_aes_ctr(indices, BLOCKS, key16, iv16);

    // 4) HMAC over shuffled blocks
    hmac_sha256_initialize(&hmac, (const uint8_t*)key, strlen(key));
    uint8_t copy[BLOCK_SIZE];
    for (int i = 0; i < BLOCKS; i++) {
        const uint8_t *blk = &real_memory[(size_t)indices[i] * BLOCK_SIZE];
        __disable_irq();
        memcpy(copy, blk, BLOCK_SIZE);
//        hmac_sha256_update(&hmac, blk, BLOCK_SIZE);
        __enable_irq();
        hmac_sha256_update(&hmac, copy, BLOCK_SIZE);
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out_digest, hmac.digest, SHA256_DIGEST_SIZE);
}


__attribute__((cmse_nonsecure_entry))
void SECURE_SMARM(uint8_t *output_digest, size_t maxlen)
{
	if (!output_digest || maxlen < SHA256_DIGEST_SIZE) return;

    srand(42);  // Use fixed seed or hardware-derived one like SysTick->VAL

	// Generate shuffled indices

	int indices[TOTAL_SIZE];

	//	    for (int i = 0; i < BLOCKS; i++) indices[i] = i;

	// 128 bit challenge -> sha256_update(challenge); finalize(K) -> seed (256-bit) // H(challenge || K)

	// Use seed for AES-CTR PRNG

	// note #include <aes/aes_cbc.h> น่าจะต้องหา lib

	// ben shuffle_aes_ctr ลองใช้ตัวนี้อยู๋ใน main.c -> shuffle.c

	for (int i = TOTAL_SIZE - 1; i > 0; i--) {

	int j = rand() % (i + 1);

	int tmp = indices[i];

	indices[i] = indices[j];

	indices[j] = tmp;

	}

	hmac_sha256_initialize(&hmac, key, strlen((const char )key));

	for (int i = 0; i < BLOCKS; i++) {

	const uint8_t *block = &real_memory[indices[i] * BLOCK_SIZE];

		__disable_irq();
		hmac_sha256_update(&hmac, block, BLOCK_SIZE);
		__enable_irq();

	}

	hmac_sha256_finalize(&hmac, NULL, 0);

	memcpy(output_digest,hmac.digest, SHA256_DIGEST_SIZE);
}




__attribute__((cmse_nonsecure_entry))
void SECURE_TEST(uint8_t *output_digest, size_t maxlen)
{
	 if (!output_digest || maxlen < SHA256_DIGEST_SIZE) return;

	    // Fill real_memory ifจำเป็น (ทำแบบ linear)
	    for (int i = 0; i < TOTAL_SIZE * BLOCK_SIZE; i++) {
	        real_memory[i] = i & 0xFF;
	    }

	    // 1. Initialize indices
	    int indices[BLOCKS];
	    for (int i = 0; i < BLOCKS; i++) indices[i] = i;

	    srand(42);

	    // 2. Shuffle indices
	    for (int i = BLOCKS - 1; i > 0; i--) {
	        int j = rand() % (i + 1);
	        int tmp = indices[i];
	        indices[i] = indices[j];
	        indices[j] = tmp;
	    }

	    hmac_sha256_initialize(&hmac, key, strlen((const char *)key));

	    for (int i = 0; i < BLOCKS; i++) {
	        const uint8_t *block = &real_memory[indices[i] * BLOCK_SIZE];
	        hmac_sha256_update(&hmac, block, BLOCK_SIZE);
	    }

	    hmac_sha256_finalize(&hmac, NULL, 0);
	    memcpy(output_digest, hmac.digest, SHA256_DIGEST_SIZE);
}


static void LocalTest_HMAC(void)
{
    const uint8_t message[] = "Hello from Secure World!";
    const uint8_t key[] = "MySecretKey";
    uint8_t digest[32];

    hmac_sha256_get(digest, message, strlen((const char*)message), key, strlen((const char*)key));

    // Output result via Print callback if available
    if (ns_print_cb)
    {
        SECURE_Print("[Secure] HMAC result:");
        for (int i = 0; i < 32; i++)
        {
            char buf[8];
            snprintf(buf, sizeof(buf), " %02X", digest[i]);
            SECURE_Print(buf);
        }
        SECURE_Print("\r\n");
    }

    // Or just toggle LED
    BSP_LED_Toggle(LED1);
}



//__attribute__((cmse_nonsecure_entry))
//void SECURE_ComputeHMAC(uint8_t *output_digest, size_t maxlen)
//{
//    if (!output_digest || maxlen < SHA256_DIGEST_SIZE) {
//        return;
//    }
//
//    const uint8_t message[] = "Hello from Secure World!";
//    const uint8_t key[] = "MySecretKey";
//
//    hmac_sha256 hmac;
//
//    // 1️⃣ Initialize with key
//    hmac_sha256_initialize(&hmac, key, strlen((const char*)key));
//
//    // 2️⃣ Update with message
//    hmac_sha256_update(&hmac, message, strlen((const char*)message));
//
//    // 3️⃣ Finalize to compute digest
//    hmac_sha256_finalize(&hmac, NULL, 0);
//
//    // 4️⃣ Copy digest back to NonSecure buffer
//    memcpy(output_digest, hmac.digest, SHA256_DIGEST_SIZE);
//}

// Ben : old secure flashtest
//__attribute__((cmse_nonsecure_entry))
//
//void Secure_FlashTest(void)
//
//{
//
//HAL_StatusTypeDef status;
//
//FLASH_EraseInitTypeDef EraseInitStruct = {0};
//
//FLASH_BBAttributesTypeDef flash_bb_attr = {0};
//
//uint32_t PageError = 0;
//
//uint64_t test_data = FLASH_TEST_DATA;
//
//uint32_t saved_attr = 0;
//
//
//
//printf("[Secure] === Flash Test with BB Attribute Management ===\r\n");
//
//
//
//// ========== Step 1: Get Current Block Attributes ==========
//
//flash_bb_attr.Bank = FLASH_TEST_BANK;
//
//flash_bb_attr.BBAttributesType = FLASH_BB_SEC;
//
//
//
//status = HAL_FLASHEx_GetConfigBBAttributes(&flash_bb_attr);
//
//if (status != HAL_OK)
//
//{
//
//printf("[Secure] ❌ Get BB Attributes FAILED: %d\r\n", status);
//
//return;
//
//}
//
//
//
//// Save current attribute for page 120
//
//// Page 120 = bit 120 in BBAttributes_array
//
//uint32_t attr_index = FLASH_TEST_PAGE / 32; // which uint32 (120/32 = 3)
//
//uint32_t attr_bit = FLASH_TEST_PAGE % 32; // which bit (120%32 = 24)
//
//
//
//saved_attr = flash_bb_attr.BBAttributes_array[attr_index];
//
//uint32_t page_is_secure = (saved_attr >> attr_bit) & 0x1;
//
//
//
//printf("[Secure] Page %d current attr: %s\r\n",
//
//FLASH_TEST_PAGE,
//
//page_is_secure ? "SECURE" : "NON-SECURE");
//
//
//
//// ========== Step 2: Promote Page to SECURE ==========
//
//printf("[Secure] Promoting page to SECURE...\r\n");
//
//
//
//// Set bit to make page Secure
//
//flash_bb_attr.BBAttributes_array[attr_index] |= (1U << attr_bit);
//
//
//
//status = HAL_FLASHEx_ConfigBBAttributes(&flash_bb_attr);
//
//if (status != HAL_OK)
//
//{
//
//printf("[Secure] ❌ Promote to SECURE FAILED: %d\r\n", status);
//
//return;
//
//}
//
//
//
//printf("[Secure] ✅ Page promoted to SECURE\r\n");
//
//
//
//// ========== Step 3: Unlock Flash ==========
//
//status = HAL_FLASH_Unlock();
//
//if (status != HAL_OK)
//
//{
//
//printf("[Secure] ❌ Unlock FAILED: %d\r\n", status);
//
//goto restore_attributes;
//
//}
//
//printf("[Secure] ✅ Unlocked\r\n");
//
//
//
//// ========== Step 4: Erase Page ==========
//
//EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//
//EraseInitStruct.Page = FLASH_TEST_PAGE;
//
//EraseInitStruct.NbPages = 1;
//
//EraseInitStruct.Banks = FLASH_TEST_BANK;
//
//
//
//status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
//
//
//
//if (status != HAL_OK || PageError != 0xFFFFFFFF)
//
//{
//
//printf("[Secure] ❌ Erase FAILED: status=%d, PageError=0x%08lX\r\n",
//
//status, PageError);
//
//HAL_FLASH_Lock();
//
//goto restore_attributes;
//
//}
//
//printf("[Secure] ✅ Erase OK\r\n");
//
//
//
//// ========== Step 5: Program ==========
//
//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
//
//FLASH_TEST_ADDR_SECURE,
//
//test_data);
//
//
//
//if (status != HAL_OK)
//
//{
//
//printf("[Secure] ❌ Program FAILED: status=%d, error=0x%08lX\r\n",
//
//status, HAL_FLASH_GetError());
//
//HAL_FLASH_Lock();
//
//goto restore_attributes;
//
//}
//
//printf("[Secure] ✅ Program OK\r\n");
//
//
//
//// ========== Step 6: Lock Flash ==========
//
//HAL_FLASH_Lock();
//
//
//
//// ========== Step 7: Verify (while still Secure) ==========
//
//__DSB();
//
//__ISB();
//
//
//
//uint64_t verify_value = *(volatile uint64_t*)FLASH_TEST_ADDR_SECURE;
//
//
//
//if (verify_value == test_data)
//
//{
//
//printf("[Secure] ✅ Verify OK!\r\n");
//
//printf("[Secure] Written: 0x%016llX\r\n", test_data);
//
//printf("[Secure] Read: 0x%016llX\r\n", verify_value);
//
//}
//
//else
//
//{
//
//printf("[Secure] ❌ Verify FAILED!\r\n");
//
//printf("[Secure] Expected: 0x%016llX\r\n", test_data);
//
//printf("[Secure] Got: 0x%016llX\r\n", verify_value);
//
//}
//
//
//
//restore_attributes:
//
//// ========== Step 8: Restore Original Attributes ==========
//
//printf("[Secure] Restoring original attributes...\r\n");
//
//
//
//// Restore saved attribute
//
//flash_bb_attr.BBAttributes_array[attr_index] = saved_attr;
//
//
//
//status = HAL_FLASHEx_ConfigBBAttributes(&flash_bb_attr);
//
//if (status != HAL_OK)
//
//{
//
//printf("[Secure] ⚠️ WARNING: Failed to restore attributes: %d\r\n", status);
//
//printf("[Secure] ⚠️ Page may remain SECURE!\r\n");
//
//}
//
//else
//
//{
//
//printf("[Secure] ✅ Attributes restored to %s\r\n",
//
//page_is_secure ? "SECURE" : "NON-SECURE");
//
//}
//
//
//
//// ========== Step 9: Invalidate Caches ==========
//
//// Clear instruction cache
//
//SCB_InvalidateICache();
//
//
//
//// Memory barrier
//
//__DSB();
//
//__ISB();
//
//
//
//printf("[Secure] 🎉 Flash Test Complete!\r\n");
//
//}

__attribute__((cmse_nonsecure_entry))
void Secure_FlashTest(void)
{
	 	HAL_StatusTypeDef status;
	    FLASH_EraseInitTypeDef EraseInitStruct = {0};
	    FLASH_BBAttributesTypeDef flash_bb_attr = {0};
	    uint32_t PageError = 0;
	    uint64_t test_data = FLASH_TEST_DATA;
	    uint32_t saved_attr = 0;

	    printf("[Secure] === Flash Test (with BB Attributes) ===\r\n");

	    // ========== Step 1: Get Current BB Attributes ==========
	    flash_bb_attr.Bank = FLASH_TEST_BANK;
	    flash_bb_attr.BBAttributesType = FLASH_BB_SEC;

	    HAL_FLASHEx_GetConfigBBAttributes(&flash_bb_attr);

	    uint32_t attr_index = FLASH_TEST_PAGE / 32;  // 120/32 = 3
	    uint32_t attr_bit = FLASH_TEST_PAGE % 32;    // 120%32 = 24

	    saved_attr = flash_bb_attr.BBAttributes_array[attr_index];
	    uint32_t page_is_secure = (saved_attr >> attr_bit) & 0x1;

	    printf("[Secure] Page %d current: %s\r\n",
	           FLASH_TEST_PAGE,
	           page_is_secure ? "SECURE" : "NON-SECURE");

	    // ========== Step 2: Promote Page to SECURE ==========
	    printf("[Secure] Promoting to SECURE...\r\n");

	    // Set bit to make page Secure
	    flash_bb_attr.BBAttributes_array[attr_index] |= (1U << attr_bit);

	    status = HAL_FLASHEx_ConfigBBAttributes(&flash_bb_attr);
	    if (status != HAL_OK)
	    {
	        printf("[Secure] ❌ Promote FAILED: %d\r\n", status);
	        return;
	    }

	    printf("[Secure] ✅ Promoted to SECURE\r\n");

	    // Small delay for attribute change to take effect
//	    for (volatile int i = 0; i < 1000; i++);

	    // ========== Step 3: Unlock Flash ==========
	    status = HAL_FLASH_Unlock();
	    if (status != HAL_OK)
	    {
	        printf("[Secure] ❌ Unlock FAILED: %d\r\n", status);
	        goto restore_attributes;
	    }
	    printf("[Secure] ✅ Unlocked\r\n");

	    // ========== Step 4: Erase Page ==========
	    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	    EraseInitStruct.Page = FLASH_TEST_PAGE;
	    EraseInitStruct.NbPages = 1;
	    EraseInitStruct.Banks = FLASH_TEST_BANK;

	    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	    if (status != HAL_OK || PageError != 0xFFFFFFFF)
	    {
	        printf("[Secure] ❌ Erase FAILED: status=%d, PageError=0x%08lX\r\n",
	               status, PageError);
	        HAL_FLASH_Lock();
	        goto restore_attributes;
	    }
	    printf("[Secure] ✅ Erase OK\r\n");

	    // ========== Step 5: Prepare Data in Work Buffer ==========
	    // Copy data to NS work buffer (required by some HAL implementations)
	    memcpy(ns_work_buffer, &test_data, sizeof(test_data));

	    // Memory barrier
	    __DSB();
	    __ISB();

	    // ========== Step 6: Program via Secure Address ==========
	    printf("[Secure] Programming...\r\n");

	    // Use Secure alias address (0x0C...)
	    uint64_t data_to_write;
	    memcpy(&data_to_write, ns_work_buffer, sizeof(data_to_write));

	    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
	                                FLASH_TEST_ADDR_SECURE,  // Secure alias
	                                data_to_write);

	    if (status != HAL_OK)
	    {
	        uint32_t flash_error = HAL_FLASH_GetError();
	        printf("[Secure] ❌ Program FAILED\r\n");
	        printf("[Secure]    Status: %d\r\n", status);
	        printf("[Secure]    Error: 0x%08lX\r\n", flash_error);
	        printf("[Secure]    SECSR: 0x%08lX\r\n", FLASH->SECSR);
	        HAL_FLASH_Lock();
	        goto restore_attributes;
	    }

	    printf("[Secure] ✅ Program OK\r\n");

	    // ========== Step 7: Lock Flash ==========
	    HAL_FLASH_Lock();

	    // ========== Step 8: Verify (while still Secure) ==========
	    // Clean and invalidate cache
	    __DSB();
	    __ISB();

	    uint64_t verify_value = *(volatile uint64_t*)FLASH_TEST_ADDR_SECURE;

	    if (verify_value == test_data)
	    {
	        printf("[Secure] ✅ Verify OK!\r\n");
	        printf("[Secure]    Written:  0x%016llX\r\n", test_data);
	        printf("[Secure]    Read:     0x%016llX\r\n", verify_value);
	    }
	    else
	    {
	        printf("[Secure] ❌ Verify FAILED!\r\n");
	        printf("[Secure]    Expected: 0x%016llX\r\n", test_data);
	        printf("[Secure]    Got:      0x%016llX\r\n", verify_value);
	    }

	restore_attributes:
	    // ========== Step 9: Demote back to Original Attributes ==========
	    printf("[Secure] Restoring attributes...\r\n");

	    // Restore original attribute
	    flash_bb_attr.BBAttributes_array[attr_index] = saved_attr;

	    status = HAL_FLASHEx_ConfigBBAttributes(&flash_bb_attr);
	    if (status != HAL_OK)
	    {
	        printf("[Secure] ⚠️  WARNING: Failed to restore: %d\r\n", status);
	        printf("[Secure] ⚠️  Page may remain SECURE!\r\n");
	    }
	    else
	    {
	        printf("[Secure] ✅ Attributes restored\r\n");
	    }

	    // ========== Step 10: Invalidate Caches ==========
	    __DSB();
	    __ISB();

	    printf("[Secure] 🎉 Flash Test Complete!\r\n");
}

// ✅ Wrapper function ที่บังคับใช้ Non-Secure register
static void FLASH_PageErase_NS(uint32_t Page, uint32_t Banks)
{
    // Wait for Flash ready
    while (FLASH->NSSR & FLASH_NSSR_NSBSY);

    // Set bank
    if (Banks == FLASH_BANK_2) {
        SET_BIT(FLASH->NSCR, FLASH_NSCR_NSBKER);
    } else {
        CLEAR_BIT(FLASH->NSCR, FLASH_NSCR_NSBKER);
    }

    // Set page number and enable page erase
    MODIFY_REG(FLASH->NSCR,
               (FLASH_NSCR_NSPNB | FLASH_NSCR_NSPER),
               ((Page << FLASH_NSCR_NSPNB_Pos) | FLASH_NSCR_NSPER));

    // Start erase
    SET_BIT(FLASH->NSCR, FLASH_NSCR_NSSTRT);
}



//Ben try to delete direcly
__attribute__((cmse_nonsecure_entry))
void Secure_EraseWriteVerify(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;
    uint64_t test_value = 0x123456789ABCDEF0ULL;

    uint32_t target_page = 127;
    uint32_t target_addr = 0x0807F800;

    volatile uint32_t debug_step = 0;
    volatile uint32_t debug_match = 0;
    volatile uint32_t debug_notmatch = 0;

    HAL_ICACHE_Disable();
    debug_step = 1;

    // ========== Unlock NS ==========
    FLASH->NSKEYR = FLASH_KEY1;
    FLASH->NSKEYR = FLASH_KEY2;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        debug_step = 10; // Unlock failed
        HAL_ICACHE_Enable();
        return;
    }

    // ========== ✅ บังคับให้ HAL คิดว่าเป็น Non-Secure operation ==========
    extern FLASH_ProcessTypeDef pFlash;
    pFlash.ProcedureOnGoing = FLASH_TYPEERASE_PAGES;  // Non-Secure operation

    FLASH->NSSR = 0xFFFFFFFF;  // Clear all flags
    __DSB();
    __ISB();

    // ========== Erase ด้วย HAL ==========
//    debug_step = 2;
//
//    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//    EraseInitStruct.Page = target_page;
//    EraseInitStruct.NbPages = 1;
//    EraseInitStruct.Banks = FLASH_BANK_2;
//
//    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
//
//    if (status != HAL_OK || PageError != 0xFFFFFFFF) {
//        debug_step = 20;  // Erase failed
//        SET_BIT(FLASH->NSCR, FLASH_NSCR_NSLOCK);
//        HAL_ICACHE_Enable();
//        return;
//    }

    debug_step = 3;  // Erase OK

    __DSB();
    __ISB();

    // ========== Program 256 words ==========
    debug_step = 4;

    for (uint32_t i = 0; i < 256; i++) {
        uint32_t addr = target_addr + (i * 8);
        uint64_t value = test_value + i;

        while (FLASH->NSSR & FLASH_NSSR_NSBSY);

        // Clear any errors
        FLASH->NSSR = 0xFFFFFFFF;

        SET_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);

        *(volatile uint32_t*)addr = (uint32_t)(value);
        __ISB();
        *(volatile uint32_t*)(addr + 4) = (uint32_t)(value >> 32);

        while (FLASH->NSSR & FLASH_NSSR_NSBSY);

        CLEAR_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);
    }

    __DSB();
    __ISB();

    // ========== Invalidate Cache ==========
    debug_step = 5;
    HAL_ICACHE_Invalidate();
    __DSB();
    __ISB();

    // ========== Verify ==========
    debug_step = 6;

    for (uint32_t j = 0; j < 256; j++) {
        uint32_t addr = target_addr + (j * 8);
        uint64_t expected = test_value + j;
        uint64_t actual = *(volatile uint64_t*)addr;

        if (actual == expected) {
            debug_match++;
        } else {
            debug_notmatch++;
        }
    }

    SET_BIT(FLASH->NSCR, FLASH_NSCR_NSLOCK);
    HAL_ICACHE_Enable();

    if (debug_match == 256) {
        debug_step = 100;  // ✅ Success
    } else {
        debug_step = 101;  // ❌ Failed
    }
}





__attribute__((cmse_nonsecure_entry))
void Simulate_flash_write_128KB(uint32_t start_addr, uint8_t start_val, uint32_t *success)
{
    int num_pages = 4;
    uint32_t page_size = FLASH_PAGE_SIZE;  // 2KB = 0x800

    // Initialize as failed
    *success = 0;

    static  uint8_t data[FLASH_PAGE_SIZE];
    for(int i = 0; i < 2048; i++) {
        data[i] = i + start_val;
    }
    HAL_ICACHE_Disable();

    // Unlock NS Flash
    FLASH->NSKEYR = FLASH_KEY1;
    FLASH->NSKEYR = FLASH_KEY2;

    if (FLASH->NSCR & FLASH_NSCR_NSLOCK) {
        HAL_ICACHE_Enable();
        return;  // Flash unlock failed
    }

    // Force Non-Secure operation
    extern FLASH_ProcessTypeDef pFlash;
    pFlash.ProcedureOnGoing = FLASH_TYPEERASE_PAGES;

    FLASH->NSSR = 0xFFFFFFFF;
    __DSB();
    __ISB();

    // ========== Write 4 pages (8KB total) ==========


    	uint32_t end_addr = start_addr + (num_pages * page_size) - 8;  // 8KB - 8 bytes
       uint32_t word_index = 0;

       for (uint32_t addr = start_addr; addr <= end_addr; addr += 8) {
           // Prepare 8-byte value from data buffer
           uint32_t offset = (addr - start_addr) % page_size;  // Offset within current page
           uint64_t value = 0;
           for (int b = 0; b < 8; b++) {
               value |= ((uint64_t)data[offset + b]) << (b * 8);
           }

           while (FLASH->NSSR & FLASH_NSSR_NSBSY);
           FLASH->NSSR = 0xFFFFFFFF;
           SET_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);

           *(volatile uint32_t*)addr = (uint32_t)(value);
           __ISB();
           *(volatile uint32_t*)(addr + 4) = (uint32_t)(value >> 32);

           while (FLASH->NSSR & FLASH_NSSR_NSBSY);
           CLEAR_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);

           word_index++;
       }

    __DSB();
    __ISB();

    // Lock flash
    SET_BIT(FLASH->NSCR, FLASH_NSCR_NSLOCK);

    // Invalidate Cache
    HAL_ICACHE_Invalidate();
    __DSB();
    __ISB();

    HAL_ICACHE_Enable();

    // ========== Verify written data ==========
    static uint8_t buf[FLASH_PAGE_SIZE];

    for (int page = 0; page < num_pages; page++) {
        uint32_t page_addr = start_addr + (page * page_size);

        // Read back one page
        for (uint32_t i = 0; i < page_size; i++) {
            buf[i] = *(volatile uint8_t*)(page_addr + i);
        }

        // Verify
        for (uint32_t i = 0; i < page_size; i++) {
            if (buf[i] != data[i]) {
                // Data mismatch - return failure
                return;  // success is still 0
            }
        }
    }

    // All verification passed
    *success = 1;
}


__attribute__((cmse_nonsecure_entry))
void Secure_WriteFlash_128KB(uint32_t *success_words, uint32_t *failed_words)
{
    uint64_t test_value = 0x123456789ABCDEF0ULL;

    // ✅ Pages 64-127 (128KB)
    uint32_t start_addr = 0x08060000;  // Page 64
    uint32_t end_addr   = 0x0807FFFF;  // Page 127
    uint32_t total_words = (end_addr - start_addr + 1) / 8;  // 16,384 words

    uint32_t success = 0;
    uint32_t failed = 0;

    HAL_ICACHE_Disable();

    // Unlock NS Flash
    FLASH->NSKEYR = FLASH_KEY1;
    FLASH->NSKEYR = FLASH_KEY2;

    if (FLASH->NSCR & FLASH_NSCR_NSLOCK) {
        HAL_ICACHE_Enable();
        *success_words = 0;
        *failed_words = total_words;
        return;
    }

    // Force Non-Secure operation
    extern FLASH_ProcessTypeDef pFlash;
    pFlash.ProcedureOnGoing = FLASH_TYPEERASE_PAGES;

    FLASH->NSSR = 0xFFFFFFFF;
    __DSB();
    __ISB();

    // ========== Write 128KB ==========
    uint32_t word_index = 0;

    for (uint32_t addr = start_addr; addr <= end_addr; addr += 8) {
        uint64_t value = test_value + word_index;

        while (FLASH->NSSR & FLASH_NSSR_NSBSY);
        FLASH->NSSR = 0xFFFFFFFF;
        SET_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);

        *(volatile uint32_t*)addr = (uint32_t)(value);
        __ISB();
        *(volatile uint32_t*)(addr + 4) = (uint32_t)(value >> 32);

        while (FLASH->NSSR & FLASH_NSSR_NSBSY);
        CLEAR_BIT(FLASH->NSCR, FLASH_NSCR_NSPG);

        word_index++;
    }

    __DSB();
    __ISB();

    // Invalidate Cache
    HAL_ICACHE_Invalidate();
    __DSB();
    __ISB();

    // ========== Verify ==========
    word_index = 0;

    for (uint32_t addr = start_addr; addr <= end_addr; addr += 8) {
        uint64_t expected = test_value + word_index;
        uint64_t actual = *(volatile uint64_t*)addr;

        if (actual == expected) {
            success++;
        } else {
            failed++;
        }

        word_index++;
    }

    SET_BIT(FLASH->NSCR, FLASH_NSCR_NSLOCK);
    HAL_ICACHE_Enable();

    *success_words = success;
    *failed_words = failed;
}






__attribute__((cmse_nonsecure_entry))
void Secure_Flash256KB(FlashResult_t *result)
{
	// just run 10 pages
    if (!result) return;

    // ⭐ ใช้ Pages 128-137 (Non-secure area, แต่ Secure World เขียนได้)
    const uint32_t START_PAGE = 246;
    const uint32_t TOTAL_PAGES = 10;   // แค่ 10 pages = 20 KB
    const uint32_t PAGE_SIZE = 0x800;
    const uint32_t BANK2_BASE = 0x08040000;
    const uint32_t DW_PER_PAGE = 256;
    const uint64_t FLASH_TEST_DATA_BASE = 0x1234567890ABCDEFULL;

    HAL_StatusTypeDef status;
    uint32_t PageError = 0;

    result->total_pages = TOTAL_PAGES;
    result->success_pages = 0;
    result->failed_pages = 0;
    result->total_words = 0;
    result->success_words = 0;
    result->failed_words = 0;

    // --- BB Attribute Management Variables ---
    FLASH_BBAttributesTypeDef flash_bb_attr = {0};
    uint32_t saved_attr_array[4] = {0}; // Bank 2 has 128 pages, 128/32 = 4 uint32_t values
    uint8_t original_page_is_secure[TOTAL_PAGES];

    flash_bb_attr.Bank = FLASH_BANK_2;
    flash_bb_attr.BBAttributesType = FLASH_BB_SEC;
    HAL_FLASHEx_GetConfigBBAttributes(&flash_bb_attr);

    if (status != HAL_OK) {
            // Cannot proceed if we can't read attributes
        return;
    }

    memcpy(saved_attr_array, flash_bb_attr.BBAttributes_array, sizeof(saved_attr_array));


    // --- Step 2: Promote all target pages to SECURE ---
       for (uint32_t i = 0; i < TOTAL_PAGES; i++) {
           uint32_t page = START_PAGE + i;
           uint32_t attr_index = page / 32;
           uint32_t attr_bit = page % 32;

           // Check if it's already secure
           if (flash_bb_attr.BBAttributes_array[attr_index] & (1U << attr_bit)) {
               original_page_is_secure[i] = 1;
           } else {
               original_page_is_secure[i] = 0;
               // Set bit to make page Secure
               flash_bb_attr.BBAttributes_array[attr_index] |= (1U << attr_bit);
           }
       }

       // Apply the new (all-secure) attributes
          status = HAL_FLASHEx_ConfigBBAttributes(&flash_bb_attr);
          if (status != HAL_OK) {
              // Failed to promote pages, cannot proceed
              return;
          }

    uint32_t start_time = HAL_GetTick();

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        result->time_ms = HAL_GetTick() - start_time;
        return;
    }

    // ⭐ Loop Pages 128-137
    for (uint32_t i = 0; i < TOTAL_PAGES; i++) {
        uint32_t page = START_PAGE + i;
        uint32_t page_addr = BANK2_BASE + (page * PAGE_SIZE);
        uint8_t page_ok = 1;


        // Erase page

        // ตรวจสอบว่า Flash ไม่ได้ทำงานอยู่ก่อน
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {};

        // ตรวจสอบและล้าง Error Flag ที่อาจค้างอยู่จากการทำงานก่อนหน้า
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ALL_ERRORS)) {
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        }


        FLASH_EraseInitTypeDef EraseInit = {0};
        EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInit.Banks = FLASH_BANK_2;
        EraseInit.Page = page;
        EraseInit.NbPages = 1;

        status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
        if (status != HAL_OK || PageError != 0xFFFFFFFF) {

        	// อ่านค่า Error Flag ทั้งหมด
        	    uint32_t flash_error = HAL_FLASH_GetError();
        	    printf("❌ Erase FAIL on Page %lu! HAL_Status=%d, Flash_Error_Flags=0x%08lX, PageError=0x%08lX\r\n",
        	           page, status, flash_error, PageError);

        	    // ล้าง Flag ที่อาจค้างอยู่
        	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);


            result->failed_pages++;
            page_ok = 0;
            continue;
        }

        InvalidateICache();
        __DSB();
        __ISB();

        // Program 256 doublewords (2 KB)
        for (uint32_t dw = 0; dw < DW_PER_PAGE; dw++) {

            uint64_t test_value = FLASH_TEST_DATA_BASE + page + dw;
            uint32_t target_addr = page_addr + (dw * 8);

            result->total_words++;

            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                       target_addr,
                                       test_value);

            if (status != HAL_OK) {
                result->failed_words++;
                page_ok = 0;
                break;
            }


            InvalidateICache();
            __DMB();
            __DSB();
            __ISB();

            // Verify
            uint64_t verify_value = *(volatile uint64_t*)target_addr;

            if (verify_value == test_value) {
                result->success_words++;
            } else {
                result->failed_words++;
                page_ok = 0;
                break;
            }
        }

        if (page_ok) {
            result->success_pages++;
        } else {
            result->failed_pages++;
        }
    }

    HAL_FLASH_Lock();

    result->time_ms = HAL_GetTick() - start_time;
}


//__attribute__((cmse_nonsecure_entry))
//void Secure_Flash256KB(FlashResult_t *result)
//{
//    if (!result) return;
//
//    // Configuration
//    const uint32_t BANK2_START = 0x08040000;
//    const uint32_t BANK2_PAGES = 128;
//    const uint32_t DW_PER_PAGE = 256;
//    const uint64_t FLASH_TEST_DATA_BASE = 0x1234567890ABCDEFULL;
//
//    HAL_StatusTypeDef status;
//    uint32_t PageError = 0;
//
//    // Initialize result
//    result->total_pages = BANK2_PAGES;
//    result->success_pages = 0;
//    result->failed_pages = 0;
//    result->total_words = 0;
//    result->success_words = 0;
//    result->failed_words = 0;
//
//    uint32_t start_time = HAL_GetTick();
//
//    // Unlock Flash
//    status = HAL_FLASH_Unlock();
//    if (status != HAL_OK) {
//        result->time_ms = HAL_GetTick() - start_time;
//        return;
//    }
//
//    // Loop every page
//    for (uint32_t page = 0; page < BANK2_PAGES; page++) {
//
//        uint32_t page_addr = BANK2_START + (page * 0x800);
//        uint8_t page_ok = 1;  // ✅ ใช้ 1 = true, 0 = false
//
//        // Erase page
//        FLASH_EraseInitTypeDef EraseInit = {0};
//        EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
//        EraseInit.Banks = FLASH_BANK_2;
//        EraseInit.Page = page;
//        EraseInit.NbPages = 1;
//
//        status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
//        if (status != HAL_OK || PageError != 0xFFFFFFFF) {
//            result->failed_pages++;
//            page_ok = 0;  // ✅ ใช้ 0 แทน false
//            continue;
//        }
//
//        __DSB();
//        __ISB();
//
//        // Program 256 doublewords
//        for (uint32_t i = 0; i < DW_PER_PAGE; i++) {
//
//            uint64_t test_value = FLASH_TEST_DATA_BASE + page + i;
//            uint32_t target_addr = page_addr + (i * 8);
//
//            result->total_words++;
//
//            // Program
//            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
//                                       target_addr,
//                                       test_value);
//
//            if (status != HAL_OK) {
//                result->failed_words++;
//                page_ok = 0;
//                break;
//            }
//
//            __DSB();
//            __ISB();
//
//
//            // Verify
//            uint64_t verify_value = *(volatile uint64_t*)target_addr;
//
//            if (verify_value == test_value) {
//                result->success_words++;  // Count success
//            } else {
//                result->failed_words++;
//                page_ok = 0;
//                break;
//            }
//        }
//
//        if (page_ok) {
//            result->success_pages++;
//        } else {
//            result->failed_pages++;
//        }
//    }
//
//    // Lock Flash
//    HAL_FLASH_Lock();
//
//    result->time_ms = HAL_GetTick() - start_time;
//}



/**
  * @}
  */

/**
  * @}
  */
/* USER CODE END Non_Secure_CallLib */

