/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    FreeRTOS/FreeRTOS_SecureIOToggle_TrustZone/NonSecure/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body (non-secure)
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "stm32l5xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "hmac-sha256.h"
#include <math.h>
#include "aes.h"   // tiny-AES header ของคุณ
#include "speck.h"
#include "ff1_speck.h"


SemaphoreHandle_t uart_mutex;
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_127   ((uint32_t)0x0803F800) /* Base @ of Page 127, 2 Kbytes */
#define FLASH_PAGE_SIZE          0x00000800U


#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_4   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE - 1)   /* End @ of user Flash area */


// flasherase 0x08082000 อันเก่า
#define FLASH_TEST_START_ADDR   0x080BE000
#define FLASH_TEST_DATA         0x1234567812345678ULL
#define FLASH_PAGE_SIZE     	0x800  // 4 KB
#define SAFE_FLASH_BANK     	FLASH_BANK_1


//#define SAFE_FLASH_PAGE     100
//#define TARGET_FLASH_ADDR   0x08072000

#define SAFE_FLASH_PAGE   120
#define TARGET_FLASH_ADDR 0x08068000

#define TEST_VALUE        0x1234567890ABCDEFULL

TIM_HandleTypeDef htim2;

static FF1_Key_Speck ff1_speck_key;

typedef struct {
    SimSpk_Cipher cipher;
} SpeckShuffleKey;

static inline void SpeckShuffle_Init(SpeckShuffleKey *sk, const uint8_t key[16])
{
    Speck_Init(&sk->cipher, cfg_128_128, MODE_ECB, (void *)key, NULL, NULL);
}

// speck_shuffle.c
//#include "speck_shuffle.h"

// เข้ารหัส 128-bit (i, pass, seed) ด้วย Speck128 แล้ว map เป็น index
uint32_t SpeckShuffle_Index(const SpeckShuffleKey *sk,
                            uint32_t i,
                            uint32_t pass,
                            uint32_t seed,
                            uint32_t max_blocks)
{
    uint64_t pt[2];
    uint64_t ct[2];

    // pack ค่าเข้า 128 บิต (เลือก layout ตามใจ)
    pt[0] = ((uint64_t)i    << 32) | (uint64_t)pass;
    pt[1] = ((uint64_t)seed << 32) | (uint64_t)i;

    Speck_Encrypt_128(
        sk->cipher.round_limit,
        sk->cipher.key_schedule,
        (const uint8_t *)pt,
        (uint8_t *)ct
    );

    uint32_t raw = (uint32_t)(ct[0] ^ ct[1]);
    return raw % max_blocks;
}

static SpeckShuffleKey g_speck_shuffle;

void InitSpeckShuffle(void)
{
    static const uint8_t speck_raw_key[16] = {
        0x00,0x01,0x02,0x03,
        0x04,0x05,0x06,0x07,
        0x08,0x09,0x0A,0x0B,
        0x0C,0x0D,0x0E,0x0F
    };
    SpeckShuffle_Init(&g_speck_shuffle, speck_raw_key);
}

void Init_FF1_Speck(void)
{
    const uint8_t rawkey[16] = {
        0x00,0x01,0x02,0x03,
        0x04,0x05,0x06,0x07,
        0x08,0x09,0x0A,0x0B,
        0x0C,0x0D,0x0E,0x0F
    };
    FF1_SetKey_Speck(&ff1_speck_key, rawkey);
}



typedef struct {
    struct AES_ctx ctx;
} FF1_Key;

// เรียกครั้งเดียวตอน init
void FF1_SetKey(FF1_Key *k, const uint8_t raw_key[16]) {
    AES_init_ctx(&k->ctx, raw_key);
}

// in[16] -> out[16]
static void AES128_Encrypt_Block(const uint8_t in[16],
                                 uint8_t out[16],
                                 const FF1_Key *k)
{
    uint8_t buf[16];
    memcpy(buf, in, 16);
    AES_ECB_encrypt(&k->ctx, buf);
    memcpy(out, buf, 16);
}

//#define SAFE_FLASH_PAGE   150
//#define TARGET_FLASH_ADDR 0x0806B800

// หรือ page 200
//#define SAFE_FLASH_PAGE   200
//#define TARGET_FLASH_ADDR 0x08074000

// Test result structure
typedef struct {
  uint32_t test_count;
  uint32_t erase_status;   // 0=pending, 1=ok, 2=error
  uint32_t program_status; // 0=pending, 1=ok, 2=error
  uint32_t verify_status;  // 0=pending, 1=ok, 2=error
  uint64_t written_value;
  uint64_t read_value;
  uint32_t error_code;
} FlashTestResult_t;


//typedef struct {
//    uint32_t page;
//    uint32_t bank;
//    uint32_t target_addr;
//    uint64_t value;
//} SecureFlashParams_t;


volatile FlashTestResult_t flash_test = {0};

/* USER CODE BEGIN PFP */
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);


static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
HAL_StatusTypeDef NonSecure_EraseWriteVerify(void);


// flasherase
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* Definitions for LEDThreadHandle */
osThreadId_t LEDThreadHandleHandle;
const osThreadAttr_t LEDThreadHandle_attributes = {
  .name = "LEDThreadHandle",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);

void LED_Thread(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
void SecureFault_Callback(void);
void SecureError_Callback(void);
void LinearHMAC(void *argument);
void NormalTask(void *argument);
void SMARM_Experiment_Task(void *argument);
void Test_Simulation(void *argument);
void NS_SMARM_Benchmark_Experiment(void *argument); // เอาไว้ดูว่า SMARM vs Feistel เกิด overhead ต่างกันเท่าไรเ
void NS_SMARM_measurement(void *argument); // เช็๕ t_disabled
void Test_SpeckShuffle_Basic(void *argument);

UART_HandleTypeDef huart1;  // or whichever UART you're using

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  uart_mutex = xSemaphoreCreateMutex();
  if (uart_mutex == NULL) {
      Error_Handler();  // Or handle the error gracefully
  }

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDThreadHandle */
//  LEDThreadHandleHandle = osThreadNew(NormalTask, NULL, &LEDThreadHandle_attributes);
  LEDThreadHandleHandle = osThreadNew(NS_SMARM_Benchmark_Experiment, NULL, &LEDThreadHandle_attributes);


  /* creation of myTask02 */
//  myTask02Handle = osThreadNew(LED_Thread, NULL, &myTask02_attributes);

//  myTask02Handle = osThreadNew(SMARM_Experiment_Task, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799; //799
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ใส่เข้ามาเพระาติดปัญหา Linker
#include <sys/time.h>

// เพิ่มฟังก์ชันนี้ลงไปเพื่อแก้ Linker Error
int _gettimeofday(struct timeval *tv, void *tzvp) {
    (void)tv;
    (void)tzvp;
    return 0;  // คืนค่า 0 เพื่อบอกว่าทำงานสำเร็จ (แม้จะไม่ได้คืนเวลาจริงก็ตาม)
}
	/**
	 * @brief  Callback called by secure code following a secure fault interrupt
	 * @note   This callback is called by secure code thanks to the registration
	 *         done by the non-secure application with non-secure callable API
	 *         SECURE_RegisterCallback(SECURE_FAULT_CB_ID, (void *)SecureFault_Callback);
	 * @retval None
	 */
	void SecureFault_Callback(void) {
		/* Go to error infinite loop when Secure fault generated by IDAU/SAU check */
		/* because of illegal access */
		Error_Handler();
	}


	int __io_putchar(int ch)
	{
	    HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	    return ch;
	}

	/**
	 * @brief  Callback called by secure code following a GTZC TZIC secure interrupt (GTZC_IRQn)
	 * @note   This callback is called by secure code thanks to the registration
	 *         done by the non-secure application with non-secure callable API
	 *         SECURE_RegisterCallback(GTZC_ERROR_CB_ID, (void *)SecureError_Callback);
	 * @retval None
	 */
	void SecureError_Callback(void) {
		/* Go to error infinite loop when Secure error generated by GTZC check */
		/* because of illegal access */
		Error_Handler();
	}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_LED_Thread */
	/**
	 * @brief  Function implementing the LEDThreadHandle thread.
	 * @param  argument: Not used
	 * @retval None
	 */
/* USER CODE END Header_LED_Thread */


	static inline void __busy_wait_cycles(uint32_t cycles)
	{
	    uint32_t iterations = (cycles + 2) / 3;  // Adjust for 3-cycle loop execution

	    __asm volatile (
	        ".balign 4;"        // Align loop for performance
	        "1:"
	        "    subs %0, %0, #1;"  // Decrement counter
	        "    bne 1b;"           // Branch to label 1 if not zero
	        : "+r" (iterations)
	        :
	        : "memory", "cc"
	    );
	}



//	static uint32_t GetPage(uint32_t Addr)
//	{
//	  uint32_t page = 0;
//
//	  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
//	  {
//	    /* Bank 1 */
//	    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
//	  }
//	  else
//	  {
//	    /* Bank 2 */
//	    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
//	  }
//
//	  return page;
//	}

	static uint32_t GetBank(uint32_t Addr)
	{
	  return FLASH_BANK_1;
	}


void LED_Thread(void *argument)
{
  /* USER CODE BEGIN 5 */


		(void) argument;

		portALLOCATE_SECURE_CONTEXT(configMINIMAL_SECURE_STACK_SIZE);
		uint32_t start_tick = 0;
		uint32_t ctround = 0;

		if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
		{

			printf("NormalS: %lu\r\n", 0);
			xSemaphoreGive(uart_mutex);

		}

		if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
		{

			printf("NormalE: %lu\r\n", 0);

			xSemaphoreGive(uart_mutex);
		}

		for (;;) {

		ctround++;

		start_tick = HAL_GetTick();
		if(ctround % 10000 == 0  ){
			if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
			{

				printf("NormalS: %lu\r\n", start_tick);
				xSemaphoreGive(uart_mutex);

			}
		}

	    uint32_t delay_cycles_1 = (SystemCoreClock / 850) * 1;
	    __busy_wait_cycles(delay_cycles_1);

	    start_tick = HAL_GetTick();
//	    printf("NormalE: %lu\r\n", start_tick);
		if(ctround % 10000 == 0  ){
			if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
			{

					printf("NormalE: %lu\r\n", start_tick);

				xSemaphoreGive(uart_mutex);
			}
		}

	    osDelay(1);

		}

}



uint32_t GetPage(uint32_t Addr)
{
    uint32_t page = 0;

    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
        // Bank 1
        page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    }
    else
    {
        // Bank 2
        page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
    }

    return page;
}


// ใน normal_code
HAL_StatusTypeDef EraseFlashPages_256KB(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    HAL_StatusTypeDef status;

    // ========== Erase Pages 1-127 (254KB) ==========
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_2;
    EraseInitStruct.Page = 64;        // Start from Page 1
    EraseInitStruct.NbPages = 64;   // Erase 127 pages (254KB)

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        // PAGEError contains faulty page
        return status;
    }

    return HAL_OK;
}


// ประกาศตัวแปรและค่าคงที่ (อ้างอิงจากขนาดที่คุณใช้ใน Secure World)
//#define NS_CHUNK_SIZE  0x40000
//#define NS_BLOCK_SIZE  1024
//#define NS_TOTAL_SIZE  0x3F000 // 512KB
//#define NS_BLOCKS      (NS_TOTAL_SIZE / NS_BLOCK_SIZE)


uint8_t *ns_test_memory = (uint8_t *)0x08040000;


#define NS_CHUNK_SIZE     0x40000  // 256KB (พื้นที่ปลอดภัยใน Bank 2)
#define NS_BLOCK_SIZE     4096
#define NS_BLOCKS         (NS_CHUNK_SIZE / NS_BLOCK_SIZE) // 256 Blocks
#define TARGET_TOTAL_KB   512
#define PASSES_REQUIRED   (TARGET_TOTAL_KB * 1024 / NS_CHUNK_SIZE)

// จอง RAM สำหรับวิธีเดิม (Stored Array) - วิธีนี้กิน RAM ตามจำนวน Block
static int stored_indices[NS_BLOCKS];


// ตรวจสอบที่อยู่ Memory ที่จะสแกน (ต้องเป็นที่อยู่ที่ NS เข้าถึงได้)



HAL_StatusTypeDef EraseFlashPages(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    HAL_StatusTypeDef status;

    // Calculate page numbers
    uint32_t StartPage = GetPage(0x0807F800);  // Returns 127
    uint32_t EndPage = GetPage(0x0807FBF0);    // Returns 127
    uint32_t NbPages = EndPage - StartPage + 1; // 1 page

    // Unlock Flash
    HAL_FLASH_Unlock();

    // Fill EraseInit structure
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_2;      // Bank 2
    EraseInitStruct.Page = StartPage;          // Page 127
    EraseInitStruct.NbPages = NbPages;         // 1 page

    // Perform erase
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

    if (status != HAL_OK)
    {
        // Erase failed
        // PAGEError contains the faulty page number
        HAL_FLASH_Lock();
        return status;
    }

    // Lock Flash
    HAL_FLASH_Lock();

    return HAL_OK;
}

static void FF1_PRF(const FF1_Key *key,
                    const uint8_t *M, size_t Mlen,
                    uint8_t Y[16])
{
    uint8_t X[16] = {0};           // Y0 = 0^128
    size_t blocks = Mlen / 16;     // Mlen ต้องหาร 16 ลงตัว

    for (size_t j = 0; j < blocks; j++) {
        uint8_t tmp[16];
        const uint8_t *Mj = M + 16*j;
        for (int i = 0; i < 16; i++) {
            tmp[i] = X[i] ^ Mj[i];
        }
        AES128_Encrypt_Block(tmp, X, key);
    }
    memcpy(Y, X, 16);
}

// =========================
// 3. NUM/STR base-2
// =========================

static uint32_t NUM2(const uint8_t *X, size_t m) {
    uint32_t x = 0;
    for (size_t i = 0; i < m; i++) {
        x = (x << 1) | (X[i] & 1);
    }
    return x;
}

static void STR2(uint32_t x, uint8_t *X, size_t m) {
    for (size_t i = 0; i < m; i++) {
        X[m - 1 - i] = (uint8_t)(x & 1);
        x >>= 1;
    }
}

// =========================
// 4. ceil(log2(x))
// =========================

static uint32_t ceil_log2(uint32_t x) {
    uint32_t n = 0;
    uint32_t v = x - 1;
    while (v > 0) {
        v >>= 1;
        n++;
    }
    return n;
}

// =========================
// 5. FF1 core (radix = 2)
// =========================

static void FF1_Encrypt_Binary(const FF1_Key *K,
                               uint8_t *X, size_t n,
                               const uint8_t *T, size_t t)
{
    size_t u = n / 2;
    size_t v = n - u;

    uint8_t *A = X;       // length u
    uint8_t *B = X + u;   // length v

    // สร้าง P แบบง่ายสำหรับ radix=2
    uint8_t P[16] = {0};
    P[0] = 0x01;          // บอกว่า FF1
    P[1] = 0x00; P[2] = 0x00; P[3] = 0x02;  // radix = 2
    P[4] = 0x0A;          // Mode FF1
    P[5] = (uint8_t)((n >> 24) & 0xFF);
    P[6] = (uint8_t)((n >> 16) & 0xFF);
    P[7] = (uint8_t)((n >>  8) & 0xFF);
    P[8] = (uint8_t)(n & 0xFF);
    P[9]  = 0x00;
    P[10] = 0x00;
    P[11] = (uint8_t)t;
    // P[12..15] = 0

    // d = 4 * ceil(v / 16) (log2(radix)=1)
    size_t d = 4 * ((v + 15) / 16);
    if (d < 4) d = 4;

    // beta = ceil(t/16), b = 16*beta (อย่างน้อย 1 block)
    size_t beta = (t + 15) / 16;
    size_t b    = 16 * beta;
    if (b == 0) b = 16;

    uint8_t Q[64];
    uint8_t PQ[16 + 64];

    for (int i = 0; i < 10; i++) {
        size_t m    = (i % 2 == 0) ? u : v;
        size_t mlen = m;

        uint32_t numB = (i % 2 == 0) ? NUM2(B, v) : NUM2(A, u);

        memset(Q, 0, sizeof(Q));
        if (t > 0) memcpy(Q, T, t);
        Q[b - 1] = (uint8_t)i;

        uint32_t tmp = numB;
        for (size_t k = 0; k < d; k++) {
            Q[b + d - 1 - k] = (uint8_t)(tmp & 0xFF);
            tmp >>= 8;
        }

        size_t Qlen  = b + d;
        size_t PQlen = 16 + Qlen;
        memcpy(PQ, P, 16);
        memcpy(PQ + 16, Q, Qlen);

        size_t pad = (16 - (PQlen % 16)) % 16;
        memset(PQ + PQlen, 0, pad);
        PQlen += pad;

        uint8_t Y[16];
        FF1_PRF(K, PQ, PQlen, Y);

        uint32_t y = 0;
        for (size_t k = 0; k < d; k++) {
            y = (y << 8) | Y[k % 16];
        }

        uint32_t a = NUM2(A, mlen);
        uint32_t mod = (mlen == 32) ? 0xFFFFFFFFu : (1u << mlen);
        uint32_t c = (a + y) % mod;

        uint8_t C[32];
        STR2(c, C, mlen);

        if (i % 2 == 0) {
            if (v > 0) memcpy(A, B, v);
            memcpy(B, C, mlen);
        } else {
            memcpy(B, A, u);
            memcpy(A, C, mlen);
        }
    }
}

// =========================
// 6. public permute (แทน FeistelPermute)
// =========================

uint32_t FF1Permute(uint32_t index,
                    uint32_t max_blocks,
                    const FF1_Key *key,
                    uint32_t tweak)
{
    if (index >= max_blocks) return index;

    uint32_t n = ceil_log2(max_blocks);
    if (n == 0) return index;
    if (n > 32) n = 32;

    uint8_t X[32];
    for (uint32_t i = 0; i < n; i++) {
        X[n - 1 - i] = (uint8_t)((index >> i) & 1u);
    }

    uint8_t T[4];
    T[0] = (uint8_t)((tweak >> 24) & 0xFF);
    T[1] = (uint8_t)((tweak >> 16) & 0xFF);
    T[2] = (uint8_t)((tweak >>  8) & 0xFF);
    T[3] = (uint8_t)( tweak        & 0xFF);

    FF1_Encrypt_Binary(key, X, n, T, sizeof(T));

    uint32_t out = 0;
    for (uint32_t i = 0; i < n; i++) {
        out = (out << 1) | (X[i] & 1u);
    }

    if (out >= max_blocks) {
        return FF1Permute(out, max_blocks, key, tweak + 1);
    }
    return out;
}

static FF1_Key ff1_key;

void Init_FF1_Key(void) {
    const uint8_t raw_key[16] = {
        0x00,0x01,0x02,0x03,
        0x04,0x05,0x06,0x07,
        0x08,0x09,0x0A,0x0B,
        0x0C,0x0D,0x0E,0x0F
    };
    FF1_SetKey(&ff1_key, raw_key);
}

void StartTask09(void *argument){


		(void) argument;
	    portALLOCATE_SECURE_CONTEXT(4096);
	    osDelay(2000);

	    // Freeze TIM2 when debugger halts (for debugging convenience)
	    __HAL_DBGMCU_FREEZE_TIM2();

	    // Ensure TIM2 is started
	    HAL_TIM_Base_Start(&htim2);

	    HAL_StatusTypeDef erase_status;
	    uint32_t fixed_addr = 0x08048000;
	    uint32_t fixed_page = GetPage(fixed_addr);  // Convert 0x08048000 → page number

	    for(;;)
	    {
	    	for(int j=1;j<=5;j++){

	    		printf("------------------------\r\n");
	    		printf("Round %d\r\n", j);

				uint32_t start_systick = osKernelGetTickCount();
				uint32_t start_tim = __HAL_TIM_GET_COUNTER(&htim2);

				__disable_irq();
				for(int i=0;i<16;i++){


					// erase
					FLASH_EraseInitTypeDef EraseInitStruct;
					uint32_t PAGEError = 0;
					HAL_StatusTypeDef status;

					HAL_FLASH_Unlock();

					EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
					EraseInitStruct.Banks = FLASH_BANK_2;
					EraseInitStruct.Page = fixed_page;
					EraseInitStruct.NbPages = 4;

					status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
					HAL_FLASH_Lock();


					uint32_t write_success = 0;
					Simulate_flash_write_128KB(fixed_addr, (uint8_t)i, &write_success);



				}
				__enable_irq();

				uint32_t end_tim = __HAL_TIM_GET_COUNTER(&htim2);
				uint32_t end_systick = osKernelGetTickCount();

				  // ========== Calculate elapsed time ==========

				            // TIM2 cycles
				            uint32_t timer_cycles = end_tim - start_tim;

				            // Convert TIM2 cycles to microseconds: (cycles × 800) / 110
				            uint32_t timer_us = (timer_cycles * 800) / 110;

				            // Convert to milliseconds
				            uint32_t timer_ms = timer_us / 1000;

				            // SysTick ticks
				            uint32_t systick_ticks = end_systick - start_systick;

				            // Convert SysTick ticks to milliseconds: each tick = 100μs
				            uint32_t systick_ms = (systick_ticks * 100) / 1000;

				            // ========== Print results ==========
				            printf("TIMER cycles: %lu\r\n", timer_cycles);
				            printf("TIMER ms: %lu\r\n", timer_ms);
				            printf("Systick tick: %lu\r\n", systick_ticks);
				            printf("Systick ms: %lu\r\n", systick_ms);


				osDelay(1000);
	    	}
			osDelay(10000);

	    }

}
void StartTask08(void *argument)
{
    (void) argument;
    portALLOCATE_SECURE_CONTEXT(4096);
    osDelay(2000);

    // Freeze TIM2 when debugger halts (for debugging convenience)
    __HAL_DBGMCU_FREEZE_TIM2();

    // Ensure TIM2 is started
    HAL_TIM_Base_Start(&htim2);

    HAL_StatusTypeDef erase_status;
    uint32_t total_words = 16384;  // (128KB / 8)

    for(;;)
    {
        // Reset counters for this iteration
        uint32_t success_words = 0;
        uint32_t failed_words = 0;

        // ================================================================
        // STEP 1: ERASE MEASUREMENT (Both RTOS ticks and TIM2)
        // ================================================================

        // Capture erase start timestamps
        uint32_t erase_rtos_start = osKernelGetTickCount();
        uint32_t erase_tim2_start = __HAL_TIM_GET_COUNTER(&htim2);

        erase_status = EraseFlashPages_256KB();

        // Capture erase end timestamps immediately
        uint32_t erase_tim2_end = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t erase_rtos_end = osKernelGetTickCount();

        if (erase_status != HAL_OK) {
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                printf("❌ ERASE FAILED !\r\n");
                xSemaphoreGive(uart_mutex);
            }
            osDelay(10000);
            continue;
        }

        uint32_t erase_rtos_ticks = erase_rtos_end - erase_rtos_start;
        uint32_t erase_tim2_cycles = erase_tim2_end - erase_tim2_start;


        // ================================================================
        // STEP 2: WRITE MEASUREMENT (Both RTOS ticks and TIM2)
        // ================================================================

        // Capture write start timestamps
        uint32_t write_rtos_start = osKernelGetTickCount();
        uint32_t write_tim2_start = __HAL_TIM_GET_COUNTER(&htim2);

         // === Execute secure flash write operation ===
        __disable_irq();
        Secure_WriteFlash_128KB(&success_words, &failed_words);
        __enable_irq();

        // Capture write end timestamps immediately
        uint32_t write_tim2_end = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t write_rtos_end = osKernelGetTickCount();


        uint32_t write_rtos_ticks = write_rtos_end - write_rtos_start;
        uint32_t write_tim2_cycles = write_tim2_end - write_tim2_start;

        // ================================================================
              // STEP 4: CONVERT TO REAL TIME UNITS (INTEGER MATH ONLY)
              // ================================================================

              // === ERASE TIME CONVERSION ===
              // SysTick: Tick rate is 10000 Hz = 100μs per tick
              uint32_t erase_rtos_us = erase_rtos_ticks * 100;
              uint32_t erase_rtos_ms = erase_rtos_us / 1000;
              uint32_t erase_rtos_ms_frac = erase_rtos_us % 1000;

              // TIM2: (cycles × 800) / 110 = microseconds
              uint32_t erase_tim2_us = (erase_tim2_cycles * 800) / 110;
              uint32_t erase_tim2_ms = erase_tim2_us / 1000;
              uint32_t erase_tim2_ms_frac = erase_tim2_us % 1000;

              // Erase difference
              int32_t erase_diff_us = (int32_t)erase_tim2_us - (int32_t)erase_rtos_us;


              // === WRITE TIME CONVERSION ===
                    // SysTick: 10000 Hz = 100μs per tick
                    uint32_t write_rtos_us = write_rtos_ticks * 100;
                    uint32_t write_rtos_ms = write_rtos_us / 1000;
                    uint32_t write_rtos_ms_frac = write_rtos_us % 1000;

                    // TIM2: (cycles × 800) / 110 = microseconds
                    uint32_t write_tim2_us = (write_tim2_cycles * 800) / 110;
                    uint32_t write_tim2_ms = write_tim2_us / 1000;
                    uint32_t write_tim2_ms_frac = write_tim2_us % 1000;

                    int32_t write_diff_us = (int32_t)write_tim2_us - (int32_t)write_rtos_us;


                    // === TOTAL TIME ===
                           uint32_t total_rtos_us = erase_rtos_us + write_rtos_us;
                           uint32_t total_rtos_ms = total_rtos_us / 1000;
                           uint32_t total_rtos_ms_frac = total_rtos_us % 1000;

                           uint32_t total_tim2_us = erase_tim2_us + write_tim2_us;
                           uint32_t total_tim2_ms = total_tim2_us / 1000;
                           uint32_t total_tim2_ms_frac = total_tim2_us % 1000;

                           int32_t total_diff_us = (int32_t)total_tim2_us - (int32_t)total_rtos_us;

                           // ================================================================
                                  // STEP 5: PRINT RESULTS WITH COMPARISON
                                  // ================================================================
                           if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                                   {
                                       static int round = 1;

                                       printf("\r\n========================================\r\n");
                                       printf("Hot Patch Timing - Round %d\r\n", round);
                                       printf("========================================\r\n");

                                       // ============ ERASE PHASE ============
                                       printf("\r\n--- ERASE PHASE ---\r\n");
                                       printf("RAW DATA:\r\n");
                                       printf("  SysTick: %lu ticks\r\n", erase_rtos_ticks);
                                       printf("  TIM2:    %lu cycles\r\n", erase_tim2_cycles);
                                       printf("========================================\r\n");
                                       printf("CONVERTED TO MICROSECONDS (us):\r\n");
                                       printf("  SysTick: %lu us\r\n", erase_rtos_us);
                                       printf("  TIM2:    %lu us\r\n", erase_tim2_us);
                                       printf("  Difference: %+ld us\r\n", erase_diff_us);
                                       printf("========================================\r\n");
                                       printf("CONVERTED TO MILLISECONDS (ms):\r\n");
                                       printf("  SysTick: %lu.%03lu ms\r\n", erase_rtos_ms, erase_rtos_ms_frac);
                                       printf("  TIM2:    %lu.%03lu ms\r\n", erase_tim2_ms, erase_tim2_ms_frac);


                                       // ============ WRITE PHASE ============
                                       printf("\r\n--- WRITE PHASE ---\r\n");
                                       printf("RAW DATA:\r\n");
                                       printf("  SysTick: %lu ticks\r\n", write_rtos_ticks);
                                       printf("  TIM2:    %lu cycles\r\n", write_tim2_cycles);
                                       printf("========================================\r\n");
                                       printf("CONVERTED TO MICROSECONDS (us):\r\n");
                                       printf("  SysTick: %lu us\r\n", write_rtos_us);
                                       printf("  TIM2:    %lu us\r\n", write_tim2_us);
                                       printf("  Difference: %+ld us\r\n", write_diff_us);
                                       printf("========================================\r\n");
                                       printf("CONVERTED TO MILLISECONDS (ms):\r\n");
                                       printf("  SysTick: %lu.%03lu ms\r\n", write_rtos_ms, write_rtos_ms_frac);
                                       printf("  TIM2:    %lu.%03lu ms\r\n", write_tim2_ms, write_tim2_ms_frac);


                                       // ============ TOTAL TIME ============
                                       printf("\r\n--- TOTAL TIME ---\r\n");
                                       printf("CONVERTED TO MICROSECONDS (us):\r\n");
                                       printf("  SysTick: %lu us\r\n", total_rtos_us);
                                       printf("  TIM2:    %lu us\r\n", total_tim2_us);
                                       printf("  Difference: %+ld us\r\n", total_diff_us);
                                       printf("========================================\r\n");
                                       printf("CONVERTED TO MILLISECONDS (ms):\r\n");
                                       printf("  SysTick: %lu.%03lu ms\r\n", total_rtos_ms, total_rtos_ms_frac);
                                       printf("  TIM2:    %lu.%03lu ms\r\n", total_tim2_ms, total_tim2_ms_frac);



                                       printf("========================================\r\n\r\n");

                                       round++;
                                       xSemaphoreGive(uart_mutex);
                                   }

        osDelay(10000);
    }
}

void StartTask07(void *argument)
{
    (void) argument;
    portALLOCATE_SECURE_CONTEXT(configMINIMAL_SECURE_STACK_SIZE);

    osDelay(2000);

    for(;;)
    {
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("\r\n[NS] === Calling Secure_FlashTest() ===\r\n");
            xSemaphoreGive(uart_mutex);
        }

        // Call Secure function
//        Secure_FlashTest();
        Secure_EraseWriteVerify();
        // Verify from Non-Secure side
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            __DSB();
            __ISB();

            uint64_t ns_read = *(volatile uint64_t*)0x080BE000;

            printf("\r\n[NS] Verify from Non-Secure:\r\n");
            printf("[NS] Address: 0x080BE000\r\n");
            printf("[NS] Read:    0x%016llX\r\n", ns_read);

            if (ns_read == 0x1234567812345678ULL)
            {
                printf("[NS] ✅ Non-Secure verify SUCCESS!\r\n");
            }
            else
            {
                printf("[NS] ❌ Non-Secure verify FAILED!\r\n");
            }

            xSemaphoreGive(uart_mutex);
        }

        osDelay(5000);
    }
}


void StartTask06(void *argument)
{

	(void) argument;
	uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;

	static HAL_StatusTypeDef status;
	 if (HAL_ICACHE_Disable() != HAL_OK)
	  {
		 printf("Error");
	    Error_Handler();
	  }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;

//    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//    EraseInitStruct.Page = 100;     // Page 255
//    EraseInitStruct.NbPages = 1;
//    EraseInitStruct.Banks = SAFE_FLASH_BANK;    // Bank 2

    FirstPage = GetPage(FLASH_USER_START_ADDR);

    /* Get the number of pages to erase from 1st page */
    NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

    /* Get the bank */
    BankNumber = GetBank(FLASH_USER_START_ADDR);

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = 2;
    EraseInitStruct.Page        = 255;
    EraseInitStruct.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("Error PageError=0x%08lX\r\n", PageError);
            xSemaphoreGive(uart_mutex);
        }
    }
    else
    {
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("Erase OK! Page %d Bank 2\r\n", SAFE_FLASH_PAGE);
            xSemaphoreGive(uart_mutex);
        }

        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
        	uint32_t *check_ptr = (uint32_t *)(0x0807F800); // เช่น Page 255 ใน Bank 2 (0x0807F800 - 0x0807FFFF)
            printf("After Erase: 0x%08lX\r\n", *check_ptr);
            xSemaphoreGive(uart_mutex);
        }


             uint64_t test_value = 0x12345678;           // ข้อมูลทดสอบ (8 byte, DOUBLEWORD)
             uint32_t target_addr = 0x0807F800;                     // Page 255 Bank 2

             HAL_StatusTypeDef prog_status = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_DOUBLEWORD,
                    target_addr,
                    test_value
             );


            uint64_t verify_value = *(volatile uint64_t*)target_addr;


               if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                   if (verify_value == test_value) {
                       printf("✅ Write+Verify OK!\r\n");
                       printf("   Expected: 0x%016llX\r\n", test_value);
                       printf("   Read:     0x%016llX\r\n", verify_value);
                   } else {
                       printf("❌ Verify FAIL!\r\n");
                       printf("   Expected: 0x%016llX\r\n", test_value);
                       printf("   Read:     0x%016llX\r\n", verify_value);
                   }
                   xSemaphoreGive(uart_mutex);
               }
    }

    HAL_FLASH_Lock();

    osDelay(1000);
}


void StartTask05(void *argument)
{
  (void) argument;

  static HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef erase_cfg;
  static uint32_t page_error = 0;
  uint64_t test_data;

  osDelay(2000);

  if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
   {
     printf("page_error addr: 0x%08lX\r\n", (uint32_t)&page_error);
     xSemaphoreGive(uart_mutex);
   }

  if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    printf("\r\n=== Flash Test Started ===\r\n");
    printf("Address: 0x%08lX (Bank2, Page2)\r\n", FLASH_TEST_START_ADDR);
    xSemaphoreGive(uart_mutex);
  }

  for(;;)
  {
    flash_test.test_count++;
    test_data = FLASH_TEST_DATA + flash_test.test_count;
    flash_test.written_value = test_data;

    // Reset status
    flash_test.erase_status = 0;
    flash_test.program_status = 0;
    flash_test.verify_status = 0;
    flash_test.error_code = 0;

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      printf("\r\n--- Test #%lu ---\r\n", flash_test.test_count);
      xSemaphoreGive(uart_mutex);
    }

    // Unlock
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
      flash_test.error_code = 1;
      if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        printf("❌ Unlock FAILED\r\n");
        xSemaphoreGive(uart_mutex);
      }
      goto test_end;
    }

    // Erase (Bank 2, Page 2 = Non-Secure)
    erase_cfg.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_cfg.Banks = FLASH_BANK_1;
    erase_cfg.Page = 2;  // ✅ Page 2 (Non-Secure!)
    erase_cfg.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_cfg, &page_error);

    if (status != HAL_OK || page_error != 0xFFFFFFFF)
    {
      flash_test.erase_status = 2;
      flash_test.error_code = 2;

      if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        printf("❌ Erase FAILED (status=%d, err=0x%08lX)\r\n", status, page_error);
        xSemaphoreGive(uart_mutex);
      }

      HAL_FLASH_Lock();
      goto test_end;
    }

    flash_test.erase_status = 1;

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      printf("✅ Erase OK\r\n");
      xSemaphoreGive(uart_mutex);
    }

    // Program
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                FLASH_TEST_START_ADDR,
                                test_data);

    if (status != HAL_OK)
    {
      flash_test.program_status = 2;
      flash_test.error_code = 3;

      if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        printf("❌ Program FAILED (status=%d)\r\n", status);
        xSemaphoreGive(uart_mutex);
      }

      HAL_FLASH_Lock();
      goto test_end;
    }

    flash_test.program_status = 1;

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      printf("✅ Program OK\r\n");
      xSemaphoreGive(uart_mutex);
    }

    // Lock
    HAL_FLASH_Lock();

    // Verify
    __DSB();
    __ISB();

    flash_test.read_value = *(volatile uint64_t*)FLASH_TEST_START_ADDR;

    if (flash_test.read_value == test_data)
    {
      flash_test.verify_status = 1;

      if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        printf("✅ Verify OK (0x%016llX)\r\n", flash_test.read_value);
        printf("🎉 Test #%lu SUCCESS!\r\n", flash_test.test_count);
        xSemaphoreGive(uart_mutex);
      }
    }
    else
    {
      flash_test.verify_status = 2;
      flash_test.error_code = 4;

      if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        printf("❌ Verify FAILED\r\n");
        printf("   Expected: 0x%016llX\r\n", test_data);
        printf("   Got:      0x%016llX\r\n", flash_test.read_value);
        xSemaphoreGive(uart_mutex);
      }
    }

test_end:
    osDelay(3000);
  }
}

// 1. ฟังก์ชันช่วยหาค่าเลขยกกำลัง 2 ที่ครอบคลุมจำนวนบล็อก (Small Domain)
static uint32_t get_next_pow2(uint32_t n) {
    uint32_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

// 2. Round Function (F-function): หัวใจของการสุ่ม
// ใช้การคำนวณบิตพื้นฐานเพื่อให้ทำงานได้เร็วบน MCU
static uint32_t f_function(uint32_t val, uint32_t round, uint32_t seed) {
    uint32_t hash = val ^ round ^ seed;
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = (hash >> 16) ^ hash;
    return hash;
}

// 3. ฟังก์ชันหลัก: Feistel Permutation พร้อมระบบ Cycle-walking
uint32_t FeistelPermute(uint32_t index, uint32_t max_blocks, uint32_t seed) {
    uint32_t domain = get_next_pow2(max_blocks);
    uint32_t half_bits = 0;
    uint32_t tmp_domain = domain;
    while (tmp_domain > 1) { tmp_domain >>= 1; half_bits++; }

    // แบ่งบิตออกเป็นฝั่งซ้าย (L) และขวา (R)
    uint32_t r_half_bits = half_bits / 2;
    uint32_t l_half_bits = half_bits - r_half_bits;
    uint32_t r_mask = (1 << r_half_bits) - 1;
    uint32_t l_mask = (1 << l_half_bits) - 1;

    uint32_t curr_val = index;

    // --- Cycle-walking Loop ---
    // ถ้าสุ่มได้เลขที่เกินจำนวนบล็อก (max_blocks) ให้ทำการเดินต่อ (Walk) จนกว่าจะตกในช่วง
    do {
        uint32_t L = curr_val >> r_half_bits;
        uint32_t R = curr_val & r_mask;

        // รัน 3-4 Rounds ก็เพียงพอสำหรับการสุ่มตำแหน่งบล็อกแล้ว
        for (int i = 0; i < 4; i++) {
            uint32_t next_L = R;
            uint32_t next_R = L ^ (f_function(R, i, seed) & l_mask);
            L = next_L;
            R = next_R;
            // สลับฝั่งเพื่อให้บิตกระจายตัว
            uint32_t temp = L; L = R; R = temp;
        }
        curr_val = (L << r_half_bits) | (R & r_mask);
    } while (curr_val >= max_blocks); //

    return curr_val;
}

void Test_Simulation(void *argument) {

	uint32_t total_blocks = 1000;
	uint32_t test_seed = 999;
    printf("--- Testing Feistel Permutation (Blocks: %d, Seed: %d) ---\n", total_blocks, test_seed);

    // ใช้ array เล็กๆ เช็กเฉพาะตอนเทสในคอม/Normal World
    uint8_t *check = (uint8_t *)calloc(total_blocks, 1);
    uint32_t collision_count = 0;

    for (uint32_t i = 0; i < total_blocks; i++) {
        uint32_t p = FeistelPermute(i, total_blocks, test_seed);
//        printf("i: %d,n: %d \r\n",i,p);

        if (p >= total_blocks) printf("Error: Out of range! (%d)\n", p);
        if (check[p] == 1) collision_count++;

        check[p] = 1;
        // printf("i: %d -> Shuffled Index: %d\n", i, p);
        osDelay(10);
    }

    if (collision_count == 0) {
        printf("SUCCESS\n");
    } else {
        printf("FAILED: Found %d collisions!\n", collision_count);
    }
    free(check);
}





#define TARGET_FREQ_HZ   1000
#define TIM2_TICKS_PER_SEC  137500
volatile uint32_t g_normal_counter = 0; // ตัวนับรอบของ NormalTask

void NormalTask(void *argument)
{
	(void) argument;

	    // คำนวณ Period (1000 / 1000 = 1 tick)
	    const uint32_t period_os_ticks = 1000 / TARGET_FREQ_HZ;

	    uint32_t next_wake_time = osKernelGetTickCount();
	    uint32_t loop_counter = 0;

	    // เริ่มจับเวลา TIM2
	    uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);

//	    printf("\r\n=== FREQUENCY TEST MODE (Integer Math) ===\r\n");
//	    printf("Target: %d Hz | TickRate: %d\r\n", TARGET_FREQ_HZ, configTICK_RATE_HZ);

	    for (;;)
	    {
	        // 1. คุมจังหวะการทำงาน
	        next_wake_time += period_os_ticks;
	        osDelayUntil(next_wake_time);

	        g_normal_counter++; // <-- เพิ่มบรรทัดนี้เพื่อให้นับรอบจริง

	        // 2. นับรอบ
	        loop_counter++;

	        // 3. เช็คเวลา TIM2
	        uint32_t current_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
	        uint32_t elapsed_tim2 = current_tim2 - start_tim2; // Auto-handle overflow

	        // ถ้าเวลาผ่านไปเกิน 1 วินาที (โดยใช้ TIM2 เป็นเกณฑ์)
	        if (elapsed_tim2 >= TIM2_TICKS_PER_SEC)
	        {
	            // --- คำนวณแบบไม่ใช้ Float (%f) เพื่อแก้ปัญหาค่าไม่ออก ---

	            // 1. คำนวณเวลาจริงหน่วยวินาที (x1000 เพื่อทำเป็นทศนิยม 3 ตำแหน่ง)
	            // สูตร: (Elapsed_Ticks * 1000) / Ticks_Per_Sec
	            uint32_t time_ms = (uint64_t)elapsed_tim2 * 1000 / TIM2_TICKS_PER_SEC;

	            // แยกวินาที กับ มิลลิวินาที
	            uint32_t time_s_int = time_ms / 1000;
	            uint32_t time_s_dec = time_ms % 1000;

	            // 2. คำนวณความถี่จริง (Hz) (x100 เพื่อทำเป็นทศนิยม 2 ตำแหน่ง)
	            // สูตร: (Count * Hardware_Ticks_Per_Sec * 100) / Elapsed_Hardware_Ticks
	            // การใช้ Hardware Ticks หารโดยตรงจะแม่นยำกว่าแปลงเป็น ms ก่อน
	            uint32_t freq_x100 = ((uint64_t)loop_counter * TIM2_TICKS_PER_SEC * 100) / elapsed_tim2;

	            // แยกจำนวนเต็ม กับ ทศนิยม
	            uint32_t freq_int = freq_x100 / 100;
	            uint32_t freq_dec = freq_x100 % 100;

	            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
	                // ปริ้นโดยใช้ %lu ทั้งหมด
//	                printf("normal : %lu.%",g_normal_counter);
	                xSemaphoreGive(uart_mutex);
	            }

	            // Reset
	            loop_counter = 0;
	            start_tim2 = current_tim2;
	        }
	    }
}



void SMARM_Experiment_Task(void *argument)
{
    (void) argument;

    portALLOCATE_SECURE_CONTEXT(4096);

    uint8_t digest[32];
    uint8_t challenge[16];

    osDelay(3000);

    for(;;)
    {

        uint32_t seed = osKernelGetTickCount();
        for(int k=0; k<4; k++) {
            uint32_t rnd = seed ^ (seed << 13) ^ (k * 0x5DEECE66D);
            memcpy(&challenge[k*4], &rnd, 4);
        }

        uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t start_systick = osKernelGetTickCount();
        uint32_t start_count = g_normal_counter;

//        SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));


        SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));


        uint32_t end_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t end_systick = osKernelGetTickCount();
        uint32_t end_count = g_normal_counter;

        uint32_t actual_run = end_count - start_count; // จำนวนรอบที่ NormalTask รันได้จริง
        uint32_t duration_os_ms = end_systick - start_systick;
        uint32_t tim2_diff = end_tim2 - start_tim2;
        uint32_t actual_duration_ms = ((uint64_t)tim2_diff * 1000) / 137500; // แปลง Ticks เป็น ms

        uint32_t expected_run = (actual_duration_ms * TARGET_FREQ_HZ) / 1000;
        int32_t missed_cycles = (int32_t)expected_run - (int32_t)actual_run;

       // uint32_t expected_run = actual_duration_ms;     // เพราะ 1000Hz = 1 รอบต่อ 1ms
        //int32_t missed_cycles = (int32_t)expected_run - (int32_t)actual_run;

//	    printf("dur: %d  | tim: %d\r\n", duration_os_ms, tim2_diff);

        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    printf("\r\n--- Attestation Event Analysis ---\r\n");
                    printf("Duration (TIM2): %lu ms\r\n", actual_duration_ms);
                    printf("NormalTask Run: %lu / %lu cycles\r\n", actual_run, expected_run);
                    printf("Missed Cycles: %ld\r\n", missed_cycles);
                    printf("Systick: %ld\r\n", duration_os_ms);

                    // คำนวณ FAR เฉพาะช่วงเวลาที่รัน Secure Call
                    if (expected_run > 0) {
                       // uint32_t local_far_x100 = (actual_run * 100) / expected_run;
                        // printf("Local FAR: %lu.%02lu\r\n", local_far_x100 / 100, local_far_x100 % 100);
                    	uint32_t local_far_x100 = (actual_run * 100) / expected_run;
                    	printf("Local FAR: %lu.%02lu\r\n", local_far_x100 / 100, local_far_x100 % 100);
                    }
                    xSemaphoreGive(uart_mutex);
                }


        osDelay(2000);
    }
}
void NS_SMARM_measurement(void *argument) {
    uint8_t digest[32];
    hmac_sha256 hmac_ctx;
    uint32_t start_tim, end_tim;

    printf("\r\n--- 512KB Multi-Pass Attestation Measurement (10 rounds) ---\r\n");

    for (int run = 1; run <= 10; run++) {

        // 1) เตรียม shuffle index
        for (int i = 0; i < NS_BLOCKS; i++) stored_indices[i] = i;
        for (int i = NS_BLOCKS - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            int tmp = stored_indices[i];
            stored_indices[i] = stored_indices[j];
            stored_indices[j] = tmp;
        }

        // 2) เตรียม HMAC
        hmac_sha256_initialize(&hmac_ctx, (uint8_t*)"key", 3);

        // จุดที่จะวัด (5 ตำแหน่ง) – ปรับได้ตามต้องการ
        int p1 = 10;                  // “เริ่มสักรอบที่ 10”
        int p2 = NS_BLOCKS / 4;       // ประมาณ 1/4
        int p3 = NS_BLOCKS / 2;       // ครึ่งหนึ่ง
        int p4 = (3 * NS_BLOCKS) / 4; // 3/4
        int p5 = NS_BLOCKS - 2;       // รอบก่อนรอบสุดท้าย (index N-2; N-1 คือสุดท้าย)

        // 3) วนลูป 2 pass เพื่อให้ได้ 512KB
        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
            for (int j = 0; j < NS_BLOCKS; j++) {

                uint8_t *block_ptr = ns_test_memory + (stored_indices[j] * NS_BLOCK_SIZE);

                start_tim = __HAL_TIM_GET_COUNTER(&htim2);
                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
                end_tim = __HAL_TIM_GET_COUNTER(&htim2);

                // วัดเฉพาะบางตำแหน่ง
                if (j == p1 || j == p2 || j == p3 || j == p4 || j == p5) {
                    uint32_t cycles = end_tim - start_tim;
                    printf("Round %d | idx %d/%d | Stored: %lu cycles\r\n",
                           run, j, NS_BLOCKS, (unsigned long)cycles);
                }

                // ถ้าไม่อยากหน่วงก็เอา osDelay ออก
                // osDelay(200);
            }
        }

        hmac_sha256_finalize(&hmac_ctx, digest, 0);
    }
}


void Test_SpeckShuffle_Basic(void *argument)
{
	uint32_t maxblocks = NS_BLOCKS;
	    uint32_t seed = 0x12345678;  // หรือ osKernelGetTickCount() ก็ได้

	    for (int pass = 0; pass < 10; pass++) {   // 10 รอบตามที่ขอ
	        uint32_t tweak = seed + pass;

	        printf("\r\nPass %d (tweak = 0x%08lX):\r\n[", pass,
	               (unsigned long)tweak);

	        for (uint32_t i = 0; i < maxblocks; i++) {
	            uint32_t p = FF1Permute_Speck(i, maxblocks, &ff1_speck_key, tweak);

	            printf("%lu", (unsigned long)p);
	            if (i + 1 < maxblocks) {
	                printf(", ");
	            }
	        }

	        printf("]\r\n");
	    }
}


void NS_SMARM_Benchmark_Experiment(void *argument) {
    uint8_t digest[32];
    hmac_sha256 hmac_ctx;
    uint32_t start_tim, end_tim;
    uint64_t sum_stored = 0, sum_feistel = 0;

    printf("\r\n Block Size  %d---\r\n", NS_BLOCK_SIZE);

    for (int run = 1; run <= 10; run++) {
    	// Store
        for (int i = 0; i < NS_BLOCKS; i++) stored_indices[i] = i;
        for (int i = NS_BLOCKS - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            int tmp = stored_indices[i];
            stored_indices[i] = stored_indices[j];
            stored_indices[j] = tmp;
        }

        start_tim = __HAL_TIM_GET_COUNTER(&htim2);
        hmac_sha256_initialize(&hmac_ctx, (uint8_t*)"key", 3);

        // วนลูป 2 รอบเพื่อให้ได้ 512KB
        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
            for (int i = 0; i < NS_BLOCKS; i++) {
                uint8_t *block_ptr = ns_test_memory + (stored_indices[i] * NS_BLOCK_SIZE);
                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
            }
        }
        hmac_sha256_finalize(&hmac_ctx, digest, 0);
        end_tim = __HAL_TIM_GET_COUNTER(&htim2);

        uint32_t round_stored = (end_tim - start_tim);
        sum_stored += round_stored;

        // ==========================================================
        // 2. METHOD: FEISTEL ON-THE-FLY (512KB)
        // ==========================================================

// Custom FF1
//        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
//            for (uint32_t i = 0; i < NS_BLOCKS; i++) {
//                uint32_t next_idx = FeistelPermute(i, NS_BLOCKS, f_seed + pass);
//                uint8_t *block_ptr = ns_test_memory + (next_idx * NS_BLOCK_SIZE);
//                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
//            }
//        }

//        FF1



        uint32_t f_seed = (uint32_t)osKernelGetTickCount() + run;

        start_tim = __HAL_TIM_GET_COUNTER(&htim2);
        hmac_sha256_initialize(&hmac_ctx, (uint8_t*)"key", 3);

//
//        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
//            for (uint32_t i = 0; i < NS_BLOCKS; i++) {
//                uint32_t tweak = f_seed + pass;
//                uint32_t next_idx = FF1Permute(i, NS_BLOCKS, &ff1_key, tweak);
//                uint8_t *block_ptr = ns_test_memory + (next_idx * NS_BLOCK_SIZE);
//                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
//            }
//        }

        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
            uint32_t tweak = f_seed + pass;

            for (uint32_t i = 0; i < NS_BLOCKS; i++) {
                uint32_t next_idx = FF1Permute_Speck(i, NS_BLOCKS, &ff1_speck_key, tweak);
                uint8_t *block_ptr = ns_test_memory + (next_idx * NS_BLOCK_SIZE);
                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
            }
        }

//
//        for (int pass = 0; pass < PASSES_REQUIRED; pass++) {
//            uint32_t tweak = f_seed + pass;         // ใช้เป็น seed/nonce ต่อ pass
//
//            for (uint32_t i = 0; i < NS_BLOCKS; i++) {
//                uint32_t next_idx = SpeckShuffle_Index(&g_speck_shuffle,
//                                                       i,
//                                                       pass,
//                                                       tweak,
//                                                       NS_BLOCKS);
//
//                uint8_t *block_ptr = ns_test_memory + (next_idx * NS_BLOCK_SIZE);
//                hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
//            }
//        }
//
        hmac_sha256_finalize(&hmac_ctx, digest, 0);
        end_tim = __HAL_TIM_GET_COUNTER(&htim2);

        uint32_t round_feistel = (end_tim - start_tim);
        sum_feistel += round_feistel;

        printf("Round %d | Stored: %lu | Feistel: %lu (Cycles)\r\n", run, round_stored, round_feistel);
        osDelay(500);
    }

    // --- สรุปผลเฉลี่ย ---
    printf("\r\n--- Final Result for 512KB ---\r\n");
    printf("Average Stored Array: %llu cycles\r\n", sum_stored / 7);
    printf("Average Feistel Mode:  %llu cycles\r\n", sum_feistel / 7);
    printf("------------------------------\r\n");
}


void NS_SMARM_Benchmark_Experiment_test(void *argument) {
    uint8_t digest[32];
    hmac_sha256 hmac_ctx;
    uint32_t start_tim, end_tim;

    // ตัวแปรเก็บผลลัพธ์รวมเพื่อหาค่าเฉลี่ย
    uint64_t sum_stored = 0;
    uint64_t sum_feistel = 0;
    uint32_t total_stored, total_feistel;

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        printf("\r\n==============================================\r\n");
        printf("   SMARM BENCHMARK - 7 ROUNDS TEST\r\n");
        printf("   Blocks: %d | Block Size: %d Bytes\r\n", NS_BLOCKS, NS_BLOCK_SIZE);
        printf("==============================================\r\n");
        xSemaphoreGive(uart_mutex);
    }

    for (int run = 1; run <= 7; run++) {
        // --- 1. METHOD: STORED ARRAY ---
        // เราจะไม่เอาเวลา Shuffle มารวมในลูป Hash เพื่อให้เห็นความต่างชัดเจน
        uint32_t s_start = __HAL_TIM_GET_COUNTER(&htim2);
        for (int i = 0; i < NS_BLOCKS; i++) stored_indices[i] = i;
        // Shuffle (Simulated with rand for NS test)
        for (int i = NS_BLOCKS - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            int tmp = stored_indices[i];
            stored_indices[i] = stored_indices[j];
            stored_indices[j] = tmp;
        }

        start_tim = __HAL_TIM_GET_COUNTER(&htim2);
//        __disable_irq();
        hmac_sha256_initialize(&hmac_ctx, (uint8_t*)"key", 3);
        for (int i = 0; i < NS_BLOCKS; i++) {
            uint8_t *block_ptr = ns_test_memory + (stored_indices[i] * NS_BLOCK_SIZE);
            hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
        }
        hmac_sha256_finalize(&hmac_ctx, digest, 0);
//        __enable_irq();
        end_tim = __HAL_TIM_GET_COUNTER(&htim2);

        total_stored = (end_tim - s_start); // รวมทั้ง Shuffle และ Hash
        sum_stored += total_stored;

        // --- 2. METHOD: FEISTEL ON-THE-FLY ---
        uint32_t f_seed = (uint32_t)osKernelGetTickCount() + run;

        start_tim = __HAL_TIM_GET_COUNTER(&htim2);
//        __disable_irq();
        hmac_sha256_initialize(&hmac_ctx, (uint8_t*)"key", 3);
        for (uint32_t i = 0; i < NS_BLOCKS; i++) {
            uint32_t next_idx = FeistelPermute(i, NS_BLOCKS, f_seed);
            uint8_t *block_ptr = ns_test_memory + (next_idx * NS_BLOCK_SIZE);
            hmac_sha256_update(&hmac_ctx, block_ptr, NS_BLOCK_SIZE);
        }
        hmac_sha256_finalize(&hmac_ctx, digest, 0);
//        __enable_irq();
        end_tim = __HAL_TIM_GET_COUNTER(&htim2);

        total_feistel = (end_tim - start_tim);
        sum_feistel += total_feistel;

        // พิมพ์ผลรายรอบ
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf("Round %d | Stored: %lu cycles | Feistel: %lu cycles\r\n",
                    run, total_stored, total_feistel);
            xSemaphoreGive(uart_mutex);
        }
        osDelay(1000); // พักช่วงสั้นๆ
    }

    // --- สรุปผลการทดลอง ---
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        printf("----------------------------------------------\r\n");
        printf("AVERAGE RESULTS (7 Rounds)\r\n");
        printf(" - Avg Stored Array:  %llu cycles\r\n", sum_stored / 7);
        printf(" - Avg Feistel Mode:   %llu cycles\r\n", sum_feistel / 7);
        printf(" - RAM Saving:        %d bytes\r\n", sizeof(stored_indices));
        printf("==============================================\r\n");
        xSemaphoreGive(uart_mutex);
    }
}



void LinearHMAC(void *argument)
{
    (void) argument;
//    osDelay(1000);
    portALLOCATE_SECURE_CONTEXT(2048);

    uint8_t digest[32];

    for(;;)
    {

    	for(int i = 1 ; i<=5 ;i++){


//			if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//				printf("------------------------\r\n");
//				printf("Roundddd %d\r\n", i);
//				xSemaphoreGive(uart_mutex);
//			}

			// Capture start timestamps
			uint32_t systick_before = osKernelGetTickCount();
			uint32_t tim2_before = __HAL_TIM_GET_COUNTER(&htim2);

			// Execute HMAC in secure world
//			__disable_irq();
//			osDelay(5);
//			SECURE_LinearHMAC(digest, sizeof(digest));
			SECURE_ShuffledHMAC(digest, sizeof(digest));

//			__enable_irq();

			// Capture end timestamps
			uint32_t tim2_after = __HAL_TIM_GET_COUNTER(&htim2);
			uint32_t systick_after = osKernelGetTickCount();

			// ========== Calculate elapsed time ==========
			uint32_t systick_elapsed = systick_after - systick_before;
			uint32_t tim2_elapsed = tim2_after - tim2_before;

			// SysTick: Tick rate is 10000 Hz = 100μs per tick
			uint32_t systick_us = systick_elapsed * 100;
			uint32_t systick_ms = systick_us / 1000;

			// TIM2: Each count = 7.272727 μs (with prescaler 799 @ 110MHz)
			// Formula: time_us = (counts × 800) / 110
			uint32_t tim2_us = (tim2_elapsed * 800) / 110;
			uint32_t tim2_ms = tim2_us / 1000;

			// ===== CALCULATE DIFFERENCE =====
			int32_t time_difference_us = (int32_t)tim2_us - (int32_t)systick_us;

			// Accuracy calculation (in 0.01% units to avoid float)
			// accuracy = (systick / tim2) × 10000 → gives accuracy in 0.01% units
			uint32_t accuracy_x100 = 0;
			uint32_t loss_x100 = 0;

			if (tim2_us > 0) {
				accuracy_x100 = ((uint64_t)systick_us * 10000ULL) / tim2_us;
				loss_x100 = 10000 - accuracy_x100;  // Loss × 100
			}

			// ========== Print results ==========
//			if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//				printf("TIMER cycles: %lu\r\n", tim2_elapsed);
//				printf("TIMER ms: %lu\r\n", tim2_ms);
//				printf("Systick tick: %lu\r\n", systick_elapsed);
//				printf("Systick ms: %lu\r\n", systick_ms);
//				xSemaphoreGive(uart_mutex);
//			}
//			osDelay(1000);
    	}
    	osDelay(1);
    }
}

/* USER CODE BEGIN Header_StartTask02 */
	/**
	 * @brief Function implementing the myTask02 thread.
	 * @param argument: Not used
	 * @retval None
	 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  (void) argument;

  osDelay(1000);
//  portALLOCATE_SECURE_CONTEXT(configMINIMAL_SECURE_STACK_SIZE);
  portALLOCATE_SECURE_CONTEXT(2048);

  char buffer[64];
  uint8_t result[32];
  uint8_t hmac_result[32];
  uint8_t digest[32];
  uint32_t start_tick = 0;
  uint8_t challenge[16];



  for(;;)
  {
;

	  	 osDelay(1000);
	     uint32_t before = HAL_GetTick();
//
	     if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
	     {
	         printf("SecureS: %lu\r\n", before);
	         xSemaphoreGive(uart_mutex);
	     }

//	     SECURE_LinearHMAC(digest, sizeof(digest));
//	     SECURE_ShuffledHMAC(digest, sizeof(digest));

//	     for (int i = 0; i < 4; i++) {
//	         uint32_t w = HAL_GetTick() ^ (0x9E3779B9u * i);
//	         memcpy(&challenge[4*i], &w, 4);
//	     }

//	     SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));


	     uint32_t after = HAL_GetTick();
	     if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
	     {
	         printf("SecureE: %lu\r\n", after);
	         xSemaphoreGive(uart_mutex);
	     }

  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
		/* LED3 on */
		BSP_LED_On(LED3);

		/* Infinite loop */
		while (1) {
		}

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
		/* Infinite loop */
		while (1)
		{
		}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
