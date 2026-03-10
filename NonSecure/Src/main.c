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


/* USER CODE BEGIN PFP */
void SecureFault_Callback(void);
void SecureError_Callback(void);
void NormalTask(void *argument);
void SMARM_Experiment_Task(void *argument);
void RTSMARM_Test_NormalWorld(void *argument);



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
 LEDThreadHandleHandle = osThreadNew(NormalTask, NULL, &LEDThreadHandle_attributes);


  /* creation of myTask02 */

  myTask02Handle = osThreadNew(SMARM_Experiment_Task, NULL, &myTask02_attributes);

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



uint8_t *ns_test_memory = (uint8_t *)0x08040000;


#define NS_CHUNK_SIZE     0x40000  // 256KB (พื้นที่ปลอดภัยใน Bank 2)
#define NS_BLOCK_SIZE     4096
#define NS_BLOCKS         (NS_CHUNK_SIZE / NS_BLOCK_SIZE) // 256 Blocks
#define TARGET_TOTAL_KB   512
#define PASSES_REQUIRED   (TARGET_TOTAL_KB * 1024 / NS_CHUNK_SIZE)

// จอง RAM สำหรับวิธีเดิม (Stored Array) - วิธีนี้กิน RAM ตามจำนวน Block
static int stored_indices[NS_BLOCKS];


#define TARGET_FREQ_HZ   1000
#define TIM2_TICKS_PER_SEC  137500
volatile uint32_t g_normal_counter = 0; // ตัวนับรอบของ NormalTask

// ---------- RT-SMARM test in Normal world (10 blocks, printf verification) ----------
#define RTSMARM_TEST_BLOCKS      100
#define RTSMARM_TEST_BLOCK_SIZE  32
#define RTSMARM_SHA256_DIGEST    32

static uint8_t rtsmarm_test_memory[RTSMARM_TEST_BLOCKS * RTSMARM_TEST_BLOCK_SIZE];
static uint8_t rtsmarm_used_bits[(RTSMARM_TEST_BLOCKS + 7) / 8];
#define RTSMARM_MAX_BLOCKS 128
static int rtsmarm_order[RTSMARM_MAX_BLOCKS];
static int rtsmarm_seen[RTSMARM_MAX_BLOCKS];


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
    uint8_t round = 0;
    osDelay(3000);

    for(;;)
    {

    	round++;
        uint32_t seed = osKernelGetTickCount();
        for(int k=0; k<4; k++) {
            uint32_t rnd = seed ^ (seed << 13) ^ (k * 0x5DEECE66D);
            memcpy(&challenge[k*4], &rnd, 4);
        }

        uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t start_systick = osKernelGetTickCount();
        uint32_t start_count = g_normal_counter;

//        SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));

//        __disable_irq();
//        osDelay(1000);
//         SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));
        SECURE_RTSMARM_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));
//        __enable_irq();

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
//                    printf("\r\n--- Attestation Event Analysis ---\r\n");
//                    printf("Duration (TIM2): %lu ms\r\n", actual_duration_ms);
                    printf("Round %u NS: %lu / %lu cycles\r\n", round ,actual_run, expected_run);
//                    printf("Missed Cycles: %ld\r\n", missed_cycles);
                    printf("Systick: %ld\r\n", duration_os_ms);

                    // คำนวณ FAR เฉพาะช่วงเวลาที่รัน Secure Call
                    if (expected_run > 0) {
                       // uint32_t local_far_x100 = (actual_run * 100) / expected_run;
                        // printf("Local FAR: %lu.%02lu\r\n", local_far_x100 / 100, local_far_x100 % 100);
//                    	uint32_t local_far_x100 = (actual_run * 100) / expected_run;
//                    	printf("Local FAR: %lu.%02lu\r\n", local_far_x100 / 100, local_far_x100 % 100);
                    }
                    xSemaphoreGive(uart_mutex);
                }


        osDelay(2000);
    }
}



/**
 * RT-SMARM test: one function, all logic inside Normal world.
 * Uses 10 blocks by default; constants above can be changed.
 * Prints round, k, idx and verifies no duplicate indices.
 * ทดสอบใน Normal World
 */
 void RTSMARM_Test_NormalWorld(void *argument)
 {
 #define N          RTSMARM_TEST_BLOCKS
 #define BLK        RTSMARM_TEST_BLOCK_SIZE
 #define DIGEST_LEN RTSMARM_SHA256_DIGEST
 
     static const uint8_t key[] = "MySecureKey123";
     uint8_t key16[16], iv16[16];
     uint8_t challenge[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                               0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
     size_t challenge_len = 16;
 
     /* PRNG state (AES-CTR) */
     struct AES_ctx prng_ctx;
     uint8_t prng_buf[16];
     int prng_idx;
 
     hmac_sha256 hmac_deriv;
     hmac_sha256 hmac_attest;
    uint8_t digest[DIGEST_LEN];
    if (N > RTSMARM_MAX_BLOCKS) return;

     /* Fill test memory with a simple pattern (optional) */
     for (int i = 0; i < N * BLK; i++)
         rtsmarm_test_memory[i] = (uint8_t)(i & 0xFF);
 
     /* ---- Derive key/IV from challenge (HMAC) ---- */
     hmac_sha256_initialize(&hmac_deriv, key, strlen((const char*)key));
     hmac_sha256_update(&hmac_deriv, challenge, challenge_len);
     hmac_sha256_finalize(&hmac_deriv, NULL, 0);
     memcpy(key16, hmac_deriv.digest, 16);
     memcpy(iv16,  hmac_deriv.digest + 16, 16);
 
     /* ---- PRNG init (AES-CTR) ---- */
     AES_init_ctx_iv(&prng_ctx, key16, iv16);
     memset(prng_buf, 0, sizeof(prng_buf));
     prng_idx = 16;
 
     /* ---- Clear bit array ---- */
     memset(rtsmarm_used_bits, 0, sizeof(rtsmarm_used_bits));
 
     /* ---- HMAC for attestation ---- */
     hmac_sha256_initialize(&hmac_attest, key, strlen((const char*)key));
 
     for (int round = 0; round < N; round++) {
         int remaining = N - round;
         /* prng_uniform: get uniform in [0, remaining) */
         uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)remaining);
         uint32_t r;
         for (;;) {
             if (prng_idx > 12) {
                 uint8_t zero[16] = {0};
                 memcpy(prng_buf, zero, 16);
                 AES_CTR_xcrypt_buffer(&prng_ctx, prng_buf, 16);
                 prng_idx = 0;
             }
             memcpy(&r, &prng_buf[prng_idx], 4);
             prng_idx += 4;
             if (r < lim) break;
         }
         int k = (int)(r % (uint32_t)remaining);
 
         /* Find k-th zero in bit array (constant-time: scan all N) */
         int count = 0, idx = 0;
         for (int i = 0; i < N; i++) {
             int b = (rtsmarm_used_bits[i/8] >> (i%8)) & 1;
             if (b == 0) {
                 if (count == k) idx = i;
                 count++;
             }
         }
        rtsmarm_used_bits[idx/8] |= (1U << (idx%8));
        rtsmarm_order[round] = idx;
        hmac_sha256_update(&hmac_attest, &rtsmarm_test_memory[(size_t)idx * BLK], BLK);
 
        //  if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        //      printf("RTSMARM round %d: k=%d idx=%d\r\n", round, k, idx);
        //      xSemaphoreGive(uart_mutex);
        //  }
        if (round + 1 == 1 || round + 1 == 5 || round + 1 == 10 || round + 1 == N) {
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                printf("RTSMARM round %d: k=%d idx=%d\r\n", round, k, idx);
                printf("RTSMARM round %d bits = [", round + 1);
                for (int i = 0; i < N; i++) {
                    int b = (rtsmarm_used_bits[i/8] >> (i%8)) & 1;
                    printf("%d", b);
                    if (i < N - 1) printf(",");
                }
                printf("]\r\n");
                xSemaphoreGive(uart_mutex);
            }
        } else {
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                printf("RTSMARM round %d: k=%d idx=%d\r\n", round, k, idx);
                xSemaphoreGive(uart_mutex);
            }
        }
     }
 
     hmac_sha256_finalize(&hmac_attest, NULL, 0);
     memcpy(digest, hmac_attest.digest, DIGEST_LEN);
 
     /* ---- Verify no duplicate: each 0..N-1 appears exactly once ---- */
     memset(rtsmarm_seen, 0, (size_t)N * sizeof(int));
     for (int i = 0; i < N; i++) {
         if (rtsmarm_order[i] >= 0 && rtsmarm_order[i] < N)
             rtsmarm_seen[rtsmarm_order[i]]++;
     }
     int ok = 1;
     for (int i = 0; i < N; i++)
         if (rtsmarm_seen[i] != 1) { ok = 0; break; }

     if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
         printf("RTSMARM order: ");
         for (int i = 0; i < N; i++) printf("%d ", rtsmarm_order[i]);
         printf("\r\n");
         printf("RTSMARM no duplicate: %s\r\n", ok ? "OK" : "FAIL");
         printf("RTSMARM digest: ");
         for (int i = 0; i < 8; i++) printf("%02X", digest[i]);
         printf("...\r\n");
         xSemaphoreGive(uart_mutex);
     }
 
 #undef N
 #undef BLK
 #undef DIGEST_LEN
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
