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
void LinearHmac(void *argument);

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
//  LEDThreadHandleHandle = osThreadNew(LED_Thread, NULL, &LEDThreadHandle_attributes);

  /* creation of myTask02 */
//  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  myTask02Handle = osThreadNew(StartTask08, NULL, &myTask02_attributes);

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

void uint32_to_str(uint32_t value, char *buffer) {
    char temp[11];  // Max 10 digits for 32-bit unsigned + null terminator
    int i = 0;

    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    while (value != 0) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }

    // Reverse the string into the output buffer
    int j = 0;
    while (i > 0) {
        buffer[j++] = temp[--i];
    }
    buffer[j] = '\0';
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
                printf("❌ ERASE FAILED!\r\n");
                xSemaphoreGive(uart_mutex);
            }
            osDelay(10000);
            continue;
        }

        // Calculate erase elapsed times
        uint32_t erase_rtos_ms = erase_rtos_end - erase_rtos_start;

        // TIM2 erase time with overflow handling (microseconds)
        uint32_t erase_tim2_us;
        if (erase_tim2_end >= erase_tim2_start) {
            erase_tim2_us = erase_tim2_end - erase_tim2_start;
        } else {
            // Handle 32-bit overflow
            erase_tim2_us = (0xFFFFFFFF - erase_tim2_start) + erase_tim2_end + 1;
        }

        // Convert to nanoseconds
        uint64_t erase_tim2_ns = (uint64_t)erase_tim2_us * 1000ULL;

        // Calculate sub-millisecond precision for erase
        uint32_t erase_sub_ms_us = erase_tim2_us % 1000;

        // ================================================================
        // STEP 2: WRITE MEASUREMENT (Both RTOS ticks and TIM2)
        // ================================================================

        // Capture write start timestamps
        uint32_t write_rtos_start = osKernelGetTickCount();
        uint32_t write_tim2_start = __HAL_TIM_GET_COUNTER(&htim2);

        // === Execute secure flash write operation ===
        Secure_WriteFlash_128KB(&success_words, &failed_words);

        // Capture write end timestamps immediately
        uint32_t write_tim2_end = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t write_rtos_end = osKernelGetTickCount();

        // ================================================================
        // STEP 3: CALCULATE WRITE ELAPSED TIMES
        // ================================================================

        // RTOS tick time (milliseconds)
        uint32_t write_rtos_ms = write_rtos_end - write_rtos_start;

        // TIM2 time with overflow handling (microseconds)
        uint32_t write_tim2_us;
        if (write_tim2_end >= write_tim2_start) {
            write_tim2_us = write_tim2_end - write_tim2_start;
        } else {
            // Handle 32-bit overflow
            write_tim2_us = (0xFFFFFFFF - write_tim2_start) + write_tim2_end + 1;
        }

        // Convert to nanoseconds for comparison with Zephyr
        uint64_t write_tim2_ns = (uint64_t)write_tim2_us * 1000ULL;

        // Calculate sub-millisecond precision for write
        uint32_t write_sub_ms_us = write_tim2_us % 1000;

        // ================================================================
        // STEP 4: PRINT RESULTS
        // ================================================================
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("\r\n=== Hot Patch Timing Measurement ===\r\n");
            printf("\n--- ERASE PHASE ---\n");
            printf("Erase time:      %5lu ms  (RTOS)\r\n", erase_rtos_ms);
            printf("Erase time:      %5lu us  (TIM2)\r\n", erase_tim2_us);
            printf("Erase time:      %5llu ns  (TIM2)\r\n", erase_tim2_ns);
            printf("Sub-ms detail:   %5lu us  (lost by RTOS)\r\n", erase_sub_ms_us);

            printf("\n--- WRITE PHASE ---\n");
            printf("Write time:      %5lu ms  (RTOS)\r\n", write_rtos_ms);
            printf("Write time:      %5lu us  (TIM2)\r\n", write_tim2_us);
            printf("Write time:      %5llu ns  (TIM2)\r\n", write_tim2_ns);
            printf("Sub-ms detail:   %5lu us  (lost by RTOS)\r\n", write_sub_ms_us);

            printf("\n--- TOTAL TIME ---\n");
            uint32_t total_rtos_ms = erase_rtos_ms + write_rtos_ms;
            uint32_t total_tim2_us = erase_tim2_us + write_tim2_us;
            printf("Total time:      %5lu ms  (RTOS)\r\n", total_rtos_ms);
            printf("Total time:      %5lu us  (TIM2)\r\n", total_tim2_us);
            printf("Total time:      %5llu ns  (TIM2)\r\n", (uint64_t)total_tim2_us * 1000ULL);

            printf("\n--- STATUS ---\n");
            if (success_words == total_words) {
                printf("Status:          SUCCESS ✅\r\n");
            } else {
                printf("Status:          FAILED ❌\r\n");
                printf("Success/Total:   %lu / %lu words\r\n", success_words, total_words);
            }

            printf("========================================\r\n\r\n");
            xSemaphoreGive(uart_mutex);
        }

        osDelay(10000);
    }
}




//void StartTask08(void *argument)
//{
//    (void) argument;
//
//    portALLOCATE_SECURE_CONTEXT(4096);
//
//    osDelay(2000);
//
//
//    __HAL_DBGMCU_FREEZE_TIM2();
//
//    FlashResult_t result = {0};
//    char  timerdata[32];
//    uint32_t start_tick, end_tick;
//    uint32_t timer_start, timer_end;
//    uint32_t systick_time, timer_time;
//    HAL_StatusTypeDef flash_status; // Variable to store function result
//
//    uint32_t success_words = 0;
//    uint32_t failed_words = 0;
//    uint32_t total_words = 16384;  // (254KB / 8)
//
////    flash_status = EraseFlashPages_256KB();
//    HAL_StatusTypeDef erase_status;
//
//
//    for(;;)
//    {
//
//        uint32_t erase_start = osKernelGetTickCount();
//
//        erase_status = EraseFlashPages_256KB();
//
//        uint32_t erase_time = osKernelGetTickCount() - erase_start;
//
//        if (erase_status != HAL_OK) {
//              if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//                  printf("ERASE FAILED!\r\n");
//                  xSemaphoreGive(uart_mutex);
//              }
//              osDelay(10000);
//              continue;
//          }
//
//        // Reset timer
//        __HAL_TIM_SET_COUNTER(&htim2, 0);
//
//        // Start measurement
//        start_tick = osKernelGetTickCount();
//        timer_start = __HAL_TIM_GET_COUNTER(&htim2);
//
//        // === Run Flash Test ===
//
//        Secure_WriteFlash_128KB(&success_words, &failed_words);
//
//        // End measurement
//
//        end_tick = osKernelGetTickCount();
//        timer_end = __HAL_TIM_GET_COUNTER(&htim2);
//
//        // Calculate
//        systick_time = end_tick - start_tick;
//        timer_time = timer_end - timer_start;
//		uint32_to_str(timer_start, timerdata);
//
//
//        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
//             {
//                 printf("\r\n--- Timing ---\r\n");
//                 printf("Erase:  %lu \r\n", erase_time);
//                 printf("Write:  %lu \r\n", systick_time);
//                 printf("TIM2:   %lu \r\n", timer_time);
//
//                 if (success_words == total_words) {
//                     printf("Status:      SUCCESS! ✅\r\n");
//                 } else {
//                     printf("Status:      FAILED! ❌\r\n");
//                 }
//
//                 printf("========================================\r\n\r\n");
//                 xSemaphoreGive(uart_mutex);
//             }
//
//             osDelay(10000);
//
//
//    }
//}



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

void LinearHmac(void *argument) {
    (void) argument;
    osDelay(1000);
    portALLOCATE_SECURE_CONTEXT(2048);

    uint8_t digest[32];



    for(;;) {
        osDelay(1000);  // Wait 1 second between measurements


//        uint32_t systick_before = HAL_GetTick();
        uint32_t systick_before = osKernelGetTickCount();
        uint32_t tim2_before = __HAL_TIM_GET_COUNTER(&htim2);

//        __disable_irq();
        SECURE_LinearHMAC(digest, sizeof(digest));
//        __enable_irq();
        uint32_t tim2_after = __HAL_TIM_GET_COUNTER(&htim2);
//        uint32_t systick_after = HAL_GetTick();
        uint32_t systick_after = osKernelGetTickCount();

        uint32_t systick_elapsed = systick_after - systick_before;

        // Handle TIM2 overflow (if counter wrapped around)
        uint32_t tim2_elapsed;
        if (tim2_after >= tim2_before) {
            tim2_elapsed = tim2_after - tim2_before;
        } else {
            // Overflow occurred
            tim2_elapsed = (0xFFFFFFFF - tim2_before) + tim2_after + 1;
        }


        uint64_t tim2_elapsed_ns = (uint64_t)tim2_elapsed * 1000ULL;


        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            static int round = 1;

            printf("\r\n=== Round %d ===\r\n", round);
            printf("SysTick elapsed: %lu ticks\r\n", systick_elapsed);
            printf("TIM2 cycles: %lu cycles\r\n", tim2_elapsed);
            printf("Time (ns): %llu ns\r\n", tim2_elapsed_ns);
            printf("Time (us): %lu us\r\n", tim2_elapsed);
            printf("Status: OK\r\n");

            round++;
            xSemaphoreGive(uart_mutex);
        }
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

	     for (int i = 0; i < 4; i++) {
	         uint32_t w = HAL_GetTick() ^ (0x9E3779B9u * i);
	         memcpy(&challenge[4*i], &w, 4);
	     }

	     SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));


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
