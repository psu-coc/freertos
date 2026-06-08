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
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32l5xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

SemaphoreHandle_t uart_mutex;

TIM_HandleTypeDef htim2;

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
  .stack_size = 16 * 1024
};

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
static void ns_dwt_enable_cycle_counter(void);
static uint64_t ns_cycles_to_energy_uj(uint32_t cycles);
static void ns_fmt_energy_mj6(char out[32], uint64_t energy_uj);

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
  {
    const char boot[] = "\r\n[NS boot] ff1-energy ready. Starting FreeRTOS...\r\n";
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U), HAL_MAX_DELAY);
  }
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
  ns_dwt_enable_cycle_counter();

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
  if (myTask02Handle == NULL || LEDThreadHandleHandle == NULL) {
    Error_Handler();
  }

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

#include <sys/time.h>

int _gettimeofday(struct timeval *tv, void *tzvp) {
    (void)tv;
    (void)tzvp;
    return 0;
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


#define TARGET_FREQ_HZ        1000
#define TIM2_TICKS_PER_SEC    137500U
#define SYSCLK_HZ             110000000UL
/** Supply and average run current for E(mJ) estimate — calibrate if you have a meter. */
#define SUPPLY_VOLTAGE_MV     3300U
#define AVG_RUN_CURRENT_UA    8000U

volatile uint32_t g_normal_counter = 0;

static void ns_dwt_enable_cycle_counter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/** E(uJ) = V(mV) * I(uA) * cycles / (SYSCLK_HZ * 1000). Integer only. */
static uint64_t ns_cycles_to_energy_uj(uint32_t cycles)
{
  return ((uint64_t)cycles * (uint64_t)SUPPLY_VOLTAGE_MV * (uint64_t)AVG_RUN_CURRENT_UA)
         / ((uint64_t)SYSCLK_HZ * 1000ULL);
}

static void ns_fmt_energy_mj6(char out[32], uint64_t energy_uj)
{
  uint32_t mj_int = (uint32_t)(energy_uj / 1000ULL);
  uint32_t frac6 = (uint32_t)(((energy_uj % 1000ULL) * 1000000ULL) / 1000ULL);
  (void)snprintf(out, 32, "%lu.%06lu", (unsigned long)mj_int, (unsigned long)frac6);
}

void NormalTask(void *argument)
{
	(void) argument;

	    const uint32_t period_os_ticks = 1000 / TARGET_FREQ_HZ;

	    uint32_t next_wake_time = osKernelGetTickCount();
	    uint32_t loop_counter = 0;

	    uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);

	    for (;;)
	    {
	        next_wake_time += period_os_ticks;
	        osDelayUntil(next_wake_time);

	        g_normal_counter++;

	        loop_counter++;

	        uint32_t current_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
	        uint32_t elapsed_tim2 = current_tim2 - start_tim2;

	        if (elapsed_tim2 >= TIM2_TICKS_PER_SEC)
	        {
	            uint32_t time_ms = (uint64_t)elapsed_tim2 * 1000 / TIM2_TICKS_PER_SEC;

	            uint32_t time_s_int = time_ms / 1000;
	            uint32_t time_s_dec = time_ms % 1000;

	            uint32_t freq_x100 = ((uint64_t)loop_counter * TIM2_TICKS_PER_SEC * 100) / elapsed_tim2;

	            uint32_t freq_int = freq_x100 / 100;
	            uint32_t freq_dec = freq_x100 % 100;

	            (void)time_s_int;
	            (void)time_s_dec;
	            (void)freq_int;
	            (void)freq_dec;

	            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
	                xSemaphoreGive(uart_mutex);
	            }

	            loop_counter = 0;
	            start_tim2 = current_tim2;
	        }
	    }
}



void SMARM_Experiment_Task(void *argument)
{
    (void) argument;
    {
        const char banner[] = "\r\nSMARM+FPE FF1-Speck benchmark task started.\r\n";
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1U), HAL_MAX_DELAY);
    }
    {
        const char msg[] = "Allocating secure context (12 KiB)...\r\n";
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), HAL_MAX_DELAY);
    }
    portALLOCATE_SECURE_CONTEXT(12288);
    {
        const char msg[] = "Secure context OK. Waiting 3 s...\r\n";
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), HAL_MAX_DELAY);
    }
    static uint32_t durations_ms[10];
    static uint32_t cycles_arr[10];
    static uint64_t energy_uj_arr[10];
    uint8_t digest[32];
    uint8_t challenge[16];
    osDelay(3000);
    {
        const char msg[] = "Starting benchmark loop (10 rounds).\r\n";
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), HAL_MAX_DELAY);
    }
    for (uint8_t round = 0; round < 10; round++)
    {
        uint32_t seed = osKernelGetTickCount();
        for (int k = 0; k < 4; k++) {
            uint32_t rnd = seed ^ (seed << 13) ^ (k * 0x5DEECE66D);
            memcpy(&challenge[k * 4], &rnd, 4);
        }
        if (round == 0U) {
            const char msg[] = "Starting FF1-Speck attestation (round 1)...\r\n";
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), HAL_MAX_DELAY);
        }
        uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t start_count = g_normal_counter;
        uint32_t start_cycles = DWT->CYCCNT;
        SECURE_RTSMARM_FF1_Speck_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));
        uint32_t end_cycles = DWT->CYCCNT;
        uint32_t end_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t end_count = g_normal_counter;
        uint32_t actual_run = end_count - start_count;
        uint32_t tim2_diff = end_tim2 - start_tim2;
        uint32_t cpu_cycles = end_cycles - start_cycles;
        uint32_t actual_duration_ms = (uint32_t)(((uint64_t)tim2_diff * 1000ULL) / TIM2_TICKS_PER_SEC);
        uint64_t energy_uj = ns_cycles_to_energy_uj(cpu_cycles);
        durations_ms[round] = actual_duration_ms;
        cycles_arr[round] = cpu_cycles;
        energy_uj_arr[round] = energy_uj;
        uint32_t expected_run = (actual_duration_ms * TARGET_FREQ_HZ) / 1000;
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            char emj[32];
            ns_fmt_energy_mj6(emj, energy_uj);
            printf("Round %u: Runtime=%lu ms, Cycles=%lu, EstEnergy=%s mJ, NS=%lu/%lu\r\n",
                   (unsigned)(round + 1), (unsigned long)actual_duration_ms,
                   (unsigned long)cpu_cycles, emj,
                   (unsigned long)actual_run, (unsigned long)expected_run);
            xSemaphoreGive(uart_mutex);
        }
        osDelay(2000);
    }
    uint32_t sum_ms = 0, min_ms = durations_ms[0], max_ms = durations_ms[0];
    uint64_t sum_uj = 0, min_uj = energy_uj_arr[0], max_uj = energy_uj_arr[0];
    uint32_t sum_cycles = 0, min_cycles = cycles_arr[0], max_cycles = cycles_arr[0];
    for (int i = 0; i < 10; i++) {
        sum_ms += durations_ms[i];
        sum_cycles += cycles_arr[i];
        sum_uj += energy_uj_arr[i];
        if (durations_ms[i] < min_ms) min_ms = durations_ms[i];
        if (durations_ms[i] > max_ms) max_ms = durations_ms[i];
        if (cycles_arr[i] < min_cycles) min_cycles = cycles_arr[i];
        if (cycles_arr[i] > max_cycles) max_cycles = cycles_arr[i];
        if (energy_uj_arr[i] < min_uj) min_uj = energy_uj_arr[i];
        if (energy_uj_arr[i] > max_uj) max_uj = energy_uj_arr[i];
    }
    uint32_t mean_ms = sum_ms / 10U;
    uint32_t mean_cycles = sum_cycles / 10U;
    uint64_t mean_uj = sum_uj / 10ULL;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        char emj_mean[32], emj_min[32], emj_max[32];
        ns_fmt_energy_mj6(emj_mean, mean_uj);
        ns_fmt_energy_mj6(emj_min, min_uj);
        ns_fmt_energy_mj6(emj_max, max_uj);
        printf("\r\n=== SUMMARY SMARM+FPE FF1-Speck (10 rounds) ===\r\n");
        printf("V=%u mV, I_avg=%u uA, SYSCLK=%lu Hz (estimated energy)\r\n",
               (unsigned)SUPPLY_VOLTAGE_MV, (unsigned)AVG_RUN_CURRENT_UA, (unsigned long)SYSCLK_HZ);
        printf("Runtime  mean=%lu ms, min=%lu ms, max=%lu ms\r\n",
               (unsigned long)mean_ms, (unsigned long)min_ms, (unsigned long)max_ms);
        printf("Cycles   mean=%lu, min=%lu, max=%lu\r\n",
               (unsigned long)mean_cycles, (unsigned long)min_cycles, (unsigned long)max_cycles);
        printf("Energy   mean=%s mJ, min=%s mJ, max=%s mJ\r\n",
               emj_mean, emj_min, emj_max);
        printf("==========================================\r\n");
        printf("Done. Press RESET on board to run 10 rounds again.\r\n");
        xSemaphoreGive(uart_mutex);
    }
    for (;;) {
        osDelay(10000);
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
