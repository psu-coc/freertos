/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    FreeRTOS/FreeRTOS_SecureIOToggle_TrustZone/NonSecure/Src/main.c
 * @author  MCD Application Team
 * @brief   Main program body (non-secure)
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
#include <string.h>

SemaphoreHandle_t uart_mutex;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef hlpuart1;

osThreadId_t LEDThreadHandleHandle;
const osThreadAttr_t LEDThreadHandle_attributes = {
  .name = "LEDThreadHandle",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
void SecureFault_Callback(void);
void SecureError_Callback(void);
void NormalTask(void *argument);
void SMARM_Experiment_Task(void *argument);
static void ns_dwt_enable_cycle_counter(void);
static uint64_t ns_cycles_to_energy_uj(uint32_t cycles);
static void ns_fmt_energy_mj6(char out[32], uint64_t energy_uj);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
  ns_dwt_enable_cycle_counter();
  osKernelInitialize();
  uart_mutex = xSemaphoreCreateMutex();
  if (uart_mutex == NULL) {
      Error_Handler();
  }
  //  LEDThreadHandleHandle = osThreadNew(NormalTask, NULL, &LEDThreadHandle_attributes);
    LEDThreadHandleHandle = osThreadNew(NormalTask, NULL, &LEDThreadHandle_attributes);
  myTask02Handle = osThreadNew(SMARM_Experiment_Task, NULL, &myTask02_attributes);
  osKernelStart();
  while (1) {}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK) { Error_Handler(); }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_LPUART1_UART_Init(void)
{
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
  if (HAL_UART_Init(&hlpuart1) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
}

#include <sys/time.h>
int _gettimeofday(struct timeval *tv, void *tzvp) { (void)tv; (void)tzvp; return 0; }
void SecureFault_Callback(void) { Error_Handler(); }
int __io_putchar(int ch) { HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY); return ch; }
void SecureError_Callback(void) { Error_Handler(); }

#define TARGET_FREQ_HZ        1000
#define TIM2_TICKS_PER_SEC    137500U
#define SYSCLK_HZ             110000000UL
/** Supply and average run current for E(mJ) estimate — calibrate if you have a meter. */
#define SUPPLY_VOLTAGE_MV     3300U
#define AVG_RUN_CURRENT_UA    8000U

volatile uint32_t g_normal_counter = 0; // ตัวนับรอบของ NormalTask

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
            (void)time_s_int; (void)time_s_dec; (void)freq_int; (void)freq_dec;
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) { xSemaphoreGive(uart_mutex); }
            loop_counter = 0;
            start_tim2 = current_tim2;
        }
    }
}

void SMARM_Experiment_Task(void *argument)
{
    (void) argument;
    portALLOCATE_SECURE_CONTEXT(4096);
    static uint32_t durations_ms[10];
    static uint32_t cycles_arr[10];
    static uint64_t energy_uj_arr[10];
    uint8_t digest[32];
    uint8_t challenge[16];
    osDelay(3000);
    for(uint8_t round = 0; round < 10; round++)
    {
        uint32_t seed = osKernelGetTickCount();
        for(int k=0; k<4; k++) {
            uint32_t rnd = seed ^ (seed << 13) ^ (k * 0x5DEECE66D);
            memcpy(&challenge[k*4], &rnd, 4);
        }
        uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t start_count = g_normal_counter;
        uint32_t start_cycles = DWT->CYCCNT;
        SECURE_ShuffledHMAC_secure(digest, sizeof(digest), challenge, sizeof(challenge));
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
        printf("\r\n=== SUMMARY SMARM Baseline (10 rounds) ===\r\n");
        printf("V=%u mV, I_avg=%u uA, SYSCLK=%lu Hz (estimated energy)\r\n",
               (unsigned)SUPPLY_VOLTAGE_MV, (unsigned)AVG_RUN_CURRENT_UA, (unsigned long)SYSCLK_HZ);
        printf("Runtime  mean=%lu ms, min=%lu ms, max=%lu ms\r\n",
               (unsigned long)mean_ms, (unsigned long)min_ms, (unsigned long)max_ms);
        printf("Cycles   mean=%lu, min=%lu, max=%lu\r\n",
               (unsigned long)mean_cycles, (unsigned long)min_cycles, (unsigned long)max_cycles);
        printf("Energy   mean=%s mJ, min=%s mJ, max=%s mJ\r\n",
               emj_mean, emj_min, emj_max);
        printf("==========================================\r\n");
        xSemaphoreGive(uart_mutex);
    }
    for(;;) { osDelay(10000); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
  while (1) {}
}
#endif /* USE_FULL_ASSERT */
