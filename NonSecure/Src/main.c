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
#include <stddef.h>
#include "hmac-sha256.h"
#include "aes.h"
#include "secure_nsc.h"

/*
 * Board smoke test (branch smarmplus-rt-smoke), NonSecure-only path:
 *   No Secure attestation / NSC calls — Secure image only boots TrustZone.
 *   MODE 0: jumper D12<->D11, TIM3 1 kHz PWM + EXTI7.
 *   MODE 1: HW speed OUT->D11 + EXTI7.
 *   MODE 2: ultrasonic SIG->D11 only.
 *   MODE 3: ultrasonic D11 + optical wheel OUT->D12 (two FreeRTOS tasks).
 *   EXTI6/7 must be Non-Secure in partition ITNS0 (rebuild Secure for MODE 3).
 * Set PULSE_SMOKE_TEST 0 to restore FAR / Snap-Guard harness tasks.
 */
#define PULSE_SMOKE_TEST  1
#define PULSE_SMOKE_MODE  3   /* 0=PWM, 1=HW speed, 2=sonar, 3=sonar+wheel */

#define PULSE_SMOKE_USE_HW_SENSOR   (PULSE_SMOKE_MODE == 1)
#define PULSE_SMOKE_USE_ULTRASONIC  ((PULSE_SMOKE_MODE == 2) || (PULSE_SMOKE_MODE == 3))
#define PULSE_SMOKE_USE_WHEEL       (PULSE_SMOKE_MODE == 3)
#define PULSE_SMOKE_USE_PWM_LOOP    (PULSE_SMOKE_MODE == 0)

/* Optical disk has 20 slots — UART "speed" display */
#define PULSE_TEETH_PER_REV  20U
#define WHEEL_CIRCUM_MM      1900U

/* Ultrasonic SIG -> D11 = PA7 | Wheel OUT -> D12 = PA6 (separate holes on CN7) */
#define US_SIG_GPIO_Port  GPIOA
#define US_SIG_Pin        GPIO_PIN_7
#define US_ECHO_TIMEOUT_US  30000U
#define WHEEL_GPIO_Port   GPIOA
#define WHEEL_Pin         GPIO_PIN_6

/*
 * SAU-ATOMICCOPY / E4: set NS_APP_MODE_E4_ATOMIC to 1 for Atomic Copy t_disabled (TIM2).
 * Rebuild NonSecure (and flash both TZ images), change NS_HMAC_BLOCK_SIZE per sweep.
 */
#define NS_APP_MODE_E4_ATOMIC  0

#if NS_APP_MODE_E4_ATOMIC
#define NS_HMAC_BLOCK_SIZE         4096     /* E4 sweep: 64,128,256,512,1024,2048,4096 */
#define NS_E4_ROUNDS               10
#define NS_E4_SAMPLES_PER_RUN      32     /* timed IRQ-masked memcpy ops per round */
#define NS_E4_QUIET_UART           1
#else
#define TARGET_FREQ_HZ             1000   /* SMARM+SAU FAR sweeps */
#define NS_E4_QUIET_UART           0
#endif

#ifndef TARGET_FREQ_HZ
#define TARGET_FREQ_HZ             1000   /* NormalTask when E4 mode (FAR not measured) */
#endif

#if NS_APP_MODE_E4_ATOMIC && (NS_HMAC_BLOCK_SIZE > 4096U)
#error NS_HMAC_BLOCK_SIZE must be <= 4096 for E4 (static copy buffer)
#endif

SemaphoreHandle_t uart_mutex;
TIM_HandleTypeDef htim2;
#if PULSE_SMOKE_TEST
#if PULSE_SMOKE_USE_PWM_LOOP
TIM_HandleTypeDef htim3;
#endif
#if PULSE_SMOKE_USE_PWM_LOOP || PULSE_SMOKE_USE_HW_SENSOR
volatile uint32_t g_pulse_edges = 0U;
#endif
#if PULSE_SMOKE_USE_WHEEL
volatile uint32_t g_wheel_edges = 0U;
#endif
#endif
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3; /* NUCLEO ST-LINK VCP (USART3 on PD8/PD9) */

#define NS_UART_VCP_TIMEOUT_MS  500U

static void ns_uart_transmit(const uint8_t *data, uint16_t len)
{
  BaseType_t took = pdFALSE;
  if (!data || !len) {
    return;
  }
  if (uart_mutex != NULL) {
    took = xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(200));
  }
  /* LPUART1 (PG7): original project log port. USART3 (PD8): Nucleo ST-Link VCP. */
  (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, len, HAL_MAX_DELAY);
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, NS_UART_VCP_TIMEOUT_MS);
  if (took == pdTRUE) {
    (void)xSemaphoreGive(uart_mutex);
  }
}

osThreadId_t LEDThreadHandleHandle;
const osThreadAttr_t LEDThreadHandle_attributes = {
  .name = "LEDThreadHandle",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  /* sha256_update uses ~300B locals; shuffle/AES adds more — keep generous margin */
  .stack_size = 16 * 1024
};
#if PULSE_SMOKE_TEST && PULSE_SMOKE_USE_WHEEL
osThreadId_t wheelTaskHandle;
const osThreadAttr_t wheelTask_attributes = {
  .name = "WheelTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 2048
};
#endif

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void SecureFault_Callback(void);
void SecureError_Callback(void);
void NormalTask(void *argument);
void SMARM_Experiment_Task(void *argument);
void NS_HashBenchmark_Task(void *argument);
#if PULSE_SMOKE_TEST
static void PulseSmoke_Init(void);
void PulseSmoke_Task(void *argument);
#if PULSE_SMOKE_USE_WHEEL
void WheelSmoke_Task(void *argument);
#endif
#if PULSE_SMOKE_USE_PWM_LOOP
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
#endif
#if PULSE_SMOKE_USE_PWM_LOOP || PULSE_SMOKE_USE_HW_SENSOR || PULSE_SMOKE_USE_WHEEL
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
#endif
#endif

volatile uint32_t g_normal_counter = 0;
volatile uint32_t g_secure_busy = 0U;
volatile uint32_t g_attest_phase = 0U;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  {
#if PULSE_SMOKE_TEST
#if PULSE_SMOKE_USE_ULTRASONIC && PULSE_SMOKE_USE_WHEEL
    const char boot[] =
        "\r\n[NS boot] TWO TASKS (NS-only): sonar D11 + wheel D12.\r\n"
        "Ultrasonic SIG->D11; optical OUT->D12; both VCC=3V3 GND. VCP 115200.\r\n"
        "Starting FreeRTOS (SonarTask + WheelTask)...\r\n";
#elif PULSE_SMOKE_USE_ULTRASONIC
    const char boot[] =
        "\r\n[NS boot] ULTRASONIC (NS-only) — SIG->D11(PA7), VCC=3V3, GND.\r\n"
        "White NC = leave unplugged. No Secure attest. VCP 115200.\r\n"
        "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
#elif PULSE_SMOKE_USE_HW_SENSOR
    const char boot[] =
        "\r\n[NS boot] HW SPEED SENSOR (NS-only) — OUT->D11(PA7), Vcc/GND.\r\n"
        "Remove D12 jumper. Spin wheel; edges/s should track speed. VCP 115200.\r\n"
        "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
#else
    const char boot[] =
        "\r\n[NS boot] PULSE SMOKE (NS-only) — wire D12(PA6) <-> D11(PA7).\r\n"
        "No Secure attest. 10 ms parse + ~1000 edges/s. VCP 115200.\r\n"
        "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
#endif
#else
    const char boot[] =
#if NS_APP_MODE_E4_ATOMIC
        "\r\n[NS boot] E4 Atomic Copy — NS mirror, TIM2 on IRQ-masked memcpy.\r\n"
        "|M|=128KiB @0x08060000. Change NS_HMAC_BLOCK_SIZE then rebuild NonSecure.\r\n"
#else
#if USE_SAU_APPROACH
#if ATTEST_FAULT_DEMO
        "\r\n[NS boot] SMARM-Guard FAULT DEMO |M|=256KiB @0x08040000 (full NS flash).\r\n"
        "Expect SecureFault when SAU locks live FreeRTOS XIP. Not a production path.\r\n"
#else
        "\r\n[NS boot] SMARM-Guard |M|=128KiB @0x08060000, 1 pass.\r\n"
#endif
#elif USE_SNAP
        "\r\n[NS boot] SMARM-Snap |M|=128KiB @0x08060000, 1 pass.\r\n"
#else
        "\r\n[NS boot] SMARM Baseline |M|=128KiB @0x08060000, 1 pass.\r\n"
#endif
#endif
        "UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.\r\n"
#if NS_APP_MODE_E4_ATOMIC
        "Starting FreeRTOS (E4: NS_HashBenchmark_Task)...\r\n";
#else
#if USE_SAU_APPROACH
        "Starting FreeRTOS (FAR: Guard + NormalTask 1000 Hz)...\r\n";
#elif USE_SNAP
        "Starting FreeRTOS (FAR: Snap + NormalTask 1000 Hz)...\r\n";
#else
        "Starting FreeRTOS (FAR: Baseline + NormalTask 1000 Hz)...\r\n";
#endif
#endif
#endif /* !PULSE_SMOKE_TEST */
    ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
  }
#if PULSE_SMOKE_TEST
  PulseSmoke_Init();
#else
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
#endif
  osKernelInitialize();
  uart_mutex = xSemaphoreCreateMutex();
  if (uart_mutex == NULL) {
      Error_Handler();
  }
#if PULSE_SMOKE_TEST
  myTask02Handle = osThreadNew(PulseSmoke_Task, NULL, &myTask02_attributes);
  if (myTask02Handle == NULL) {
    Error_Handler();
  }
#if PULSE_SMOKE_USE_WHEEL
  wheelTaskHandle = osThreadNew(WheelSmoke_Task, NULL, &wheelTask_attributes);
  if (wheelTaskHandle == NULL) {
    Error_Handler();
  }
  {
    const char ok[] = "SonarTask + WheelTask created. Starting scheduler...\r\n";
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#else
  {
    const char ok[] = "PulseSmoke_Task created. Starting scheduler...\r\n";
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#endif
#else
  /* FAR run: Real-time NormalTask at TARGET_FREQ_HZ counts activations during attestation. */
#if !NS_APP_MODE_E4_ATOMIC
  LEDThreadHandleHandle = osThreadNew(NormalTask, NULL, &LEDThreadHandle_attributes);
  if (LEDThreadHandleHandle == NULL) {
    Error_Handler();
  }
#endif
#if NS_APP_MODE_E4_ATOMIC
  myTask02Handle = osThreadNew(NS_HashBenchmark_Task, NULL, &myTask02_attributes);
#else
  myTask02Handle = osThreadNew(SMARM_Experiment_Task, NULL, &myTask02_attributes);
#endif
  if (myTask02Handle == NULL) {
    Error_Handler();
  }
  {
#if NS_APP_MODE_E4_ATOMIC
    const char ok[] = "FreeRTOS tasks created (NS_HashBenchmark, no NormalTask). Starting scheduler...\r\n";
#else
#if USE_SAU_APPROACH
    const char ok[] = "FreeRTOS tasks created (Guard + NormalTask 1000 Hz). Starting scheduler...\r\n";
#elif USE_SNAP
    const char ok[] = "FreeRTOS tasks created (Snap + NormalTask 1000 Hz). Starting scheduler...\r\n";
#else
    const char ok[] = "FreeRTOS tasks created (Baseline + NormalTask 1000 Hz). Starting scheduler...\r\n";
#endif
#endif
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#endif /* !PULSE_SMOKE_TEST */
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

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
}

#if PULSE_SMOKE_TEST
#if PULSE_SMOKE_USE_PWM_LOOP
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) {
    return;
  }
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  {
    GPIO_InitTypeDef gpio = {0};
    /* PA6 = D12 = TIM3_CH1 PWM out */
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);
  }
}
#endif

#if PULSE_SMOKE_USE_ULTRASONIC
static void us_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t us_clk_mhz(void)
{
  uint32_t hz = HAL_RCC_GetHCLKFreq();
  if (hz < 1000000U) {
    hz = SystemCoreClock;
  }
  if (hz < 1000000U) {
    hz = 110000000U; /* Nucleo-L552 typical after PLL */
  }
  return hz / 1000000U;
}

static void us_delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * us_clk_mhz();
  while ((DWT->CYCCNT - start) < ticks) {
  }
}

static void us_sig_output(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = US_SIG_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(US_SIG_GPIO_Port, &gpio);
}

static void us_sig_input(void)
{
  GPIO_InitTypeDef gpio = {0};
  /* No pull: Grove idle is low; MCU pull-up fakes echo and causes err=-2 */
  gpio.Pin = US_SIG_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_SIG_GPIO_Port, &gpio);
}

/* Grove single-wire: >10 us trigger, then echo high time → cm (us/58). */
static int32_t us_read_distance_cm(void)
{
  uint32_t t0;
  uint32_t mhz = us_clk_mhz();
  uint32_t timeout_ticks = US_ECHO_TIMEOUT_US * mhz;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  us_sig_output();
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_RESET);
  us_delay_us(2);
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_SET);
  us_delay_us(20); /* Seeed: >10 us TTL trigger */
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_RESET);
  us_sig_input();

  /* Wait until line is low (release / pre-echo), then rising edge = echo */
  t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_SET) {
    if ((DWT->CYCCNT - t0) > timeout_ticks) {
      if (!primask) {
        __enable_irq();
      }
      return -4; /* stuck high before echo */
    }
  }

  t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_RESET) {
    if ((DWT->CYCCNT - t0) > timeout_ticks) {
      if (!primask) {
        __enable_irq();
      }
      return -1; /* no echo start — often wiring / no  power */
    }
  }

  t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_SET) {
    if ((DWT->CYCCNT - t0) > timeout_ticks) {
      if (!primask) {
        __enable_irq();
      }
      return -2; /* echo stuck high */
    }
  }

  {
    uint32_t echo_us = (DWT->CYCCNT - t0) / mhz;
    if (!primask) {
      __enable_irq();
    }
    if (echo_us < 100U) {
      return -3; /* too short */
    }
    return (int32_t)(echo_us / 58U);
  }
}
#endif /* PULSE_SMOKE_USE_ULTRASONIC */

static void PulseSmoke_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if PULSE_SMOKE_USE_ULTRASONIC
  us_dwt_init();
  us_sig_output();
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_RESET);
#endif

#if PULSE_SMOKE_USE_WHEEL
  {
    GPIO_InitTypeDef gpio = {0};
    /* PA6 = D12: optical encoder OUT (Robotlinking U-slot + slotted disk) */
    gpio.Pin = WHEEL_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP; /* many modules are open-collector / idle-high */
    HAL_GPIO_Init(WHEEL_GPIO_Port, &gpio);
    HAL_NVIC_SetPriority(EXTI6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI6_IRQn);
  }
#endif

#if PULSE_SMOKE_USE_PWM_LOOP || PULSE_SMOKE_USE_HW_SENSOR
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_IT_RISING;
#if PULSE_SMOKE_USE_HW_SENSOR
    gpio.Pull = GPIO_PULLUP;
#else
    gpio.Pull = GPIO_PULLDOWN;
#endif
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(EXTI7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI7_IRQn);
  }
#endif

#if PULSE_SMOKE_USE_PWM_LOOP
  {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 109;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
      Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
  }
#endif
}

#if PULSE_SMOKE_USE_PWM_LOOP || PULSE_SMOKE_USE_HW_SENSOR || PULSE_SMOKE_USE_WHEEL
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
#if PULSE_SMOKE_USE_WHEEL
  if (GPIO_Pin == WHEEL_Pin) {
    g_wheel_edges++;
  }
#endif
#if PULSE_SMOKE_USE_PWM_LOOP || PULSE_SMOKE_USE_HW_SENSOR
  if (GPIO_Pin == GPIO_PIN_7) {
    g_pulse_edges++;
  }
#endif
}
#endif

void PulseSmoke_Task(void *argument)
{
  (void)argument;
  char line[128];
#if PULSE_SMOKE_USE_ULTRASONIC
  uint32_t n_ok = 0U;
  uint32_t n_fail = 0U;
#if !PULSE_SMOKE_USE_WHEEL
  {
    const char help[] =
        "WIRE TEST 8s: yellow in D11 (CN7). Touch free end to red metal -> SIG=1.\r\n"
        "White NC unused. Then yellow back to sensor SIG. Red=3V3 Black=GND.\r\n";
    ns_uart_transmit((const uint8_t *)help, (uint16_t)(sizeof(help) - 1U));
  }
  us_sig_input();
  for (uint32_t i = 0U; i < 40U; i++) {
    int sig = (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_SET) ? 1 : 0;
    int n = snprintf(line, sizeof(line),
                     "[wire] t=%lu.%lu s  SIG=%d  HCLK=%luMHz\r\n",
                     (unsigned long)(i / 5U),
                     (unsigned long)((i % 5U) * 2U),
                     sig,
                     (unsigned long)us_clk_mhz());
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    osDelay(200);
  }
  {
    const char go[] = "WIRE TEST done. Ranging...\r\n";
    ns_uart_transmit((const uint8_t *)go, (uint16_t)(sizeof(go) - 1U));
  }
#else
  {
    const char go[] =
        "SonarTask: ultrasonic on D11. WheelTask uses D12 (spin slotted disk).\r\n";
    ns_uart_transmit((const uint8_t *)go, (uint16_t)(sizeof(go) - 1U));
  }
#endif
  for (;;) {
    int32_t cm = us_read_distance_cm();
    if (cm >= 0) {
      n_ok++;
      int n = snprintf(line, sizeof(line),
                       "[sonar] distance_cm=%ld  ok=%lu  fail=%lu\r\n",
                       (long)cm, (unsigned long)n_ok, (unsigned long)n_fail);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    } else {
      n_fail++;
      int n = snprintf(line, sizeof(line),
                       "[sonar] FAIL err=%ld  ok=%lu  fail=%lu\r\n",
                       (long)cm, (unsigned long)n_ok, (unsigned long)n_fail);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    }
    osDelay(200);
  }
#else
  uint32_t sec = 0U;
  uint32_t last_edges = g_pulse_edges;
  uint32_t win_sum = 0U;
  uint32_t win_n = 0U;
  uint32_t win_min = UINT32_MAX;
  uint32_t win_max = 0U;
  uint32_t tick = osKernelGetTickCount();
  {
#if PULSE_SMOKE_USE_HW_SENSOR
    const char ready[] =
        "HW sensor ready. Idle edges/s≈0; spin disk → edges/s rises.\r\n";
#else
    const char ready[] =
        "PULSE_SMOKE ready (NS-only). Expect ~10 pulses / 10 ms, ~1000/s.\r\n";
#endif
    ns_uart_transmit((const uint8_t *)ready, (uint16_t)(sizeof(ready) - 1U));
  }
  for (;;) {
    tick += 10U;
    osDelayUntil(tick);

    uint32_t now = g_pulse_edges;
    uint32_t d = now - last_edges;
    last_edges = now;
    win_sum += d;
    win_n++;
    if (d < win_min) {
      win_min = d;
    }
    if (d > win_max) {
      win_max = d;
    }

    if (win_n < 100U) {
      continue;
    }

    uint32_t edges_per_s = win_sum;
    uint32_t mean_per_10ms = win_sum / win_n;
    uint32_t mm_per_s = (edges_per_s * WHEEL_CIRCUM_MM) / PULSE_TEETH_PER_REV;
    sec++;
    int n = snprintf(line, sizeof(line),
                     "[smoke] t=%lus  /10ms mean=%lu min=%lu max=%lu  "
                     "edges/s=%lu  v≈%lu mm/s  total=%lu\r\n",
                     (unsigned long)sec,
                     (unsigned long)mean_per_10ms,
                     (unsigned long)win_min,
                     (unsigned long)win_max,
                     (unsigned long)edges_per_s,
                     (unsigned long)mm_per_s,
                     (unsigned long)now);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    win_sum = 0U;
    win_n = 0U;
    win_min = UINT32_MAX;
    win_max = 0U;
  }
#endif /* !ULTRASONIC */
}

#if PULSE_SMOKE_USE_WHEEL
void WheelSmoke_Task(void *argument)
{
  (void)argument;
  char line[128];
  uint32_t sec = 0U;
  uint32_t last = g_wheel_edges;
  {
    const char ready[] =
        "WheelTask: slotted disk in U-slot, OUT->D12. Spin by hand → edges/s > 0.\r\n";
    ns_uart_transmit((const uint8_t *)ready, (uint16_t)(sizeof(ready) - 1U));
  }
  for (;;) {
    uint32_t before = last;
    osDelay(1000);
    uint32_t now = g_wheel_edges;
    uint32_t delta = now - before;
    last = now;
    sec++;
    /* edges/s / 20 slots ≈ rev/s */
    uint32_t rpm = (delta * 60U) / PULSE_TEETH_PER_REV;
    int n = snprintf(line, sizeof(line),
                     "[wheel] t=%lus  edges/s=%lu  rpm≈%lu  total=%lu\r\n",
                     (unsigned long)sec,
                     (unsigned long)delta,
                     (unsigned long)rpm,
                     (unsigned long)now);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
  }
}
#endif /* WHEEL */
#endif /* PULSE_SMOKE_TEST */

#include <sys/time.h>
int _gettimeofday(struct timeval *tv, void *tzvp) { (void)tv; (void)tzvp; return 0; }
void SecureFault_Callback(void)
{
  const char msg[] = "\r\n[NS] SecureFault — flash Secure+NonSecure, rebuild both.\r\n";
  (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), HAL_MAX_DELAY);
  Error_Handler();
}
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  ns_uart_transmit(&c, 1U);
  return ch;
}
void SecureError_Callback(void) { Error_Handler(); }

#define TIM2_TICKS_PER_SEC  137500

static void ns_print_attest_phase(uint32_t phase)
{
  char buf[72];
  int n;
  if (phase == 0U) {
    return;
  }
  if (phase == 9000U) {
//    n = snprintf(buf, sizeof(buf), "[NS] attest done (phase 9000)\r\n");
  } else if (phase >= 100U && phase < 200U) {
//    n = snprintf(buf, sizeof(buf), "[NS] attest block %lu\r\n",
//                 (unsigned long)(phase - 100U));
  } else if (phase == 1U) {
//    n = snprintf(buf, sizeof(buf), "[NS] attest enter 128KiB\r\n");
  } else if (phase == 2U) {
//    n = snprintf(buf, sizeof(buf), "[NS] attest shuffle done\r\n");
  } else if (phase == 3U) {
//    n = snprintf(buf, sizeof(buf), "[NS] attest SAU+HMAC...\r\n");
  } else {
//    n = snprintf(buf, sizeof(buf), "[NS] attest phase=%lu\r\n", (unsigned long)phase);
  }
  if (n > 0) {
    ns_uart_transmit((const uint8_t *)buf, (uint16_t)n);
  }
}

void NormalTask(void *argument)
{
    (void) argument;
    const uint32_t period_os_ticks = 1000 / TARGET_FREQ_HZ;
    uint32_t next_wake_time = osKernelGetTickCount();
    uint32_t loop_counter = 0;
    uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t last_attest_phase = 0U;
    for (;;)
    {
        next_wake_time += period_os_ticks;
        osDelayUntil(next_wake_time);
        g_normal_counter++;
        loop_counter++;
        if (g_secure_busy != 0U) {
            uint32_t ph = g_attest_phase;
            if (ph != last_attest_phase) {
                last_attest_phase = ph;
                ns_print_attest_phase(ph);
            }
            static uint32_t hb;
            hb++;
            if ((hb % (TARGET_FREQ_HZ / 2U)) == 0U) {
                const char dot[] = ".";
                ns_uart_transmit((const uint8_t *)dot, 1U);
            }
        }
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

/*
 * Memory map (must match secure_nsc.c):
 *   0x08050000..0x0805FFFF  guard gap (not attested)
 *   0x08060000..0x0807FFFF  data window 128 KiB (SAU R6/R7)
 */
#define NS_APP_FLASH_BASE          0x08040000U
#define NS_APP_FLASH_BYTES         (64U * 1024U)
#define NS_ATTEST_DATA_BASE       0x08060000U
#define NS_ATTEST_DATA_BYTES       0x20000U   /* 128 KiB */

/* Optional NS-side HMAC mirror timing; must match attested data window (128 KiB). */
#define NS_HMAC_SHA256_DIGEST_SIZE 32
#ifndef NS_HMAC_BLOCK_SIZE
#define NS_HMAC_BLOCK_SIZE         1024
#endif
#define NS_HMAC_TOTAL_SIZE         NS_ATTEST_DATA_BYTES
#define NS_HMAC_BLOCKS             (NS_HMAC_TOTAL_SIZE / NS_HMAC_BLOCK_SIZE)
#if (NS_HMAC_TOTAL_SIZE % NS_HMAC_BLOCK_SIZE) != 0
#error NS_HMAC_TOTAL_SIZE must be divisible by NS_HMAC_BLOCK_SIZE
#endif
#if NS_APP_MODE_E4_ATOMIC
#define NS_HMAC_TIMING_ITERATIONS  NS_E4_SAMPLES_PER_RUN
#else
/** How many hmac_sha256_update calls to time per benchmark run (digest differs from full scan). */
#define NS_HMAC_TIMING_ITERATIONS  5
#endif

#if NS_HMAC_TOTAL_SIZE > NS_ATTEST_DATA_BYTES
#error NS_HMAC_TOTAL_SIZE larger than attestation data window
#endif

static uint8_t *const ns_hmac_real_memory = (uint8_t *)NS_ATTEST_DATA_BASE;
static const uint8_t ns_hmac_key[] = "MySecureKey123";
static hmac_sha256 ns_hmac_state;
static int ns_hmac_indices[NS_HMAC_BLOCKS];
/** Scratch for copy-then-hash: IRQ disabled only during memcpy; HMAC runs on RAM. */
static uint8_t ns_hmac_copy_buf[NS_HMAC_BLOCK_SIZE];

typedef struct {
  struct AES_ctx ctx;
  uint8_t buf[16];
  int idx;
} ns_ctr_prng_t;

static void ns_derive_aes_key_iv_from_challenge(uint8_t key16[16], uint8_t iv16[16],
                                                const uint8_t *challenge, size_t clen)
{
  hmac_sha256_initialize(&ns_hmac_state, ns_hmac_key, strlen((const char *)ns_hmac_key));
      if (challenge && clen) {
    hmac_sha256_update(&ns_hmac_state, challenge, clen);
      } else {
          uint32_t tick = (uint32_t)SysTick->VAL;
    hmac_sha256_update(&ns_hmac_state, (uint8_t *)&tick, sizeof(tick));
  }
  hmac_sha256_finalize(&ns_hmac_state, NULL, 0);
  memcpy(key16, ns_hmac_state.digest, 16);
  memcpy(iv16, ns_hmac_state.digest + 16, 16);
}

static void ns_prng_init(ns_ctr_prng_t *p, const uint8_t key16[16], const uint8_t iv16[16])
  {
      AES_init_ctx_iv(&p->ctx, key16, iv16);
      memset(p->buf, 0, sizeof(p->buf));
  p->idx = 16;
  }
  
static void ns_prng_refill_block(ns_ctr_prng_t *p)
  {
      uint8_t zero[16] = {0};
      memcpy(p->buf, zero, 16);
  AES_CTR_xcrypt_buffer(&p->ctx, p->buf, 16);
      p->idx = 0;
  }
  
static uint32_t ns_prng_next_u32(ns_ctr_prng_t *p)
  {
  if (p->idx > 12) {
    ns_prng_refill_block(p);
      }
      uint32_t v;
      memcpy(&v, &p->buf[p->idx], 4);
      p->idx += 4;
      return v;
  }
  
static int ns_prng_uniform_u32(ns_ctr_prng_t *p, int n)
  {
      const uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
      for (;;) {
    uint32_t r = ns_prng_next_u32(p);
    if (r < lim) {
      return (int)(r % (uint32_t)n);
    }
  }
}

static void ns_shuffle_aes_ctr(int *arr, int n, const uint8_t key16[16], const uint8_t iv16[16])
{
  ns_ctr_prng_t prng;
  ns_prng_init(&prng, key16, iv16);
  for (int i = n - 1; i > 0; i--) {
    int j = ns_prng_uniform_u32(&prng, i + 1);
    int tmp = arr[i];
    arr[i] = arr[j];
    arr[j] = tmp;
  }
}

/** Format duration as decimal milliseconds with 6 fractional digits (no float, no %llu — nano libc friendly). */
static void ns_fmt_ticks_ms6(char out[32], uint32_t ticks)
{
  uint64_t dur_ns = ((uint64_t)ticks * 1000000000ULL) / (uint64_t)TIM2_TICKS_PER_SEC;
  uint32_t ms_int = (uint32_t)(dur_ns / 1000000ULL);
  uint32_t frac6 = (uint32_t)(dur_ns % 1000000ULL);
  (void)snprintf(out, 32, "%lu.%06lu", (unsigned long)ms_int, (unsigned long)frac6);
}

/** Summarize min/max/mean over @p timed_updates samples (not necessarily NS_HMAC_BLOCKS). */
static void ns_uart_hmac_timing_summary(unsigned round, unsigned timed_updates, uint32_t min_ticks, uint32_t max_ticks,
                                        uint64_t sum_ticks, uint32_t hash_loop_ticks)
{
  const unsigned tu = timed_updates ? timed_updates : 1U;
  uint32_t mean_ticks = (uint32_t)(sum_ticks / (uint64_t)tu);
  char ms_min[32], ms_max[32], ms_mean[32], ms_span[32];
  ns_fmt_ticks_ms6(ms_min, min_ticks);
  ns_fmt_ticks_ms6(ms_max, max_ticks);
  ns_fmt_ticks_ms6(ms_mean, mean_ticks);
  ns_fmt_ticks_ms6(ms_span, hash_loop_ticks);
  char buf[288];
  int n = snprintf(buf, sizeof(buf),
                   "\r\n=== NS HMAC timing (UART) round %u ===\r\n"
                   "BLOCK_SIZE=%u TOTAL=0x%X logical_BLOCKS=%u | timed_updates=%u\r\n"
                   "Per IRQ-masked memcpy: min=%lu ticks (%s ms), max=%lu ticks (%s ms), "
                   "mean=%lu ticks (%s ms)\r\n"
                   "Sum ticks=%lu; hash-loop TIM2 span=%lu ticks (%s ms)\r\n",
                   (unsigned)round, (unsigned)NS_HMAC_BLOCK_SIZE, (unsigned)NS_HMAC_TOTAL_SIZE,
                   (unsigned)NS_HMAC_BLOCKS, tu, (unsigned long)min_ticks, ms_min,
                   (unsigned long)max_ticks, ms_max, (unsigned long)mean_ticks, ms_mean,
                   (unsigned long)sum_ticks, (unsigned long)hash_loop_ticks, ms_span);
  if (n > 0 && n < (int)sizeof(buf)) {
    ns_uart_transmit((const uint8_t *)buf, (uint16_t)n);
  }
}

/**
 * Non-secure mirror of SECURE_ShuffledHMAC_secure crypto steps (derive → shuffle → HMAC).
 * IRQs masked only around memcpy(ns_hmac_copy_buf ← blk); TIM2 measures that copy.
 * hmac_sha256_update runs on RAM without IRQ masking.
 */
static void NS_ShuffledHMAC_benchmark(uint8_t *out_digest, size_t out_len,
                                      const uint8_t *challenge, size_t challenge_len,
                                      uint32_t *out_min_ticks, uint32_t *out_max_ticks,
                                      uint64_t *out_sum_ticks, uint32_t *out_hash_loop_ticks)
{
  if (!out_digest || out_len < NS_HMAC_SHA256_DIGEST_SIZE) {
    return;
  }

  for (int i = 0; i < NS_HMAC_BLOCKS; i++) {
    ns_hmac_indices[i] = i;
  }

  uint8_t key16[16], iv16[16];
  ns_derive_aes_key_iv_from_challenge(key16, iv16, challenge, challenge_len);
#if !NS_E4_QUIET_UART || !NS_APP_MODE_E4_ATOMIC
  {
    const char msg[] = "derive OK, shuffling indices...\r\n";
    ns_uart_transmit((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));
  }
#endif
  ns_shuffle_aes_ctr(ns_hmac_indices, NS_HMAC_BLOCKS, key16, iv16);
#if !NS_E4_QUIET_UART || !NS_APP_MODE_E4_ATOMIC
  {
    char msg[72];
    int nn = snprintf(msg, sizeof(msg),
                      "shuffle OK, IRQ-masked memcpy + HMAC from RAM (%u timed iters)...\r\n",
                      (unsigned)NS_HMAC_TIMING_ITERATIONS);
    if (nn > 0 && nn < (int)sizeof(msg)) {
      ns_uart_transmit((const uint8_t *)msg, (uint16_t)nn);
    }
  }

  {
    const char m[] = "calling hmac_sha256_initialize...\r\n";
    ns_uart_transmit((const uint8_t *)m, (uint16_t)(sizeof(m) - 1U));
  }
#endif
  hmac_sha256_initialize(&ns_hmac_state, ns_hmac_key, strlen((const char *)ns_hmac_key));
#if !NS_E4_QUIET_UART || !NS_APP_MODE_E4_ATOMIC
  {
    const char m[] = "hmac_sha256_initialize OK.\r\n";
    ns_uart_transmit((const uint8_t *)m, (uint16_t)(sizeof(m) - 1U));
  }
#endif

  uint32_t min_ticks = 0xFFFFFFFFu;
  uint32_t max_ticks = 0;
  uint64_t sum_ticks = 0;

  unsigned n_timed = (unsigned)NS_HMAC_BLOCKS;
  if (n_timed > (unsigned)NS_HMAC_TIMING_ITERATIONS) {
    n_timed = (unsigned)NS_HMAC_TIMING_ITERATIONS;
  }

  uint32_t t_hash_start = __HAL_TIM_GET_COUNTER(&htim2);

  for (unsigned i = 0; i < n_timed; i++) {
    const uint8_t *blk = &ns_hmac_real_memory[(size_t)ns_hmac_indices[i] * NS_HMAC_BLOCK_SIZE];
#if !NS_E4_QUIET_UART || !NS_APP_MODE_E4_ATOMIC
    if (i == 0) {
      char ibuf[100];
      int ni = snprintf(ibuf, sizeof(ibuf),
                        "iter[0]: idx=%d blk=%p copy=%p len=%u\r\n",
                        ns_hmac_indices[i], (void *)blk, (void *)ns_hmac_copy_buf,
                        (unsigned)NS_HMAC_BLOCK_SIZE);
      if (ni > 0 && ni < (int)sizeof(ibuf)) {
        ns_uart_transmit((const uint8_t *)ibuf, (uint16_t)ni);
      }
    }
#endif
    uint32_t t0;
    uint32_t t1;
    __disable_irq();
    t0 = __HAL_TIM_GET_COUNTER(&htim2);
    memcpy(ns_hmac_copy_buf, blk, NS_HMAC_BLOCK_SIZE);
    t1 = __HAL_TIM_GET_COUNTER(&htim2);
    __enable_irq();
    uint32_t dt = t1 - t0;
    if (dt < min_ticks) {
      min_ticks = dt;
    }
    if (dt > max_ticks) {
      max_ticks = dt;
    }
    sum_ticks += dt;
#if (!NS_APP_MODE_E4_ATOMIC) || !NS_E4_QUIET_UART
    {
      char msbuf[32];
      ns_fmt_ticks_ms6(msbuf, dt);
      char line[160];
      int nn = snprintf(line, sizeof(line),
                        "  [%u] memcpy TIM2 dt=%lu ticks (%s ms)\r\n",
                        (unsigned)i, (unsigned long)dt, msbuf);
      if (nn > 0 && nn < (int)sizeof(line)) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)nn);
      }
    }
#endif
    hmac_sha256_update(&ns_hmac_state, ns_hmac_copy_buf, (int)NS_HMAC_BLOCK_SIZE);
  }

  uint32_t t_hash_end = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t total_hash_ticks = t_hash_end - t_hash_start;

  hmac_sha256_finalize(&ns_hmac_state, NULL, 0);
  memcpy(out_digest, ns_hmac_state.digest, NS_HMAC_SHA256_DIGEST_SIZE);

  if (out_min_ticks) {
    *out_min_ticks = min_ticks;
  }
  if (out_max_ticks) {
    *out_max_ticks = max_ticks;
  }
  if (out_sum_ticks) {
    *out_sum_ticks = sum_ticks;
  }
  if (out_hash_loop_ticks) {
    *out_hash_loop_ticks = total_hash_ticks;
  }
}

void NS_HashBenchmark_Task(void *argument)
{
  (void)argument;
  uint8_t digest[NS_HMAC_SHA256_DIGEST_SIZE];
  uint8_t challenge[16];

  {
    char banner[160];
    int bn = snprintf(banner, sizeof(banner),
                      "\r\nNS_HashBenchmark_Task started (E4 Atomic Copy, B=%u, %u rounds, %u samples/run).\r\n",
                      (unsigned)NS_HMAC_BLOCK_SIZE,
#if NS_APP_MODE_E4_ATOMIC
                      (unsigned)NS_E4_ROUNDS,
                      (unsigned)NS_E4_SAMPLES_PER_RUN
#else
                      5U, (unsigned)NS_HMAC_TIMING_ITERATIONS
#endif
                      );
    if (bn > 0 && bn < (int)sizeof(banner)) {
      ns_uart_transmit((const uint8_t *)banner, (uint16_t)bn);
    }
  }

  osDelay(1000);

#if NS_APP_MODE_E4_ATOMIC
  const uint8_t n_rounds = NS_E4_ROUNDS;
  uint32_t round_mean_ticks[NS_E4_ROUNDS];
#else
  const uint8_t n_rounds = 5;
  uint32_t round_mean_ticks[5];
#endif

  for (uint8_t round = 0; round < n_rounds; round++) {
    uint32_t seed = osKernelGetTickCount();
    for (int k = 0; k < 4; k++) {
      uint32_t rnd = seed ^ (seed << 13) ^ (uint32_t)(k * 0x5DEECE66Du);
      memcpy(&challenge[(size_t)k * 4], &rnd, 4);
    }

    uint32_t min_ticks = 0xFFFFFFFFu, max_ticks = 0;
    uint64_t sum_ticks = 0;
    uint32_t hash_loop_ticks = 0;

    unsigned n_timed = (unsigned)NS_HMAC_BLOCKS;
    if (n_timed > (unsigned)NS_HMAC_TIMING_ITERATIONS) {
      n_timed = (unsigned)NS_HMAC_TIMING_ITERATIONS;
    }

#if !NS_APP_MODE_E4_ATOMIC
    {
      const char runmsg[] =
          "Running NS benchmark (TIM2 = IRQ-masked memcpy only; HMAC from RAM, IRQ on).\r\n";
      ns_uart_transmit((const uint8_t *)runmsg, (uint16_t)(sizeof(runmsg) - 1U));
    }
#endif

    NS_ShuffledHMAC_benchmark(digest, sizeof(digest), challenge, sizeof(challenge),
                              &min_ticks, &max_ticks, &sum_ticks, &hash_loop_ticks);

    uint32_t mean_ticks = (n_timed > 0U) ? (uint32_t)(sum_ticks / (uint64_t)n_timed) : 0U;
    round_mean_ticks[round] = mean_ticks;

    ns_uart_hmac_timing_summary((unsigned)(round + 1), n_timed,
                                min_ticks, max_ticks, sum_ticks, hash_loop_ticks);

    {
      char ms_mean[32];
      ns_fmt_ticks_ms6(ms_mean, mean_ticks);
      char line[128];
      int nn = snprintf(line, sizeof(line),
                        "Round %u: memcpy mean=%lu ticks (%s ms) over %u samples\r\n",
                        (unsigned)(round + 1), (unsigned long)mean_ticks, ms_mean,
                        n_timed);
      if (nn > 0 && nn < (int)sizeof(line)) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)nn);
      }
    }
    osDelay(500);
  }

#if NS_APP_MODE_E4_ATOMIC
  {
    uint64_t sum_mean = 0;
    uint32_t gmin = round_mean_ticks[0], gmax = round_mean_ticks[0];
    for (uint8_t r = 0; r < n_rounds; r++) {
      sum_mean += round_mean_ticks[r];
      if (round_mean_ticks[r] < gmin) {
        gmin = round_mean_ticks[r];
      }
      if (round_mean_ticks[r] > gmax) {
        gmax = round_mean_ticks[r];
      }
    }
    uint32_t overall_mean = (uint32_t)(sum_mean / (uint64_t)n_rounds);
    uint64_t var = 0;
    for (uint8_t r = 0; r < n_rounds; r++) {
      int64_t d = (int64_t)round_mean_ticks[r] - (int64_t)overall_mean;
      var += (uint64_t)(d * d);
    }
    uint64_t var_mean = var / (uint64_t)n_rounds;
    uint32_t std_ticks = 0;
    for (uint32_t s = 0xFFFFFFFFU / 2U; s > 0; s /= 2U) {
      uint32_t next = std_ticks + s;
      if ((uint64_t)next * (uint64_t)next <= var_mean) {
        std_ticks = next;
      }
    }
    char ms_mean[32], ms_std[32];
    ns_fmt_ticks_ms6(ms_mean, overall_mean);
    ns_fmt_ticks_ms6(ms_std, std_ticks);
    char sumbuf[288];
    int sn = snprintf(sumbuf, sizeof(sumbuf),
                      "\r\n=== E4 Atomic Copy SUMMARY (B=%u, %u rounds) ===\r\n"
                      "Per-update memcpy (IRQ-off): mean=%lu ticks (%s ms), "
                      "std~=%lu ticks (%s ms), min=%lu max=%lu ticks\r\n"
                      "Archive in thesis/e4_atomic_raw_data.md\r\n"
                      "===============================================\r\n",
                      (unsigned)NS_HMAC_BLOCK_SIZE, (unsigned)n_rounds,
                      (unsigned long)overall_mean, ms_mean,
                      (unsigned long)std_ticks, ms_std,
                      (unsigned long)gmin, (unsigned long)gmax);
    if (sn > 0 && sn < (int)sizeof(sumbuf)) {
      ns_uart_transmit((const uint8_t *)sumbuf, (uint16_t)sn);
    }
  }
#endif

  {
    const char done[] = "\r\nNS HMAC benchmark done. Press RESET for next B (after rebuild).\r\n";
    ns_uart_transmit((const uint8_t *)done, (uint16_t)(sizeof(done) - 1U));
  }

  for (;;) {
    osDelay(10000);
  }
}

void SMARM_Experiment_Task(void *argument)
{
    (void) argument;
    {
        const char alive[] = "SMARM_Experiment_Task running (FAR + NormalTask 1000 Hz).\r\n";
        ns_uart_transmit((const uint8_t *)alive, (uint16_t)(sizeof(alive) - 1U));
    }
    {
        char banner[220];
        int bn = snprintf(banner, sizeof(banner),
#if USE_SAU_APPROACH
#if ATTEST_FAULT_DEMO
            "\r\nSMARM-Guard FAULT DEMO B=%u |M|=256KiB @0x08040000 "
            "(SAU over live NS XIP; expect fault).\r\n",
#else
            "\r\nSMARM-Guard B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @1000 Hz (SAU every block) (10 rounds).\r\n",
#endif
#elif USE_SNAP
            "\r\nSMARM-Snap B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @1000 Hz (memcpy IRQ-off) (10 rounds).\r\n",
#else
            "\r\nSMARM Baseline B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @1000 Hz (HMAC IRQ-off) (10 rounds).\r\n",
#endif
            (unsigned)ATTEST_BLOCK_SIZE);
        if (bn > 0 && bn < (int)sizeof(banner)) {
            ns_uart_transmit((const uint8_t *)banner, (uint16_t)bn);
        }
    }
    {
        const char msg[] = "Allocating secure context (8 KiB)...\r\n";
        ns_uart_transmit((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));
    }
    portALLOCATE_SECURE_CONTEXT(8192);
    {
        const char msg[] = "Secure context OK. Waiting 1 s then starting rounds...\r\n";
        ns_uart_transmit((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));
    }
    static uint32_t durations_ms[10];
    uint64_t far_sum_x10000 = 0U;
    uint32_t far_min_x10000 = UINT32_MAX;
    uint32_t far_max_x10000 = 0U;
    uint32_t far_valid_rounds = 0U;
    uint8_t digest[32];
    uint8_t challenge[16];
    osDelay(1000);
    for(uint8_t round = 0; round < 10; round++)
    {
        uint32_t seed = osKernelGetTickCount();
        for(int k=0; k<4; k++) {
            uint32_t rnd = seed ^ (seed << 13) ^ (k * 0x5DEECE66D);
            memcpy(&challenge[k*4], &rnd, 4);
        }
        if (round == 0U) {
            SECURE_LEDToggle();
            const char ping[] = "  NS: SECURE_LEDToggle OK (SG path alive)\r\n";
            ns_uart_transmit((const uint8_t *)ping, (uint16_t)(sizeof(ping) - 1U));
            const char msg[] =
                "Calling SECURE_ShuffledHMAC_secure (round 1)...\r\n";
            ns_uart_transmit((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));
        }
        g_attest_phase = 0U;
        g_secure_busy = 1U;
        uint32_t start_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t start_count = g_normal_counter;
        SECURE_AttestReport_t attest_report = {
            .sau_config_avg_cycles = 0U,
            .t_disabled_avg_cycles = 0U,
            .t_disabled_max_cycles = 0U,
            .attest_phase_ptr = &g_attest_phase,
        };
        SECURE_ShuffledHMAC_secure(digest, challenge, sizeof(challenge), &attest_report);
        uint32_t end_tim2 = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t end_count = g_normal_counter;
        uint32_t sau_avg = attest_report.sau_config_avg_cycles;
        uint32_t tdis_avg = attest_report.t_disabled_avg_cycles;
        uint32_t tdis_max = attest_report.t_disabled_max_cycles;
        g_secure_busy = 0U;
        if (round == 0U) {
            const char back[] = "  NS: returned from SECURE_ShuffledHMAC_secure\r\n";
            ns_uart_transmit((const uint8_t *)back, (uint16_t)(sizeof(back) - 1U));
        }
        if (round == 0U) {
            const char msg[] = "Returned from secure attestation (round 1).\r\n";
            ns_uart_transmit((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));
        }
        uint32_t actual_run = end_count - start_count;
        uint32_t tim2_diff = end_tim2 - start_tim2;
        uint32_t actual_duration_ms = ((uint64_t)tim2_diff * 1000) / 137500;
        durations_ms[round] = actual_duration_ms;
        uint32_t expected_run = (uint32_t)(((uint64_t)actual_duration_ms * TARGET_FREQ_HZ) / 1000U);
        uint32_t far_x10000 = 0U;
        if (expected_run > 0U) {
            far_x10000 = (uint32_t)((((uint64_t)actual_run * 10000U)
                                  + (expected_run / 2U)) / expected_run);
            far_sum_x10000 += far_x10000;
            if (far_x10000 < far_min_x10000) far_min_x10000 = far_x10000;
            if (far_x10000 > far_max_x10000) far_max_x10000 = far_x10000;
            far_valid_rounds++;
        }
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            printf("Round %u: Runtime=%lu ms, NS=%lu/%lu activations",
                   round + 1, actual_duration_ms, actual_run, expected_run);
            if (expected_run > 0U) {
            printf(", FAR=%lu.%04lu\r\n",
                   (unsigned long)(far_x10000 / 10000U),
                   (unsigned long)(far_x10000 % 10000U));
            } else {
                printf(", FAR=N/A\r\n");
            }
            if (sau_avg == 0xBAD00002U) {
                printf("SAU_config_avg_cycles=CMSE_FAIL(challenge ptr)\r\n");
            } else if (sau_avg == 0xBAD00001U) {
                printf("SAU_config_avg_cycles=CMSE_FAIL(digest ptr)\r\n");
            } else {
#if USE_SAU_APPROACH
                printf("SAU_config_avg_cycles=%lu\r\n", (unsigned long)sau_avg);
#endif
                /* CPU = 110 MHz → time_us = cycles/110. Print µs (3 dp) or ms if ≥1 ms. */
                {
                    uint32_t avg_us_x1000 =
                        (uint32_t)(((uint64_t)tdis_avg * 1000U) / 110U);
                    uint32_t max_us_x1000 =
                        (uint32_t)(((uint64_t)tdis_max * 1000U) / 110U);
                    if (avg_us_x1000 >= 1000000U || max_us_x1000 >= 1000000U) {
                        printf("t_disabled avg=%lu cy (%lu.%03lu ms), max=%lu cy (%lu.%03lu ms)\r\n",
                               (unsigned long)tdis_avg,
                               (unsigned long)(avg_us_x1000 / 1000000U),
                               (unsigned long)((avg_us_x1000 / 1000U) % 1000U),
                               (unsigned long)tdis_max,
                               (unsigned long)(max_us_x1000 / 1000000U),
                               (unsigned long)((max_us_x1000 / 1000U) % 1000U));
                    } else {
                        printf("t_disabled avg=%lu cy (%lu.%03lu us), max=%lu cy (%lu.%03lu us)\r\n",
                               (unsigned long)tdis_avg,
                               (unsigned long)(avg_us_x1000 / 1000U),
                               (unsigned long)(avg_us_x1000 % 1000U),
                               (unsigned long)tdis_max,
                               (unsigned long)(max_us_x1000 / 1000U),
                               (unsigned long)(max_us_x1000 % 1000U));
                    }
                }
            }
            xSemaphoreGive(uart_mutex);
        }
        osDelay(2000);
    }
    uint32_t sum = 0, min_val = durations_ms[0], max_val = durations_ms[0];
    for (int i = 0; i < 10; i++) { sum += durations_ms[i]; if (durations_ms[i] < min_val) min_val = durations_ms[i]; if (durations_ms[i] > max_val) max_val = durations_ms[i]; }
    uint32_t mean = sum / 10;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
#if USE_SAU_APPROACH
        printf("\r\n=== SUMMARY SMARM-Guard FAR @1000 Hz (10 rounds) ===\r\n");
#elif USE_SNAP
        printf("\r\n=== SUMMARY SMARM-Snap FAR @1000 Hz (10 rounds) ===\r\n");
#else
        printf("\r\n=== SUMMARY SMARM Baseline FAR @1000 Hz (10 rounds) ===\r\n");
#endif
        printf("Mean: %lu ms\r\n", mean);
        printf("Min:  %lu ms\r\n", min_val);
        printf("Max:  %lu ms\r\n", max_val);
        if (far_valid_rounds > 0U) {
            uint32_t far_mean_x10000 =
                (uint32_t)(far_sum_x10000 / far_valid_rounds);
            printf("FAR mean=%lu.%04lu, min=%lu.%04lu, max=%lu.%04lu\r\n",
                   (unsigned long)(far_mean_x10000 / 10000U),
                   (unsigned long)(far_mean_x10000 % 10000U),
                   (unsigned long)(far_min_x10000 / 10000U),
                   (unsigned long)(far_min_x10000 % 10000U),
                   (unsigned long)(far_max_x10000 / 10000U),
                   (unsigned long)(far_max_x10000 % 10000U));
        }
        printf("t_disabled: DWT over each IRQ-off window (110 MHz).\r\n");
        printf("==========================================\r\n");
        printf("Done. Press RESET to run again.\r\n");
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
