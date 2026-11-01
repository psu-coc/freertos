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
#include "task.h"
#include "semphr.h"
#include "stm32l5xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "hmac-sha256.h"
#include "aes.h"
#include "secure_nsc.h"
#include "rt_experiment_profile.h"
#include "rt_dmr_formal.h"

#if RT_DMR_FORMAL && !RT_WALL_OBS_ENABLE
#error RT_DMR_FORMAL requires RT_RUN_LEN=RT_RUN_WALL_OBS
#endif

/*
 * Board smoke test (branch smarmplus-rt-smoke), NonSecure-only path:
 *   No Secure attestation / NSC calls — Secure image only boots TrustZone.
 *   MODE 0: jumper D12<->D11, TIM3 1 kHz PWM + EXTI7.
 *   MODE 1: HW speed OUT->D11 + EXTI7.
 *   MODE 2: ultrasonic SIG->D11 only.
 *   MODE 3: ultrasonic D11 + optical wheel OUT->D12 (two FreeRTOS tasks).
 *   EXTI6/7 must be Non-Secure in partition ITNS0 (rebuild Secure for MODE 3).
 * Set PULSE_SMOKE_TEST 0 to restore FAR / Snap-Guard harness tasks.
 * TABLE_I_RUNTIME=1: Access Table I (512 KiB, no NormalTask); forces pulse smoke off.
 */
#ifndef TABLE_I_RUNTIME
#define TABLE_I_RUNTIME  0
#endif
#if TABLE_I_RUNTIME
#define PULSE_SMOKE_TEST  0
#else
#define PULSE_SMOKE_TEST  1
#endif
#define PULSE_SMOKE_MODE  3   /* 0=PWM, 1=HW speed, 2=sonar, 3=sonar+wheel */
/*
 * Phase 2 — no-attestation RT baseline (buffer jobs, dump CSV after run).
 * Set 0 to restore continuous UART smoke print + osDelay (Phase 0 behaviour).
 */
#define PULSE_SMOKE_PHASE2  1

#define PULSE_SMOKE_USE_HW_SENSOR   (PULSE_SMOKE_MODE == 1)
#define PULSE_SMOKE_USE_ULTRASONIC  ((PULSE_SMOKE_MODE == 2) || (PULSE_SMOKE_MODE == 3))
#define PULSE_SMOKE_USE_WHEEL       (PULSE_SMOKE_MODE == 3)
#define PULSE_SMOKE_USE_PWM_LOOP    (PULSE_SMOKE_MODE == 0)

/* Optical disk has 20 slots — UART "speed" display */
#define PULSE_TEETH_PER_REV  20U
#define WHEEL_CIRCUM_MM      1900U

#if PULSE_SMOKE_PHASE2 && PULSE_SMOKE_TEST && (PULSE_SMOKE_MODE == 3)
/* Periods/jobs/deadlines come from rt_experiment_profile.h (selectable). */
#define PHASE2_SONAR_PERIOD_MS           RT_SONAR_PERIOD_MS
#define PHASE2_WHEEL_PERIOD_MS           RT_WHEEL_PERIOD_MS
#define PHASE2_SONAR_DEADLINE_MS         RT_SONAR_DEADLINE_MS
#define PHASE2_WHEEL_COMPUTE_DEADLINE_US RT_WHEEL_COMPUTE_DEADLINE_US
#define PHASE2_WALL_TARGET_US            RT_WALL_TARGET_US
#define PHASE2_SONAR_JOBS                RT_SONAR_JOBS
#define PHASE2_WHEEL_JOBS                RT_WHEEL_JOBS
#define PHASE2_ENABLE 1
/*
 * 0 = no-attestation baseline (this validation).
 * 1 = Original SMARM concurrent (same sensor profile) — enable after baseline OK.
 * Does not change Secure attest implementation; only creates AttestTask.
 */
#ifndef PHASE3_WITH_ORIGINAL_SMARM
#define PHASE3_WITH_ORIGINAL_SMARM  0
#endif
#else
#define PHASE2_ENABLE 0
#undef PHASE3_WITH_ORIGINAL_SMARM
#define PHASE3_WITH_ORIGINAL_SMARM  0
#endif

/*
 * Grove echo path:
 *   0 = GPIO poll with IRQs enabled (Phase2 default — SysTick/EXTI6 keep running)
 *   1 = legacy __disable_irq() for entire echo wait (regression / A-B vs old smoke)
 * TIM3_CH2 on PA7 (AF2) is feasible for input-capture later; TIM2 stays free-running
 * µs reference and is not used for IC. MODE 0 uses TIM3 PWM so IC is MODE 2/3 only.
 */
#ifndef US_IRQ_MASK_DURING_ECHO
#if PHASE2_ENABLE
#define US_IRQ_MASK_DURING_ECHO  0
#else
#define US_IRQ_MASK_DURING_ECHO  0
#endif
#endif

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
#ifndef TARGET_FREQ_HZ
#define TARGET_FREQ_HZ             1000   /* Dummy-task FAR; build may set 10/100/1000 */
#endif
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
  /* USART3 (PD8/PD9) = ST-Link VCP first. LPUART1 (PG7) is optional debug;
   * never block forever on it or the VCP log never appears. */
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, NS_UART_VCP_TIMEOUT_MS);
  (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, len, NS_UART_VCP_TIMEOUT_MS);
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
  .name = "SonarTask",
  .priority = (osPriority_t) osPriorityNormal,
#if PULSE_SMOKE_TEST
  /* Keep Phase-0 sizes: larger stacks can exhaust TZ secure-context pool on L5 */
  .stack_size = 4 * 1024
#else
  /* sha256_update uses ~300B locals; shuffle/AES adds more — keep generous margin */
  .stack_size = 16 * 1024
#endif
};
#if PULSE_SMOKE_TEST && PULSE_SMOKE_USE_WHEEL
osThreadId_t wheelTaskHandle;
const osThreadAttr_t wheelTask_attributes = {
  .name = "WheelTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 2048
};
#endif
#if PHASE3_WITH_ORIGINAL_SMARM
osThreadId_t attestTaskHandle;
const osThreadAttr_t attestTask_attributes = {
  .name = "AttestTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 8 * 1024
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
#if PHASE3_WITH_ORIGINAL_SMARM
void Phase3_Attest_Task(void *argument);
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

#if RT_DMR_FORMAL
/* Retained across NRST so two hard resets of one image cannot look like one capture. */
static struct {
  uint32_t magic;
  uint32_t count;
} g_dmr_boot_store __attribute__((section(".noinit"), used));
static volatile uint32_t g_dmr_boot_count;

static void dmr_boot_account(void)
{
  if (g_dmr_boot_store.magic != 0x444D5231u) {
    g_dmr_boot_store.magic = 0x444D5231u;
    g_dmr_boot_store.count = 1U;
  } else {
    g_dmr_boot_store.count++;
  }
  g_dmr_boot_count = g_dmr_boot_store.count;
}
#endif

int main(void)
{
#if RT_DMR_FORMAL
  dmr_boot_account();
#endif
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  {
#if PULSE_SMOKE_TEST
#if PULSE_SMOKE_USE_ULTRASONIC && PULSE_SMOKE_USE_WHEEL && PHASE2_ENABLE
    {
      char boot[288];
      int bn;
#if PHASE3_WITH_ORIGINAL_SMARM
#if RT_WALL_OBS_ENABLE
      bn = snprintf(
          boot, sizeof(boot),
          "\r\n[NS boot] PHASE3 Original SMARM + sensors profile=%s run=%s.\r\n"
          "TIM2 fixed 60s wall obs; Sonar/Wheel %ums; Attest Low. Quiet then OBS+CSV.\r\n"
          "Starting FreeRTOS (Sonar + Wheel + Attest)...\r\n",
          RT_PROFILE_NAME, RT_RUN_NAME,
          (unsigned)PHASE2_SONAR_PERIOD_MS);
#else
      bn = snprintf(
          boot, sizeof(boot),
          "\r\n[NS boot] PHASE3 Original SMARM + sensors profile=%s run=%s.\r\n"
          "Sonar %ums x%u Wheel %ums x%u; Attest Low. Quiet then CSV. VCP 115200.\r\n"
          "Starting FreeRTOS (Sonar + Wheel + Attest)...\r\n",
          RT_PROFILE_NAME, RT_RUN_NAME,
          (unsigned)PHASE2_SONAR_PERIOD_MS, (unsigned)PHASE2_SONAR_JOBS,
          (unsigned)PHASE2_WHEEL_PERIOD_MS, (unsigned)PHASE2_WHEEL_JOBS);
#endif
#else
#if RT_WALL_OBS_ENABLE
      bn = snprintf(
          boot, sizeof(boot),
          "\r\n[NS boot] No-attest baseline profile=%s run=%s.\r\n"
          "TIM2 fixed 60s wall obs; Sonar/Wheel %ums D=%ums; echo IRQ-ON. Quiet then OBS+CSV.\r\n"
          "Starting FreeRTOS (SonarTask + WheelTask)...\r\n",
          RT_PROFILE_NAME, RT_RUN_NAME,
          (unsigned)PHASE2_SONAR_PERIOD_MS,
          (unsigned)PHASE2_SONAR_DEADLINE_MS);
#else
      bn = snprintf(
          boot, sizeof(boot),
          "\r\n[NS boot] No-attest baseline profile=%s run=%s.\r\n"
          "Sonar %ums x%u (D=%ums) Wheel %ums x%u; echo IRQ-ON. Quiet then CSV.\r\n"
          "Starting FreeRTOS (SonarTask + WheelTask)...\r\n",
          RT_PROFILE_NAME, RT_RUN_NAME,
          (unsigned)PHASE2_SONAR_PERIOD_MS, (unsigned)PHASE2_SONAR_JOBS,
          (unsigned)PHASE2_SONAR_DEADLINE_MS,
          (unsigned)PHASE2_WHEEL_PERIOD_MS, (unsigned)PHASE2_WHEEL_JOBS);
#endif
#endif
      if (bn > 0) {
        ns_uart_transmit((const uint8_t *)boot, (uint16_t)bn);
      }
    }
#elif PULSE_SMOKE_USE_ULTRASONIC && PULSE_SMOKE_USE_WHEEL
    {
      const char boot[] =
          "\r\n[NS boot] TWO TASKS (NS-only): sonar D11 + wheel D12.\r\n"
          "Ultrasonic SIG->D11; optical OUT->D12; both VCC=3V3 GND. VCP 115200.\r\n"
          "Starting FreeRTOS (SonarTask + WheelTask)...\r\n";
      ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
    }
#elif PULSE_SMOKE_USE_ULTRASONIC
    {
      const char boot[] =
          "\r\n[NS boot] ULTRASONIC (NS-only) — SIG->D11(PA7), VCC=3V3, GND.\r\n"
          "White NC = leave unplugged. No Secure attest. VCP 115200.\r\n"
          "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
      ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
    }
#elif PULSE_SMOKE_USE_HW_SENSOR
    {
      const char boot[] =
          "\r\n[NS boot] HW SPEED SENSOR (NS-only) — OUT->D11(PA7), Vcc/GND.\r\n"
          "Remove D12 jumper. Spin wheel; edges/s should track speed. VCP 115200.\r\n"
          "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
      ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
    }
#else
    {
      const char boot[] =
          "\r\n[NS boot] PULSE SMOKE (NS-only) — wire D12(PA6) <-> D11(PA7).\r\n"
          "No Secure attest. 10 ms parse + ~1000 edges/s. VCP 115200.\r\n"
          "Starting FreeRTOS (PulseSmoke_Task)...\r\n";
      ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
    }
#endif /* PULSE_SMOKE_TEST modes */
#else /* !PULSE_SMOKE_TEST */
    {
      const char boot[] =
#if NS_APP_MODE_E4_ATOMIC
          "\r\n[NS boot] E4 Atomic Copy — NS mirror, TIM2 on IRQ-masked memcpy.\r\n"
          "|M|=128KiB @0x08060000. Change NS_HMAC_BLOCK_SIZE then rebuild NonSecure.\r\n"
#else
#if TABLE_I_RUNTIME
          "\r\n[NS boot] Table I runtime |M|=512KiB @0x08000000, 1 pass, no NormalTask.\r\n"
#elif USE_SAU_APPROACH
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
#elif TABLE_I_RUNTIME
          "Starting FreeRTOS (Table I runtime, no NormalTask)...\r\n";
#else
#if USE_SAU_APPROACH
          "Starting FreeRTOS (FAR: Guard + NormalTask 1000 Hz)...\r\n";
#elif USE_SNAP
          "Starting FreeRTOS (FAR: Snap + NormalTask 1000 Hz)...\r\n";
#else
          "Starting FreeRTOS (FAR: Baseline + NormalTask 1000 Hz)...\r\n";
#endif
#endif
      ns_uart_transmit((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1U));
    }
#endif /* !PULSE_SMOKE_TEST */
  }
#if PULSE_SMOKE_TEST
  {
    const char step[] = "Init sensors (TIM2 us-timer, no DWT)...\r\n";
    ns_uart_transmit((const uint8_t *)step, (uint16_t)(sizeof(step) - 1U));
  }
  PulseSmoke_Init();
  {
    const char step[] = "Sensors init OK. Creating FreeRTOS tasks...\r\n";
    ns_uart_transmit((const uint8_t *)step, (uint16_t)(sizeof(step) - 1U));
  }
#else
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
#endif
  osKernelInitialize();
  uart_mutex = xSemaphoreCreateMutex();
  if (uart_mutex == NULL) {
      {
        const char e[] = "FATAL: uart_mutex create failed\r\n";
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)e, (uint16_t)(sizeof(e) - 1U), 500);
      }
      Error_Handler();
  }
#if PULSE_SMOKE_TEST
#if !RT_HR_WHEEL_ONLY
  myTask02Handle = osThreadNew(PulseSmoke_Task, NULL, &myTask02_attributes);
  if (myTask02Handle == NULL) {
    {
      const char e[] = "FATAL: SonarTask create failed (heap?)\r\n";
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)e, (uint16_t)(sizeof(e) - 1U), 500);
    }
    Error_Handler();
  }
#else
  /* HR Wheel-only: Sonar not created (echo cannot sustain high rate).
   * g_phase2_sonar_done is set when WheelTask starts (see WheelSmoke_Task). */
#endif
#if PULSE_SMOKE_USE_WHEEL
  wheelTaskHandle = osThreadNew(WheelSmoke_Task, NULL, &wheelTask_attributes);
  if (wheelTaskHandle == NULL) {
    {
      const char e[] = "FATAL: WheelTask create failed (heap?)\r\n";
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)e, (uint16_t)(sizeof(e) - 1U), 500);
    }
    Error_Handler();
  }
#if PHASE3_WITH_ORIGINAL_SMARM
  attestTaskHandle = osThreadNew(Phase3_Attest_Task, NULL, &attestTask_attributes);
  if (attestTaskHandle == NULL) {
    {
      const char e[] = "FATAL: AttestTask create failed (heap?)\r\n";
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)e, (uint16_t)(sizeof(e) - 1U), 500);
    }
    Error_Handler();
  }
  {
#if RT_HR_WHEEL_ONLY
    const char ok[] =
        "WheelTask + AttestTask(Low) created (HR Wheel-only). Starting scheduler...\r\n";
#else
    const char ok[] =
        "SonarTask + WheelTask + AttestTask(Low) created. Starting scheduler...\r\n";
#endif
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#else
  {
#if RT_HR_WHEEL_ONLY
    const char ok[] = "WheelTask created (HR Wheel-only, no Sonar). Starting scheduler...\r\n";
#else
    const char ok[] = "SonarTask + WheelTask created. Starting scheduler...\r\n";
#endif
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#endif
#else
  {
    const char ok[] = "PulseSmoke_Task created. Starting scheduler...\r\n";
    ns_uart_transmit((const uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
  }
#endif
#else
  /* FAR: NormalTask @ TARGET_FREQ_HZ. Table I: no RT task (Access caption). */
#if !NS_APP_MODE_E4_ATOMIC && !TABLE_I_RUNTIME
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
#elif TABLE_I_RUNTIME
    const char ok[] = "FreeRTOS tasks created (Table I runtime, no NormalTask). Starting scheduler...\r\n";
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
/* DWT/CYCCNT is often Secure-only on TrustZone-M — do not touch from NS.
 * Use TIM2 free-running at 1 MHz (1 tick = 1 µs) instead. */
static void us_tim_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk = pclk1;
  /* When APB1 prescaler != 1, timer kernel clock is 2× PCLK1 */
  if ((RCC->CFGR & RCC_CFGR_PPRE1_Msk) != 0U) {
    timclk = pclk1 * 2U;
  }
  if (timclk < 1000000U) {
    timclk = SystemCoreClock;
  }

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (timclk / 1000000U) - 1U; /* 1 µs/tick */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFU;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
    Error_Handler();
  }
}

static uint32_t us_now(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

static void us_delay_us(uint32_t us)
{
  uint32_t start = us_now();
  while ((us_now() - start) < us) {
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

/* Grove single-wire: >10 us trigger, then echo high time → cm (us/58).
 * echo_us_out     = echo high duration (TIM2)
 * measure_span_out = wall time of whole read (TIM2)
 * When US_IRQ_MASK_DURING_ECHO=1, IRQs are masked for the whole wait (legacy).
 * When 0, SysTick and EXTI6 can preempt — preferred for multitask baseline. */
static int32_t us_read_distance_cm_ex(uint32_t *echo_us_out, uint32_t *measure_span_out)
{
  uint32_t t0;
  uint32_t t_span0;
  uint32_t timeout_us = US_ECHO_TIMEOUT_US;
  uint32_t primask = 0U;
  int32_t result = -1;

  if (echo_us_out != NULL) {
    *echo_us_out = 0U;
  }
  if (measure_span_out != NULL) {
    *measure_span_out = 0U;
  }

#if US_IRQ_MASK_DURING_ECHO
  primask = __get_PRIMASK();
  __disable_irq();
#endif
  t_span0 = us_now();

  us_sig_output();
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_RESET);
  us_delay_us(2);
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_SET);
  us_delay_us(20); /* Seeed: >10 us TTL trigger */
  HAL_GPIO_WritePin(US_SIG_GPIO_Port, US_SIG_Pin, GPIO_PIN_RESET);
  us_sig_input();

  /* Wait until line is low (release / pre-echo), then rising edge = echo */
  t0 = us_now();
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_SET) {
    if ((us_now() - t0) > timeout_us) {
      result = -4; /* stuck high before echo */
      goto us_read_done;
    }
  }

  t0 = us_now();
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_RESET) {
    if ((us_now() - t0) > timeout_us) {
      result = -1; /* no echo start — often wiring / no power */
      goto us_read_done;
    }
  }

  t0 = us_now();
  while (HAL_GPIO_ReadPin(US_SIG_GPIO_Port, US_SIG_Pin) == GPIO_PIN_SET) {
    if ((us_now() - t0) > timeout_us) {
      result = -2; /* echo stuck high */
      goto us_read_done;
    }
  }

  {
    uint32_t echo_us = us_now() - t0;
    if (echo_us_out != NULL) {
      *echo_us_out = echo_us;
    }
    if (echo_us < 100U) {
      result = -3; /* too short */
    } else {
      result = (int32_t)(echo_us / 58U);
    }
  }

us_read_done:
  if (measure_span_out != NULL) {
    *measure_span_out = us_now() - t_span0;
  }
#if US_IRQ_MASK_DURING_ECHO
  if (!primask) {
    __enable_irq();
  }
#else
  (void)primask;
#endif
  return result;
}

#if !PHASE2_ENABLE
static int32_t us_read_distance_cm(void)
{
  return us_read_distance_cm_ex(NULL, NULL);
}
#endif
#endif /* PULSE_SMOKE_USE_ULTRASONIC */

#if PHASE2_ENABLE
/*
 * Timing vocabulary (TIM2 µs). Release is NOT observed — only osDelayUntil wake + start.
 *
 *   nominal_schedule_us = TIM2 model epoch + k·P  (NOT an observed OS release time)
 *   actual_start_us     = first TIM2 sample in job body after DelayUntil returns
 *   finish_us           = output-ready / job completion
 *   schedule_start_dev  = actual_start − nominal_schedule  (may be negative: tick phase)
 *   schedule_to_finish  = finish − nominal_schedule  (NOT formal release-to-completion)
 *   exec_us             = finish − actual_start (wall span; may include preemption)
 *   interval_dev_us     = (s_k − s_{k-1}) − P
 *
 * Negative schedule_start_dev (~−0.4…−1.0 ms) = SysTick phase vs TIM2 epoch, not
 * “started before release”. Do NOT subtract a fixed offset to force zero.
 *
 * Sonar deadline: miss iff schedule_to_finish > D (D=P for active profile).
 * Wheel: miss_compute iff compute_us > D_compute; miss_output iff
 *   (finish − nominal_window_end) > D_compute. Intentionally NOT D=P on
 *   schedule_to_finish (includes intentional measurement window).
 *
 * Compact on-device records (16 B each): full CSV columns are reconstructed at
 * dump from epoch + period + stored stamps. Needed so 600+600 jobs fit in NS RAM.
 */
/* 32 B: prior TIM2 fields + DelayUntil instrumentation (schedule unchanged). */
typedef struct __attribute__((packed)) {
  uint32_t actual_start_us;    /* s_exec: post-osDelayUntil TIM2 */
  uint32_t finish_us;
  uint32_t sched_wake_tick;    /* absolute tick passed to osDelayUntil */
  uint32_t tick_before_delay;
  uint32_t tick_after_delay;
  uint16_t echo_us;
  uint16_t measure_span_us;
  int16_t  distance_cm;
  int8_t   status;
  int8_t   delay_until_status; /* osStatus_t; osErrorParameter => immediate */
  uint8_t  flags; /* bit0 lattice deadline_miss, bit1 irq_masked, bit2 immediate_wake */
  uint8_t  _rsvd;
  uint16_t iter_ord;
} phase2_sonar_rec_t;

typedef struct __attribute__((packed)) {
  uint32_t actual_start_us; /* window_open TIM2 */
  uint32_t window_end_us;   /* s_exec: post-osDelayUntil TIM2 */
  uint32_t finish_us;
  uint32_t sched_wake_tick;
  uint32_t tick_before_delay;
  uint32_t tick_after_delay;
  uint16_t pulse_count;
  uint16_t iter_ord;
  int8_t   delay_until_status;
  uint8_t  flags; /* bit0 miss_compute, bit1 miss_output (1ms), bit2 immediate,
                   * bit3 short_window (<P-1ms elapsed) */
  uint8_t  _rsvd[2];
} phase2_wheel_rec_t;

/* HR Wheel-only compact record (16 B) — RAM-feasible at ~3k jobs. */
typedef struct __attribute__((packed)) {
  uint32_t s_exec_us;          /* post-osDelayUntil TIM2 (David s_k) */
  uint32_t finish_us;
  uint32_t sched_wake_tick;
  uint16_t iter_ord;
  int8_t   delay_until_status;
  uint8_t  flags; /* bit0 miss_compute, bit2 immediate, bit3 short_window */
} phase2_wheel_hr_rec_t;

_Static_assert(sizeof(phase2_sonar_rec_t) == 32, "sonar rec must be 32 bytes");
_Static_assert(sizeof(phase2_wheel_rec_t) == 32, "wheel rec must be 32 bytes");
_Static_assert(sizeof(phase2_wheel_hr_rec_t) == 16, "HR wheel rec must be 16 bytes");

#if RT_DMR_FORMAL
/* Formal DMR replaces phase2 job arrays (16 B × cap; HR 3100×16 ≈ 50 KiB). */
#if RT_HR_WHEEL_ONLY
static dmr_job_rec_t g_dmr_sonar[1];
static dmr_job_rec_t g_dmr_wheel[PHASE2_WHEEL_JOBS];
#else
static dmr_job_rec_t g_dmr_sonar[PHASE2_SONAR_JOBS];
static dmr_job_rec_t g_dmr_wheel[PHASE2_WHEEL_JOBS];
#endif
#else /* !RT_DMR_FORMAL — frozen RT corpora layout */
#if RT_HR_WHEEL_ONLY
static phase2_sonar_rec_t g_phase2_sonar[1]; /* unused; dump skips Sonar */
static phase2_wheel_hr_rec_t g_phase2_wheel[PHASE2_WHEEL_JOBS];
#else
static phase2_sonar_rec_t g_phase2_sonar[PHASE2_SONAR_JOBS];
static phase2_wheel_rec_t g_phase2_wheel[PHASE2_WHEEL_JOBS];
#endif
#endif /* RT_DMR_FORMAL */
static uint32_t g_phase2_sonar_epoch_us;
static uint32_t g_phase2_wheel_epoch_us;
static volatile uint32_t g_phase2_sonar_n;
static volatile uint32_t g_phase2_wheel_n;
static volatile uint32_t g_phase2_sonar_overflow;
static volatile uint32_t g_phase2_wheel_overflow;
static volatile uint32_t g_phase2_sonar_done;
static volatile uint32_t g_phase2_wheel_done;
static volatile uint32_t g_phase2_csv_dumped;
static volatile uint32_t g_phase2_wall_start_us; /* TIM2 at first recorded job / obs T0 */
static volatile uint32_t g_phase2_wall_end_us;   /* sensor observe end (pre-attest-drain) */
#if PHASE3_WITH_ORIGINAL_SMARM
static volatile uint32_t g_phase2_wall_dump_us;  /* after attest drain + before CSV */
static volatile uint32_t g_phase3_attest_stop;
static volatile uint32_t g_phase3_attest_idle;
static volatile uint32_t g_phase3_attest_rounds;
static volatile uint32_t g_phase3_attest_rounds_at_stop; /* completed before stop */
static volatile uint32_t g_phase3_attest_rounds_boundary; /* finished after stop (0/1) */
static volatile uint32_t g_phase3_attest_incomplete; /* 1 if drain timed out mid-NSC */
static volatile uint32_t g_phase3_attest_tdis_avg;
static volatile uint32_t g_phase3_attest_tdis_max;
static volatile uint32_t g_phase3_attest_last_ms;
#if RT_WALL_OBS_ENABLE
static volatile uint32_t g_phase3_attest_rounds_in_obs; /* NSC finish in [T0,T0+60s) */
#endif
#endif

#if RT_WALL_OBS_ENABLE
/* Fixed TIM2 observation: warm-up then [T0, T0+duration). Not RTOS-tick gated.
 * See thesis/03_rt_experiment/METRIC_SEMANTICS_wall30x3.md */
static volatile uint32_t g_obs_epoch; /* both tasks ready */
static volatile uint32_t g_obs_t0;    /* epoch + warmup */
static volatile uint32_t g_obs_tend;  /* t0 + duration */
static volatile uint32_t g_obs_ready; /* bit0 sonar, bit1 wheel */
static volatile uint32_t g_obs_tick0;
static volatile uint32_t g_obs_tick_end;
#if RT_DMR_FORMAL
/* TIM2 time of the tick0 SysTick edge. Not T0, and not a task wake. */
static volatile uint32_t g_obs_tick0_tim2;
static volatile uint32_t g_obs_tick0_sample_us;
static volatile uint32_t g_obs_tick0_elapsed_us;
static volatile uint32_t g_obs_tick0_systick_load;
static volatile uint32_t g_obs_tick0_systick_val;
static volatile uint32_t g_obs_tick0_val_span;
#endif
static volatile uint32_t g_obs_drain_us;
/* Per-task tick at observation-loop entry (after warm-up), before next-=P. */
static volatile uint32_t g_sonar_tick_entry;
static volatile uint32_t g_wheel_tick_entry;
static volatile uint32_t g_sonar_first_immediate;
static volatile uint32_t g_wheel_first_immediate;
static volatile uint32_t g_wheel_first_short_window;
static volatile uint32_t g_sonar_starts;
static volatile uint32_t g_sonar_comps;
static volatile uint32_t g_wheel_starts;
static volatile uint32_t g_wheel_comps;
static uint16_t g_sonar_bin_start[RT_OBS_BINS];
static uint16_t g_sonar_bin_comp[RT_OBS_BINS];
static uint16_t g_wheel_bin_start[RT_OBS_BINS];
static uint16_t g_wheel_bin_comp[RT_OBS_BINS];
/* Interval stats on consecutive s_exec (post-DelayUntil) in observation. */
static uint32_t g_sonar_isum, g_sonar_imax, g_sonar_imin;
static uint32_t g_sonar_osum, g_sonar_omax, g_sonar_omin;
static uint32_t g_sonar_ipairs, g_sonar_opairs;
static uint32_t g_wheel_isum, g_wheel_imax, g_wheel_imin;
static uint32_t g_wheel_osum, g_wheel_omax, g_wheel_omin;
static uint32_t g_wheel_ipairs, g_wheel_opairs;

static int obs_in_window(uint32_t t_us)
{
  return ((t_us - g_obs_t0) < RT_OBS_DURATION_US) ? 1 : 0;
}

static uint32_t obs_bin_index(uint32_t t_us)
{
  uint32_t d = t_us - g_obs_t0;
  if (d >= RT_OBS_DURATION_US) {
    return RT_OBS_BINS; /* invalid */
  }
  return d / 1000000U;
}

static void obs_note_interval(uint32_t *sum, uint32_t *mx, uint32_t *mn,
                              uint32_t *npairs, uint32_t dt)
{
  if (*npairs == 0U) {
    *mn = dt;
    *mx = dt;
  } else {
    if (dt > *mx) {
      *mx = dt;
    }
    if (dt < *mn) {
      *mn = dt;
    }
  }
  *sum += dt;
  (*npairs)++;
}

#if RT_DMR_FORMAL
typedef struct {
  uint32_t tick;
  uint32_t tim2_us;
  uint32_t sample_us;
  uint32_t elapsed_us;
  uint32_t load;
  uint32_t val;
  uint32_t val_span_cyc;
} dmr_tick_anchor_t;

/* TIM2 timestamp of the current SysTick edge.
 * IRQs are masked only for the snapshot and restored to the caller's PRIMASK.
 * Retries if the counter reloads between the two VAL reads so the edge is not
 * inferred from T0 or from a later osDelayUntil wake. */
static void dmr_latch_tick_edge(dmr_tick_anchor_t *a)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t load;
  uint32_t val_before;
  uint32_t val_after;
  uint32_t us;
  uint32_t tick;
  uint32_t elapsed_cyc;
  uint32_t spins = 0U;

  __disable_irq();
  load = SysTick->LOAD;
  do {
    val_before = SysTick->VAL;
    us = us_now();
    val_after = SysTick->VAL;
    tick = (uint32_t)xTaskGetTickCountFromISR();
    spins++;
  } while ((val_after > val_before) && (spins < 8U));
  if (!primask) {
    __enable_irq();
  }
  if (val_after > val_before) {
    val_after = val_before;
  }
  elapsed_cyc = (val_after <= load) ? (load - val_after) : 0U;
  a->tick = tick;
  a->sample_us = us;
  a->load = load;
  a->val = val_after;
  a->val_span_cyc = val_before - val_after;
  if ((load + 1U) == 0U) {
    a->elapsed_us = 0U;
    a->tim2_us = us;
  } else {
    a->elapsed_us =
        (uint32_t)(((uint64_t)elapsed_cyc * 1000ULL) / ((uint64_t)load + 1ULL));
    a->tim2_us = us - a->elapsed_us;
  }
}
#endif

/* Last ready task publishes epoch; T0 = epoch + warmup. Block until T0.
 * Dual-task: bits 0|1 must both be set. HR Wheel-only: bit1 alone starts obs. */
static void obs_sync_t0(uint32_t ready_bit)
{
  uint32_t primask = __get_PRIMASK();
  const uint32_t need =
#if RT_HR_WHEEL_ONLY
      2U;
#else
      3U;
#endif
  __disable_irq();
  g_obs_ready |= ready_bit;
  if (((g_obs_ready & need) == need) && (g_obs_epoch == 0U)) {
    g_obs_epoch = us_now();
    g_obs_t0 = g_obs_epoch + RT_OBS_WARMUP_US;
    g_obs_tend = g_obs_t0 + RT_OBS_DURATION_US;
    g_phase2_wall_start_us = g_obs_t0;
  }
  if (!primask) {
    __enable_irq();
  }
  while (g_obs_epoch == 0U) {
    osDelay(1);
  }
  /* Discard warm-up on TIM2 wall; tick0 sampled at observation entry. */
  while (us_now() < g_obs_t0) {
    osDelay(1);
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if (g_obs_tick0 == 0U) {
#if RT_DMR_FORMAL
    dmr_tick_anchor_t anchor;
    dmr_latch_tick_edge(&anchor);
    g_obs_tick0 = anchor.tick;
    g_obs_tick0_tim2 = anchor.tim2_us;
    g_obs_tick0_sample_us = anchor.sample_us;
    g_obs_tick0_elapsed_us = anchor.elapsed_us;
    g_obs_tick0_systick_load = anchor.load;
    g_obs_tick0_systick_val = anchor.val;
    g_obs_tick0_val_span = anchor.val_span_cyc;
#else
    g_obs_tick0 = osKernelGetTickCount();
#endif
  }
  if (!primask) {
    __enable_irq();
  }
}

#if RT_DMR_FORMAL
static const char *dmr_status_name(uint8_t st)
{
  switch (st) {
  case DMR_ST_ONTIME:
    return "ontime";
  case DMR_ST_LATE:
    return "late";
  case DMR_ST_INCOMPLETE_MISS:
    return "incomplete_miss";
  case DMR_ST_CENSORED:
    return "censored";
  default:
    return "instr_lost";
  }
}

static void dmr_busy_inject_delay(void)
{
#if RT_DMR_INJECT_DELAY_US > 0
  uint32_t t0 = us_now();
  while ((us_now() - t0) < (uint32_t)RT_DMR_INJECT_DELAY_US) {
    /* intentional validation delay — not in campaign builds */
  }
#else
  /* no-op */
#endif
}

/* Warm-up filter uses derived r_us (not stored). */
static int dmr_release_in_obs(uint32_t W)
{
  uint32_t r_us = dmr_map_release_us(g_obs_tick0_tim2, g_obs_tick0, W);
  /* The tick edge can sit up to one tick before the sample, hence before T0. */
  return ((int32_t)(r_us - (g_obs_t0 - 1000U)) >= 0) ? 1 : 0;
}

/* Store ledger row if release is not pre-T0 warm-up. Overflow → instr_lost count. */
static void dmr_store(dmr_job_rec_t *buf, volatile uint32_t *n,
                      volatile uint32_t *ovf, uint32_t cap,
                      const dmr_job_rec_t *src)
{
  if (!dmr_release_in_obs(src->sched_wake_tick)) {
    return; /* warm-up / pre-obs release — not eligible */
  }
  if (*n < cap) {
    buf[*n] = *src;
    (*n)++;
  } else {
    (*ovf)++;
  }
}

static void dmr_fill(dmr_job_rec_t *out, uint32_t W, uint32_t period_us,
                     uint32_t s_us, uint32_t f_us, int have_finish,
                     uint32_t window_open_us, uint8_t flags)
{
  uint32_t r_us = dmr_map_release_us(g_obs_tick0_tim2, g_obs_tick0, W);
  uint32_t d_us = dmr_deadline_us(r_us, period_us);
  out->sched_wake_tick = W;
  out->s_us = s_us;
  out->f_us = have_finish ? f_us : 0U;
  out->window_open_before_ms = dmr_open_before_ms(window_open_us, s_us);
  out->flags = flags;
  out->status = dmr_classify(d_us, g_obs_tend, f_us, have_finish);
}
#endif /* RT_DMR_FORMAL */
#endif /* RT_WALL_OBS_ENABLE */

#if !RT_WALL_OBS_ENABLE
static void phase2_note_wall_start(uint32_t t_us)
{
  /* First writer wins; both tasks sample within ~1 ms of true start. */
  if (g_phase2_wall_start_us == 0U) {
    g_phase2_wall_start_us = t_us;
  }
}
#endif

static void phase2_try_dump_csv(void)
{
  char line[480];
  uint32_t i;
  int n;
  uint32_t wall_elapsed_us;

  if (g_phase2_sonar_done == 0U || g_phase2_wheel_done == 0U) {
    return;
  }
  if (g_phase2_csv_dumped != 0U) {
    return;
  }
  g_phase2_csv_dumped = 1U;
#if RT_WALL_OBS_ENABLE
  /* Measurement cutoff is TIM2 Tend = T0+duration (not job count, not tick). */
  g_phase2_wall_end_us = g_obs_tend;
  wall_elapsed_us = RT_OBS_DURATION_US;
  if (g_obs_tick_end == 0U) {
    g_obs_tick_end = osKernelGetTickCount();
  }
#else
  /* Sensor window wall — same stamp point as Phase 2.1 (before CSV / attest drain). */
  g_phase2_wall_end_us = us_now();
  wall_elapsed_us = g_phase2_wall_end_us - g_phase2_wall_start_us;
#endif
#if PHASE3_WITH_ORIGINAL_SMARM
  g_phase3_attest_rounds_at_stop = g_phase3_attest_rounds;
  g_phase3_attest_stop = 1U;
  {
    uint32_t wait_n = 0U;
    uint32_t drain0 = us_now();
    while ((g_phase3_attest_idle == 0U) && (wait_n < 6000U)) {
      osDelay(10);
      wait_n++;
    }
    if (g_phase3_attest_idle == 0U) {
      g_phase3_attest_incomplete = 1U; /* drain timeout; in-flight NSC not finished */
    }
#if RT_WALL_OBS_ENABLE
    g_obs_drain_us = us_now() - drain0;
#endif
  }
  g_phase3_attest_rounds_boundary =
      g_phase3_attest_rounds - g_phase3_attest_rounds_at_stop;
  g_phase2_wall_dump_us = us_now();
#endif

#if PHASE3_WITH_ORIGINAL_SMARM
  n = snprintf(line, sizeof(line),
               "\r\n=== PHASE3 SMARM + sensors dump ===\r\n"
               "profile=%s run=%s shuffle_mode=%s B=%u IRQ_HMAC=%u "
               "USE_SAU=%u USE_SNAP=%u |M|=0x%lX attest_rounds=%lu "
               "rounds_at_stop=%lu rounds_boundary=%lu incomplete=%lu "
               "tdis_avg=%lu tdis_max=%lu last_round_ms=%lu\r\n",
               RT_PROFILE_NAME, RT_RUN_NAME,
               ATTEST_SHUFFLE_MODE_NAME,
               (unsigned)ATTEST_BLOCK_SIZE,
               (unsigned)ATTEST_IRQ_HMAC_MASK,
               (unsigned)USE_SAU_APPROACH,
               (unsigned)USE_SNAP,
               (unsigned long)0x20000UL, /* 128 KiB @ 0x08060000 when FAULT_DEMO=0 */
               (unsigned long)g_phase3_attest_rounds,
               (unsigned long)g_phase3_attest_rounds_at_stop,
               (unsigned long)g_phase3_attest_rounds_boundary,
               (unsigned long)g_phase3_attest_incomplete,
               (unsigned long)g_phase3_attest_tdis_avg,
               (unsigned long)g_phase3_attest_tdis_max,
               (unsigned long)g_phase3_attest_last_ms);
  if (n > 0) {
    ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
  }
  n = snprintf(line, sizeof(line),
               "sonar_n=%lu wheel_n=%lu sonar_ovf=%lu wheel_ovf=%lu\r\n"
               "wall_start_us=%lu wall_sensor_end_us=%lu wall_sensor_elapsed_us=%lu "
               "wall_dump_us=%lu wall_target_us=%lu\r\n",
               (unsigned long)g_phase2_sonar_n,
               (unsigned long)g_phase2_wheel_n,
               (unsigned long)g_phase2_sonar_overflow,
               (unsigned long)g_phase2_wheel_overflow,
               (unsigned long)g_phase2_wall_start_us,
               (unsigned long)g_phase2_wall_end_us,
               (unsigned long)wall_elapsed_us,
               (unsigned long)g_phase2_wall_dump_us,
               (unsigned long)PHASE2_WALL_TARGET_US);
#else
  n = snprintf(line, sizeof(line),
               "\r\n=== No-attest baseline dump profile=%s run=%s ===\r\n"
               "sonar_period_ms=%u wheel_period_ms=%u "
               "sonar_n=%lu wheel_n=%lu sonar_ovf=%lu wheel_ovf=%lu\r\n"
               "wall_start_us=%lu wall_sensor_end_us=%lu wall_sensor_elapsed_us=%lu "
               "wall_target_us=%lu\r\n",
               RT_PROFILE_NAME, RT_RUN_NAME,
               (unsigned)PHASE2_SONAR_PERIOD_MS,
               (unsigned)PHASE2_WHEEL_PERIOD_MS,
               (unsigned long)g_phase2_sonar_n,
               (unsigned long)g_phase2_wheel_n,
               (unsigned long)g_phase2_sonar_overflow,
               (unsigned long)g_phase2_wheel_overflow,
               (unsigned long)g_phase2_wall_start_us,
               (unsigned long)g_phase2_wall_end_us,
               (unsigned long)wall_elapsed_us,
               (unsigned long)PHASE2_WALL_TARGET_US);
#endif
  if (n > 0) {
    ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
  }
#if RT_WALL_OBS_ENABLE
  {
    uint32_t tick_delta = g_obs_tick_end - g_obs_tick0;
    uint32_t s_starts = g_sonar_starts;
    uint32_t w_starts = g_wheel_starts;
    uint32_t s_comps = g_sonar_comps;
    uint32_t w_comps = g_wheel_comps;
    /* frequency in milliHz: starts * 1000 / observation_seconds */
    uint32_t obs_sec = RT_OBS_DURATION_US / 1000000U;
    uint32_t s_mHz = (obs_sec > 0U) ? ((s_starts * 1000U) / obs_sec) : 0U;
    uint32_t w_mHz = (obs_sec > 0U) ? ((w_starts * 1000U) / obs_sec) : 0U;
    uint32_t s_ratio_ppm = (s_starts * 1000000U) / RT_OBS_NOMINAL_JOBS;
    uint32_t w_ratio_ppm = (w_starts * 1000000U) / RT_OBS_NOMINAL_JOBS;
    uint32_t s_comp_ppm = (s_comps * 1000000U) / RT_OBS_NOMINAL_JOBS;
    uint32_t w_comp_ppm = (w_comps * 1000000U) / RT_OBS_NOMINAL_JOBS;
    uint32_t bin;
    uint32_t s_bin_min = 0xFFFFFFFFu;
    uint32_t s_bin_max = 0U;
    uint32_t w_bin_min = 0xFFFFFFFFu;
    uint32_t w_bin_max = 0U;
    uint32_t s_bins_lt10 = 0U;
    uint32_t w_bins_lt10 = 0U;
    for (bin = 0U; bin < RT_OBS_BINS; bin++) {
      uint32_t ss = g_sonar_bin_start[bin];
      uint32_t ws = g_wheel_bin_start[bin];
      if (ss < s_bin_min) {
        s_bin_min = ss;
      }
      if (ss > s_bin_max) {
        s_bin_max = ss;
      }
      if (ws < w_bin_min) {
        w_bin_min = ws;
      }
      if (ws > w_bin_max) {
        w_bin_max = ws;
      }
      if (ss < 10U) {
        s_bins_lt10++;
      }
      if (ws < 10U) {
        w_bins_lt10++;
      }
    }
    n = snprintf(line, sizeof(line),
                 "OBS_WALL mode=TIM2_warmup_plus_fixed T0=%lu Tend=%lu "
                 "epoch=%lu warmup_us=%lu duration_us=%lu drain_us=%lu\r\n"
                 "nominal_jobs=%u P_us=%u sonar_lattice_D_us=%u "
                 "wheel_D_compute_us=%u wheel_D_formal=%s "
                 "tick0=%lu tick_end=%lu tick_delta=%lu "
                 "tick_expected_approx=%lu\r\n",
                 (unsigned long)g_obs_t0,
                 (unsigned long)g_obs_tend,
                 (unsigned long)g_obs_epoch,
                 (unsigned long)RT_OBS_WARMUP_US,
                 (unsigned long)RT_OBS_DURATION_US,
#if PHASE3_WITH_ORIGINAL_SMARM
                 (unsigned long)g_obs_drain_us,
#else
                 0UL,
#endif
                 (unsigned)RT_OBS_NOMINAL_JOBS,
                 (unsigned)RT_OBS_PERIOD_US,
                 (unsigned)RT_SONAR_D_US,
                 (unsigned)RT_WHEEL_D_COMPUTE_US,
#if RT_DMR_FORMAL
                 "APPROVED_Q1Q5_D_eq_P",
#else
                 "NOT_APPROVED",
#endif
                 (unsigned long)g_obs_tick0,
                 (unsigned long)g_obs_tick_end,
                 (unsigned long)tick_delta,
                 (unsigned long)(RT_OBS_DURATION_US / 1000U));
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    n = snprintf(line, sizeof(line),
                 "sonar_tick_entry=%lu wheel_tick_entry=%lu "
                 "sonar_first_immediate=%lu wheel_first_immediate=%lu "
                 "wheel_first_short_window=%lu\r\n"
                 "NOTE_INSTR: sched_wake_tick is DelayUntil target; "
                 "wall_sched_candidate_us is diagnostic NOT verified release; "
                 "lattice flags != formal DMR.\r\n",
                 (unsigned long)g_sonar_tick_entry,
                 (unsigned long)g_wheel_tick_entry,
                 (unsigned long)g_sonar_first_immediate,
                 (unsigned long)g_wheel_first_immediate,
                 (unsigned long)g_wheel_first_short_window);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    n = snprintf(line, sizeof(line),
                 "sonar_starts=%lu sonar_comps=%lu sonar_freq_mHz=%lu "
                 "sonar_start_ratio_ppm=%lu sonar_comp_ratio_ppm=%lu\r\n"
                 "wheel_starts=%lu wheel_comps=%lu wheel_freq_mHz=%lu "
                 "wheel_start_ratio_ppm=%lu wheel_comp_ratio_ppm=%lu\r\n",
                 (unsigned long)s_starts, (unsigned long)s_comps,
                 (unsigned long)s_mHz, (unsigned long)s_ratio_ppm,
                 (unsigned long)s_comp_ppm,
                 (unsigned long)w_starts, (unsigned long)w_comps,
                 (unsigned long)w_mHz, (unsigned long)w_ratio_ppm,
                 (unsigned long)w_comp_ppm);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    n = snprintf(line, sizeof(line),
                 "sonar_bin_starts min/max/lt10=%lu/%lu/%lu "
                 "wheel_bin_starts min/max/lt10=%lu/%lu/%lu\r\n"
                 "sonar_inter_start_us n/min/avg/max=%lu/%lu/%lu/%lu "
                 "sonar_out_iv_us n/min/avg/max=%lu/%lu/%lu/%lu\r\n"
                 "wheel_inter_start_us n/min/avg/max=%lu/%lu/%lu/%lu "
                 "wheel_out_iv_us n/min/avg/max=%lu/%lu/%lu/%lu\r\n",
                 (unsigned long)s_bin_min, (unsigned long)s_bin_max,
                 (unsigned long)s_bins_lt10,
                 (unsigned long)w_bin_min, (unsigned long)w_bin_max,
                 (unsigned long)w_bins_lt10,
                 (unsigned long)g_sonar_ipairs, (unsigned long)g_sonar_imin,
                 (unsigned long)((g_sonar_ipairs > 0U) ? (g_sonar_isum / g_sonar_ipairs) : 0U),
                 (unsigned long)g_sonar_imax,
                 (unsigned long)g_sonar_opairs, (unsigned long)g_sonar_omin,
                 (unsigned long)((g_sonar_opairs > 0U) ? (g_sonar_osum / g_sonar_opairs) : 0U),
                 (unsigned long)g_sonar_omax,
                 (unsigned long)g_wheel_ipairs, (unsigned long)g_wheel_imin,
                 (unsigned long)((g_wheel_ipairs > 0U) ? (g_wheel_isum / g_wheel_ipairs) : 0U),
                 (unsigned long)g_wheel_imax,
                 (unsigned long)g_wheel_opairs, (unsigned long)g_wheel_omin,
                 (unsigned long)((g_wheel_opairs > 0U) ? (g_wheel_osum / g_wheel_opairs) : 0U),
                 (unsigned long)g_wheel_omax);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
#if PHASE3_WITH_ORIGINAL_SMARM
    n = snprintf(line, sizeof(line),
                 "attest_rounds_in_obs=%lu (NSC finish with TIM2 in [T0,T0+60s))\r\n",
                 (unsigned long)g_phase3_attest_rounds_in_obs);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
#endif
    {
      const char bh[] =
          "BEGIN obs_per_second.csv\r\n"
          "sec,sonar_starts,sonar_comps,wheel_starts,wheel_comps\r\n";
      ns_uart_transmit((const uint8_t *)bh, (uint16_t)(sizeof(bh) - 1U));
    }
    for (bin = 0U; bin < RT_OBS_BINS; bin++) {
      n = snprintf(line, sizeof(line), "%lu,%u,%u,%u,%u\r\n",
                   (unsigned long)bin,
                   (unsigned)g_sonar_bin_start[bin],
                   (unsigned)g_sonar_bin_comp[bin],
                   (unsigned)g_wheel_bin_start[bin],
                   (unsigned)g_wheel_bin_comp[bin]);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    }
    {
      const char be[] = "END obs_per_second.csv\r\n";
      ns_uart_transmit((const uint8_t *)be, (uint16_t)(sizeof(be) - 1U));
    }
  }
#endif /* RT_WALL_OBS_ENABLE */
  {
    const char note[] =
        "NOTE: nominal_schedule_us is TIM2 model (not observed release). "
        "Negative schedule_start_dev = SysTick phase vs epoch; do not zero-subtract. "
        "schedule_to_finish is NOT formal release-to-completion response.\r\n"
#if RT_WALL_OBS_ENABLE
        "OBS: starts/comps if TIM2 stamp in [T0,Tend). "
        "s_exec=post-DelayUntil (Wheel: window_end). "
        "Formal DMR/jitter offline via release map — lattice flags != DMR.\r\n"
#endif
        ;
    ns_uart_transmit((const uint8_t *)note, (uint16_t)(sizeof(note) - 1U));
  }

#if RT_DMR_FORMAL
  {
    dmr_tick_anchor_t end_anchor;
    uint32_t delta_tick;
    uint32_t delta_tim2;
    int32_t continuity_err;
    dmr_latch_tick_edge(&end_anchor);
    delta_tick = end_anchor.tick - g_obs_tick0;
    delta_tim2 = end_anchor.tim2_us - g_obs_tick0_tim2;
    continuity_err = (int32_t)(delta_tim2 - (delta_tick * 1000U));
    n = snprintf(line, sizeof(line),
                 "DMR_ANCHOR method=systick_edge tick0=%lu tick0_tim2_us=%lu "
                 "sample_us=%lu elapsed_us=%lu systick_load=%lu systick_val=%lu "
                 "val_span_cyc=%lu T0_us=%lu Tend_us=%lu\r\n",
                 (unsigned long)g_obs_tick0,
                 (unsigned long)g_obs_tick0_tim2,
                 (unsigned long)g_obs_tick0_sample_us,
                 (unsigned long)g_obs_tick0_elapsed_us,
                 (unsigned long)g_obs_tick0_systick_load,
                 (unsigned long)g_obs_tick0_systick_val,
                 (unsigned long)g_obs_tick0_val_span,
                 (unsigned long)g_obs_t0,
                 (unsigned long)g_obs_tend);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    n = snprintf(line, sizeof(line),
                 "DMR_CLOCK tick_end=%lu tick_end_tim2_us=%lu delta_tick=%lu "
                 "delta_tim2_us=%lu continuity_err_us=%ld\r\n",
                 (unsigned long)end_anchor.tick,
                 (unsigned long)end_anchor.tim2_us,
                 (unsigned long)delta_tick,
                 (unsigned long)delta_tim2,
                 (long)continuity_err);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
    n = snprintf(line, sizeof(line),
                 "DMR_SESSION boot=%lu magic=DMR1\r\n",
                 (unsigned long)g_dmr_boot_count);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
    }
  }
  {
    const char hdr[] =
        "BEGIN formal_dmr.csv\r\n"
        "task,job_id,iter_ord,sched_wake_tick,r_us,d_us,s_us,f_us,"
        "window_open_us,status,status_code,flags,"
        "T0_us,Tend_us,tick0,tick0_tim2_us,period_us,D_equals_P\r\n";
    ns_uart_transmit((const uint8_t *)hdr, (uint16_t)(sizeof(hdr) - 1U));
  }
  {
    const uint32_t sonar_p = PHASE2_SONAR_PERIOD_MS * 1000U;
    const uint32_t wheel_p = PHASE2_WHEEL_PERIOD_MS * 1000U;
#if !RT_HR_WHEEL_ONLY
    for (i = 0U; i < g_phase2_sonar_n; i++) {
      const dmr_job_rec_t *r = &g_dmr_sonar[i];
      uint32_t r_us = dmr_map_release_us(g_obs_tick0_tim2, g_obs_tick0, r->sched_wake_tick);
      uint32_t d_us = dmr_deadline_us(r_us, sonar_p);
      uint32_t wopen = dmr_window_open_us(r->s_us, r->window_open_before_ms);
      n = snprintf(line, sizeof(line),
                   "sonar,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%s,%u,%u,"
                   "%lu,%lu,%lu,%lu,%lu,1\r\n",
                   (unsigned long)i,
                   (unsigned)i,
                   (unsigned long)r->sched_wake_tick,
                   (unsigned long)r_us,
                   (unsigned long)d_us,
                   (unsigned long)r->s_us,
                   (unsigned long)r->f_us,
                   (unsigned long)wopen,
                   dmr_status_name(r->status),
                   (unsigned)r->status,
                   (unsigned)r->flags,
                   (unsigned long)g_obs_t0,
                   (unsigned long)g_obs_tend,
                   (unsigned long)g_obs_tick0,
                   (unsigned long)g_obs_tick0_tim2,
                   (unsigned long)sonar_p);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    }
#else
    (void)sonar_p;
#endif
    for (i = 0U; i < g_phase2_wheel_n; i++) {
      const dmr_job_rec_t *r = &g_dmr_wheel[i];
      uint32_t r_us = dmr_map_release_us(g_obs_tick0_tim2, g_obs_tick0, r->sched_wake_tick);
      uint32_t d_us = dmr_deadline_us(r_us, wheel_p);
      uint32_t wopen = dmr_window_open_us(r->s_us, r->window_open_before_ms);
      n = snprintf(line, sizeof(line),
                   "wheel,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%s,%u,%u,"
                   "%lu,%lu,%lu,%lu,%lu,1\r\n",
                   (unsigned long)i,
                   (unsigned)i,
                   (unsigned long)r->sched_wake_tick,
                   (unsigned long)r_us,
                   (unsigned long)d_us,
                   (unsigned long)r->s_us,
                   (unsigned long)r->f_us,
                   (unsigned long)wopen,
                   dmr_status_name(r->status),
                   (unsigned)r->status,
                   (unsigned)r->flags,
                   (unsigned long)g_obs_t0,
                   (unsigned long)g_obs_tend,
                   (unsigned long)g_obs_tick0,
                   (unsigned long)g_obs_tick0_tim2,
                   (unsigned long)wheel_p);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    }
  }
  {
    char end[180];
    int en;
#if PHASE3_WITH_ORIGINAL_SMARM
    en = snprintf(end, sizeof(end),
                  "END formal_dmr.csv\r\n"
                  "=== PHASE3 DMR done profile=%s run=%s. Idle. ===\r\n",
                  RT_PROFILE_NAME, RT_RUN_NAME);
#else
    en = snprintf(end, sizeof(end),
                  "END formal_dmr.csv\r\n"
                  "=== No-attest DMR done profile=%s run=%s. Idle. ===\r\n",
                  RT_PROFILE_NAME, RT_RUN_NAME);
#endif
    if (en > 0) {
      ns_uart_transmit((const uint8_t *)end, (uint16_t)en);
    }
  }
#else /* !RT_DMR_FORMAL — frozen RT dump path */
#if RT_HR_WHEEL_ONLY
  {
    const char skip[] =
        "BEGIN sonar_timing.csv\r\n"
        "NOTE: HR Wheel-only profile — SonarTask not created; no sonar rows.\r\n"
        "END sonar_timing.csv\r\n";
    ns_uart_transmit((const uint8_t *)skip, (uint16_t)(sizeof(skip) - 1U));
  }
#else
  {
    const char hdr1[] = "BEGIN sonar_timing.csv\r\n";
    const char hdr2a[] =
        "job_id,nominal_schedule_us,actual_start_us,finish_us,"
        "schedule_to_finish_us,exec_us,schedule_start_dev_us,interval_dev_us,"
        "deadline_miss,";
    const char hdr2b[] =
        "actual_interval_us,output_interval_us,"
        "distance_cm,status,echo_us,measure_span_us,irq_masked,"
        "iter_ord,sched_wake_tick,tick_before_delay,delay_until_status,"
        "tick_after_delay,immediate_wake,"
        "wall_sched_candidate_us\r\n";
    ns_uart_transmit((const uint8_t *)hdr1, (uint16_t)(sizeof(hdr1) - 1U));
    ns_uart_transmit((const uint8_t *)hdr2a, (uint16_t)(sizeof(hdr2a) - 1U));
    ns_uart_transmit((const uint8_t *)hdr2b, (uint16_t)(sizeof(hdr2b) - 1U));
  }
  {
    const uint32_t period_us = PHASE2_SONAR_PERIOD_MS * 1000U;
    uint32_t prev_start = 0U;
    uint32_t prev_finish = 0U;
    for (i = 0U; i < g_phase2_sonar_n; i++) {
      const phase2_sonar_rec_t *r = &g_phase2_sonar[i];
      uint32_t nominal = g_phase2_sonar_epoch_us + (i + 1U) * period_us;
      uint32_t wall_cand = g_phase2_sonar_epoch_us + (uint32_t)r->iter_ord * period_us;
      uint32_t stf = r->finish_us - nominal;
      uint32_t exec = r->finish_us - r->actual_start_us;
      int32_t sdev = (int32_t)(r->actual_start_us - nominal);
      uint32_t interval_us = 0U;
      uint32_t output_interval_us = 0U;
      int32_t interval_dev = 0;
      if (i > 0U) {
        interval_us = r->actual_start_us - prev_start;
        interval_dev = (int32_t)interval_us - (int32_t)period_us;
        output_interval_us = r->finish_us - prev_finish;
      }
      prev_start = r->actual_start_us;
      prev_finish = r->finish_us;
      n = snprintf(line, sizeof(line),
                   "%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%u,%lu,%lu,%d,%d,%lu,%lu,%u,"
                   "%u,%lu,%lu,%d,%lu,%u,%lu\r\n",
                   (unsigned long)i,
                   (unsigned long)nominal,
                   (unsigned long)r->actual_start_us,
                   (unsigned long)r->finish_us,
                   (unsigned long)stf,
                   (unsigned long)exec,
                   (long)sdev,
                   (long)interval_dev,
                   (unsigned)(r->flags & 1U),
                   (unsigned long)interval_us,
                   (unsigned long)output_interval_us,
                   (int)r->distance_cm,
                   (int)r->status,
                   (unsigned long)r->echo_us,
                   (unsigned long)r->measure_span_us,
                   (unsigned)((r->flags >> 1) & 1U),
                   (unsigned)r->iter_ord,
                   (unsigned long)r->sched_wake_tick,
                   (unsigned long)r->tick_before_delay,
                   (int)r->delay_until_status,
                   (unsigned long)r->tick_after_delay,
                   (unsigned)((r->flags >> 2) & 1U),
                   (unsigned long)wall_cand);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
    }
  }
  {
    const char end[] = "END sonar_timing.csv\r\n";
    ns_uart_transmit((const uint8_t *)end, (uint16_t)(sizeof(end) - 1U));
  }
#endif /* !RT_HR_WHEEL_ONLY */

  {
    const char hdr1[] = "BEGIN wheel_timing.csv\r\n";
#if RT_HR_WHEEL_ONLY
    const char hdr2[] =
        "job_id,s_exec_us,finish_us,compute_us,iter_ord,sched_wake_tick,"
        "delay_until_status,immediate_wake,short_window,miss_compute,"
        "actual_interval_us,interval_dev_us\r\n";
    ns_uart_transmit((const uint8_t *)hdr1, (uint16_t)(sizeof(hdr1) - 1U));
    ns_uart_transmit((const uint8_t *)hdr2, (uint16_t)(sizeof(hdr2) - 1U));
#else
    const char hdr2a[] =
        "job_id,nominal_schedule_us,window_open_us,nominal_window_end_us,"
        "s_exec_us,finish_us,schedule_start_dev_us,interval_dev_us,";
    const char hdr2b[] =
        "compute_us,schedule_output_dev_us,schedule_to_finish_us,miss_compute,miss_output,"
        "actual_interval_us,output_interval_us,"
        "pulse_count,window_elapsed_us,rpm_nominal,rpm_elapsed,"
        "iter_ord,sched_wake_tick,tick_before_delay,delay_until_status,"
        "tick_after_delay,immediate_wake,short_window,"
        "wall_sched_candidate_us\r\n";
    ns_uart_transmit((const uint8_t *)hdr1, (uint16_t)(sizeof(hdr1) - 1U));
    ns_uart_transmit((const uint8_t *)hdr2a, (uint16_t)(sizeof(hdr2a) - 1U));
    ns_uart_transmit((const uint8_t *)hdr2b, (uint16_t)(sizeof(hdr2b) - 1U));
#endif
  }
  {
    const uint32_t period_us = PHASE2_WHEEL_PERIOD_MS * 1000U;
    uint32_t prev_start = 0U;
    uint32_t prev_finish = 0U;
    for (i = 0U; i < g_phase2_wheel_n; i++) {
#if RT_HR_WHEEL_ONLY
      const phase2_wheel_hr_rec_t *r = &g_phase2_wheel[i];
      uint32_t compute_us = r->finish_us - r->s_exec_us;
      uint32_t interval_us = 0U;
      int32_t interval_dev = 0;
      if (i > 0U) {
        interval_us = r->s_exec_us - prev_start;
        interval_dev = (int32_t)interval_us - (int32_t)period_us;
      }
      prev_start = r->s_exec_us;
      prev_finish = r->finish_us;
      (void)prev_finish;
      n = snprintf(line, sizeof(line),
                   "%lu,%lu,%lu,%lu,%u,%lu,%d,%u,%u,%u,%lu,%ld\r\n",
                   (unsigned long)i,
                   (unsigned long)r->s_exec_us,
                   (unsigned long)r->finish_us,
                   (unsigned long)compute_us,
                   (unsigned)r->iter_ord,
                   (unsigned long)r->sched_wake_tick,
                   (int)r->delay_until_status,
                   (unsigned)((r->flags >> 2) & 1U),
                   (unsigned)((r->flags >> 3) & 1U),
                   (unsigned)(r->flags & 1U),
                   (unsigned long)interval_us,
                   (long)interval_dev);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
#else
      const phase2_wheel_rec_t *r = &g_phase2_wheel[i];
      uint32_t nominal = g_phase2_wheel_epoch_us + i * period_us;
      uint32_t nominal_wend = nominal + period_us;
      uint32_t wall_cand = g_phase2_wheel_epoch_us + (uint32_t)r->iter_ord * period_us;
      uint32_t window_elapsed = r->window_end_us - r->actual_start_us;
      uint32_t compute_us = r->finish_us - r->window_end_us;
      uint32_t stf = r->finish_us - nominal;
      int32_t sdev = (int32_t)(r->actual_start_us - nominal);
      int32_t out_dev = (int32_t)(r->finish_us - nominal_wend);
      uint32_t interval_us = 0U;
      uint32_t output_interval_us = 0U;
      int32_t interval_dev = 0;
      uint32_t rpm_nom;
      uint32_t rpm_el;
      if (i > 0U) {
        interval_us = r->window_end_us - prev_start;
        interval_dev = (int32_t)interval_us - (int32_t)period_us;
        output_interval_us = r->finish_us - prev_finish;
      }
      prev_start = r->window_end_us;
      prev_finish = r->finish_us;
      rpm_nom = ((uint32_t)r->pulse_count * 60U) / PULSE_TEETH_PER_REV;
      if (window_elapsed > 0U) {
        rpm_el = (uint32_t)(((uint64_t)r->pulse_count * 60ULL * 1000000ULL) /
                            ((uint64_t)PULSE_TEETH_PER_REV * (uint64_t)window_elapsed));
      } else {
        rpm_el = 0U;
      }
      if (rpm_nom > 65535U) {
        rpm_nom = 65535U;
      }
      if (rpm_el > 65535U) {
        rpm_el = 65535U;
      }
      n = snprintf(line, sizeof(line),
                   "%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%ld,%lu,%u,%u,%lu,%lu,%lu,%lu,%u,%u,"
                   "%u,%lu,%lu,%d,%lu,%u,%u,%lu\r\n",
                   (unsigned long)i,
                   (unsigned long)nominal,
                   (unsigned long)r->actual_start_us,
                   (unsigned long)nominal_wend,
                   (unsigned long)r->window_end_us,
                   (unsigned long)r->finish_us,
                   (long)sdev,
                   (long)interval_dev,
                   (unsigned long)compute_us,
                   (long)out_dev,
                   (unsigned long)stf,
                   (unsigned)(r->flags & 1U),
                   (unsigned)((r->flags >> 1) & 1U),
                   (unsigned long)interval_us,
                   (unsigned long)output_interval_us,
                   (unsigned long)r->pulse_count,
                   (unsigned long)window_elapsed,
                   (unsigned)rpm_nom,
                   (unsigned)rpm_el,
                   (unsigned)r->iter_ord,
                   (unsigned long)r->sched_wake_tick,
                   (unsigned long)r->tick_before_delay,
                   (int)r->delay_until_status,
                   (unsigned long)r->tick_after_delay,
                   (unsigned)((r->flags >> 2) & 1U),
                   (unsigned)((r->flags >> 3) & 1U),
                   (unsigned long)wall_cand);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
#endif /* !RT_HR_WHEEL_ONLY */
    }
  }
  {
    char end[140];
    int en;
#if PHASE3_WITH_ORIGINAL_SMARM
    en = snprintf(end, sizeof(end),
                  "END wheel_timing.csv\r\n"
                  "=== PHASE3 done profile=%s run=%s. Idle. ===\r\n",
                  RT_PROFILE_NAME, RT_RUN_NAME);
#else
    en = snprintf(end, sizeof(end),
                  "END wheel_timing.csv\r\n"
                  "=== No-attest done profile=%s run=%s. Idle. ===\r\n",
                  RT_PROFILE_NAME, RT_RUN_NAME);
#endif
    if (en > 0) {
      ns_uart_transmit((const uint8_t *)end, (uint16_t)en);
    }
  }
#endif /* RT_DMR_FORMAL */
}
#endif /* PHASE2_ENABLE */

static void PulseSmoke_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if PULSE_SMOKE_USE_ULTRASONIC
  us_tim_init();
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
#if PULSE_SMOKE_USE_ULTRASONIC
#if PHASE2_ENABLE
  {
    char go[200];
    int gn = snprintf(go, sizeof(go),
#if RT_WALL_OBS_ENABLE
                      "SonarTask profile=%s run=%s: TIM2 warm%lus+obs%lus @%ums D=%ums; echo %s.\r\n",
#else
                      "SonarTask profile=%s: %u jobs @%ums D=%ums; echo %s. Quiet until dump.\r\n",
#endif
                      RT_PROFILE_NAME,
#if RT_WALL_OBS_ENABLE
                      RT_RUN_NAME,
                      (unsigned long)(RT_OBS_WARMUP_US / 1000000U),
                      (unsigned long)(RT_OBS_DURATION_US / 1000000U),
#else
                      (unsigned)PHASE2_SONAR_JOBS,
#endif
                      (unsigned)PHASE2_SONAR_PERIOD_MS,
                      (unsigned)PHASE2_SONAR_DEADLINE_MS,
#if US_IRQ_MASK_DURING_ECHO
                      "IRQ-MASKED"
#else
                      "IRQ-ON"
#endif
    );
    if (gn > 0) {
      ns_uart_transmit((const uint8_t *)go, (uint16_t)gn);
    }
  }
  {
    const uint32_t period_ms = PHASE2_SONAR_PERIOD_MS;
    const uint32_t period_us = period_ms * 1000U;
    const uint32_t deadline_us = PHASE2_SONAR_DEADLINE_MS * 1000U;
    uint32_t next = osKernelGetTickCount();
    uint32_t epoch_us;
    uint32_t job = 0U;
#if RT_WALL_OBS_ENABLE
    uint32_t prev_start_us = 0U;
    uint32_t prev_finish_us = 0U;
    uint32_t have_prev = 0U;

    obs_sync_t0(1U);
    epoch_us = g_obs_t0;
    g_phase2_sonar_epoch_us = epoch_us;
    /* Preserve schedule: next init, next+=P, osDelayUntil(next). */
    g_sonar_tick_entry = osKernelGetTickCount();
    next = g_sonar_tick_entry;
    if (next >= period_ms) {
      next -= period_ms;
    }

    for (;;) {
      uint32_t nominal_schedule_us;
      uint32_t actual_start_us;
      uint32_t finish_us;
      uint32_t echo_us = 0U;
      uint32_t measure_span_us = 0U;
      int32_t cm;
      uint32_t schedule_to_finish_us;
      uint32_t bin;
      uint32_t sched_wake_tick;
      uint32_t tick_before;
      uint32_t tick_after;
      osStatus_t dly_st;
      uint8_t instr_flags;

      next += period_ms;
      sched_wake_tick = next;
      tick_before = osKernelGetTickCount();
      dly_st = osDelayUntil(next);
      tick_after = osKernelGetTickCount();

      actual_start_us = us_now(); /* s_exec: post-DelayUntil */
      if (!obs_in_window(actual_start_us)) {
#if RT_DMR_FORMAL
        /* Released job; Tend before/at start — incomplete or right-censored. */
        {
          dmr_job_rec_t rec;
          uint8_t f = 0U;
          if ((dly_st != osOK) || (tick_before >= sched_wake_tick)) {
            f |= 1U;
          }
          dmr_fill(&rec, sched_wake_tick, period_us, actual_start_us, 0U, 0,
                   0U, f);
          dmr_store(g_dmr_sonar, &g_phase2_sonar_n, &g_phase2_sonar_overflow,
                    PHASE2_SONAR_JOBS, &rec);
        }
#endif
        break; /* do not run work past TIM2 cutoff */
      }

      instr_flags = 0U;
      if ((dly_st != osOK) || (tick_before >= sched_wake_tick)) {
        instr_flags |= 4U; /* immediate_wake */
        if (job == 0U) {
          g_sonar_first_immediate = 1U;
        }
      }

      bin = obs_bin_index(actual_start_us);
      g_sonar_starts++;
      if (bin < RT_OBS_BINS) {
        g_sonar_bin_start[bin]++;
      }
      if (have_prev) {
        obs_note_interval(&g_sonar_isum, &g_sonar_imax, &g_sonar_imin,
                          &g_sonar_ipairs, actual_start_us - prev_start_us);
      }

      /* Legacy lattice label only — NOT verified release (kept for CSV compat). */
      nominal_schedule_us = epoch_us + (job + 1U) * period_us;
#if RT_DMR_FORMAL
      dmr_busy_inject_delay();
#endif
      cm = us_read_distance_cm_ex(&echo_us, &measure_span_us);
      finish_us = us_now();
      schedule_to_finish_us = finish_us - nominal_schedule_us;

      if (obs_in_window(finish_us)) {
        uint32_t b2 = obs_bin_index(finish_us);
        g_sonar_comps++;
        if (b2 < RT_OBS_BINS) {
          g_sonar_bin_comp[b2]++;
        }
        if (have_prev) {
          obs_note_interval(&g_sonar_osum, &g_sonar_omax, &g_sonar_omin,
                            &g_sonar_opairs, finish_us - prev_finish_us);
        }
      }

#if RT_DMR_FORMAL
      {
        dmr_job_rec_t rec;
        uint8_t f = 0U;
        if (instr_flags & 4U) {
          f |= 1U;
        }
        dmr_fill(&rec, sched_wake_tick, period_us, actual_start_us, finish_us, 1,
                 0U, f);
        dmr_store(g_dmr_sonar, &g_phase2_sonar_n, &g_phase2_sonar_overflow,
                  PHASE2_SONAR_JOBS, &rec);
      }
      (void)deadline_us;
      (void)tick_after;
      (void)echo_us;
      (void)measure_span_us;
      (void)cm;
      (void)schedule_to_finish_us;
#else
      if (g_phase2_sonar_n < PHASE2_SONAR_JOBS) {
        phase2_sonar_rec_t *r = &g_phase2_sonar[g_phase2_sonar_n];
        r->actual_start_us = actual_start_us;
        r->finish_us = finish_us;
        r->sched_wake_tick = sched_wake_tick;
        r->tick_before_delay = tick_before;
        r->tick_after_delay = tick_after;
        r->echo_us = (echo_us > 65535U) ? 65535U : (uint16_t)echo_us;
        r->measure_span_us =
            (measure_span_us > 65535U) ? 65535U : (uint16_t)measure_span_us;
        r->distance_cm = (int16_t)((cm >= 0) ? cm : 0);
        r->status = (int8_t)((cm >= 0) ? 0 : cm);
        r->delay_until_status = (int8_t)dly_st;
        r->flags = instr_flags;
        if (schedule_to_finish_us > deadline_us) {
          r->flags |= 1U; /* lattice flag — not formal DMR */
        }
        if (US_IRQ_MASK_DURING_ECHO) {
          r->flags |= 2U;
        }
        r->_rsvd = 0U;
        r->iter_ord = (job > 65535U) ? 65535U : (uint16_t)job;
        g_phase2_sonar_n++;
      } else {
        g_phase2_sonar_overflow++;
      }
#endif /* RT_DMR_FORMAL */

      prev_start_us = actual_start_us;
      prev_finish_us = finish_us;
      have_prev = 1U;
      job++;
    }
    g_obs_tick_end = osKernelGetTickCount();
#else /* !RT_WALL_OBS_ENABLE — legacy job-count campaign */
    epoch_us = us_now();
    g_phase2_sonar_epoch_us = epoch_us;

    for (job = 0U; job < PHASE2_SONAR_JOBS; job++) {
      uint32_t nominal_schedule_us;
      uint32_t actual_start_us;
      uint32_t finish_us;
      uint32_t echo_us = 0U;
      uint32_t measure_span_us = 0U;
      int32_t cm;
      uint32_t schedule_to_finish_us;

      next += period_ms;
      (void)osDelayUntil(next);

      actual_start_us = us_now();
      phase2_note_wall_start(actual_start_us);
      nominal_schedule_us = epoch_us + (job + 1U) * period_us;

      cm = us_read_distance_cm_ex(&echo_us, &measure_span_us);
      finish_us = us_now();
      schedule_to_finish_us = finish_us - nominal_schedule_us;

      if (g_phase2_sonar_n < PHASE2_SONAR_JOBS) {
        phase2_sonar_rec_t *r = &g_phase2_sonar[g_phase2_sonar_n];
        r->actual_start_us = actual_start_us;
        r->finish_us = finish_us;
        r->sched_wake_tick = 0U;
        r->tick_before_delay = 0U;
        r->tick_after_delay = 0U;
        r->echo_us = (echo_us > 65535U) ? 65535U : (uint16_t)echo_us;
        r->measure_span_us =
            (measure_span_us > 65535U) ? 65535U : (uint16_t)measure_span_us;
        r->distance_cm = (int16_t)((cm >= 0) ? cm : 0);
        r->status = (int8_t)((cm >= 0) ? 0 : cm);
        r->delay_until_status = 0;
        r->flags = 0U;
        if (schedule_to_finish_us > deadline_us) {
          r->flags |= 1U;
        }
        if (US_IRQ_MASK_DURING_ECHO) {
          r->flags |= 2U;
        }
        r->_rsvd = 0U;
        r->iter_ord = (job > 65535U) ? 65535U : (uint16_t)job;
        g_phase2_sonar_n++;
      } else {
        g_phase2_sonar_overflow++;
      }
    }
#endif /* RT_WALL_OBS_ENABLE */
  }
  g_phase2_sonar_done = 1U;
  phase2_try_dump_csv();
  for (;;) {
    osDelay(1000);
  }
#else /* !PHASE2_ENABLE — Phase 0 smoke (osDelay + per-job UART) */
  char line[128];
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
                     "[wire] t=%lu.%lu s  SIG=%d  TIM2_us=%lu\r\n",
                     (unsigned long)(i / 5U),
                     (unsigned long)((i % 5U) * 2U),
                     sig,
                     (unsigned long)us_now());
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
#endif /* PHASE2_ENABLE */
#else
  char line[128];
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
#if PHASE2_ENABLE
#if RT_HR_WHEEL_ONLY
  g_phase2_sonar_done = 1U; /* no SonarTask — allow CSV dump */
#endif
  {
    char ready[200];
    int rn = snprintf(
        ready, sizeof(ready),
#if RT_WALL_OBS_ENABLE
        "WheelTask profile=%s run=%s: TIM2 warm%lus+obs%lus @%ums; "
        "D_compute=%uus (formal D NOT approved). Quiet until dump.\r\n",
        RT_PROFILE_NAME, RT_RUN_NAME,
        (unsigned long)(RT_OBS_WARMUP_US / 1000000U),
        (unsigned long)(RT_OBS_DURATION_US / 1000000U),
#else
        "WheelTask profile=%s: %u windows @%ums; miss_compute/miss_output "
        "(D_compute=%uus after window). Quiet until dump.\r\n",
        RT_PROFILE_NAME,
        (unsigned)PHASE2_WHEEL_JOBS,
#endif
        (unsigned)PHASE2_WHEEL_PERIOD_MS,
        (unsigned)PHASE2_WHEEL_COMPUTE_DEADLINE_US);
    if (rn > 0) {
      ns_uart_transmit((const uint8_t *)ready, (uint16_t)rn);
    }
  }
  {
    const uint32_t period_ms = PHASE2_WHEEL_PERIOD_MS;
    const uint32_t period_us = period_ms * 1000U;
    const uint32_t compute_deadline_us = PHASE2_WHEEL_COMPUTE_DEADLINE_US;
    uint32_t next = osKernelGetTickCount();
    uint32_t epoch_us;
    uint32_t job = 0U;
#if RT_WALL_OBS_ENABLE
    uint32_t prev_start_us = 0U;
    uint32_t prev_finish_us = 0U;
    uint32_t have_prev = 0U;

    obs_sync_t0(2U);
    epoch_us = g_obs_t0;
    g_phase2_wheel_epoch_us = epoch_us;
    g_wheel_tick_entry = osKernelGetTickCount();
    next = g_wheel_tick_entry;
    if (next >= period_ms) {
      next -= period_ms;
    }

    for (;;) {
      uint32_t nominal_schedule_us;
      uint32_t nominal_window_end_us;
      uint32_t window_open_us;
      uint32_t s_exec_us;
      uint32_t finish_us;
      uint32_t edges0;
      uint32_t edges1;
      uint32_t pulse_count;
      uint32_t compute_us;
      int32_t schedule_output_dev_us;
      uint32_t bin;
      uint32_t sched_wake_tick;
      uint32_t tick_before;
      uint32_t tick_after;
      osStatus_t dly_st;
      uint8_t instr_flags;
      uint32_t window_elapsed_us;

      /* Measurement window open (preserved). s_exec = post-DelayUntil below. */
      window_open_us = us_now();
      if (window_open_us >= g_obs_tend) {
        break;
      }

      nominal_schedule_us = epoch_us + job * period_us;
      nominal_window_end_us = nominal_schedule_us + period_us;
      edges0 = g_wheel_edges;

      next += period_ms;
      sched_wake_tick = next;
      tick_before = osKernelGetTickCount();
      dly_st = osDelayUntil(next);
      tick_after = osKernelGetTickCount();

      s_exec_us = us_now(); /* post-DelayUntil — compute eligibility (window close) */
      if (!obs_in_window(s_exec_us)) {
#if RT_DMR_FORMAL
        {
          dmr_job_rec_t rec;
          uint8_t f = 0U;
          if ((dly_st != osOK) || (tick_before >= sched_wake_tick)) {
            f |= 1U;
          }
          if ((s_exec_us - window_open_us) + 1000U < period_us) {
            f |= 2U; /* short_window */
          }
          dmr_fill(&rec, sched_wake_tick, period_us, s_exec_us, 0U, 0,
                   window_open_us, f);
          dmr_store(g_dmr_wheel, &g_phase2_wheel_n, &g_phase2_wheel_overflow,
                    PHASE2_WHEEL_JOBS, &rec);
        }
#endif
        break;
      }

      instr_flags = 0U;
      if ((dly_st != osOK) || (tick_before >= sched_wake_tick)) {
        instr_flags |= 4U; /* immediate_wake */
        if (job == 0U) {
          g_wheel_first_immediate = 1U;
        }
      }
      window_elapsed_us = s_exec_us - window_open_us;
      /* Flag window that did not span ~P (e.g. immediate DelayUntil). */
      if (window_elapsed_us + 1000U < period_us) {
        instr_flags |= 8U; /* short_window */
        if (job == 0U) {
          g_wheel_first_short_window = 1U;
        }
      }

      bin = obs_bin_index(s_exec_us);
      g_wheel_starts++;
      if (bin < RT_OBS_BINS) {
        g_wheel_bin_start[bin]++;
      }
      if (have_prev) {
        obs_note_interval(&g_wheel_isum, &g_wheel_imax, &g_wheel_imin,
                          &g_wheel_ipairs, s_exec_us - prev_start_us);
      }

      edges1 = g_wheel_edges;
      pulse_count = edges1 - edges0;
#if RT_DMR_FORMAL
      dmr_busy_inject_delay();
#endif
      finish_us = us_now();
      compute_us = finish_us - s_exec_us;
      schedule_output_dev_us = (int32_t)(finish_us - nominal_window_end_us);

      if (obs_in_window(finish_us)) {
        uint32_t b2 = obs_bin_index(finish_us);
        g_wheel_comps++;
        if (b2 < RT_OBS_BINS) {
          g_wheel_bin_comp[b2]++;
        }
        if (have_prev) {
          obs_note_interval(&g_wheel_osum, &g_wheel_omax, &g_wheel_omin,
                            &g_wheel_opairs, finish_us - prev_finish_us);
        }
      }

#if RT_DMR_FORMAL
      {
        dmr_job_rec_t rec;
        uint8_t f = 0U;
        if (instr_flags & 4U) {
          f |= 1U;
        }
        if (instr_flags & 8U) {
          f |= 2U;
        }
        /* Q3: r = nominal scheduled window close (sched_wake_tick); D=P. */
        dmr_fill(&rec, sched_wake_tick, period_us, s_exec_us, finish_us, 1,
                 window_open_us, f);
        dmr_store(g_dmr_wheel, &g_phase2_wheel_n, &g_phase2_wheel_overflow,
                  PHASE2_WHEEL_JOBS, &rec);
      }
      (void)compute_deadline_us;
      (void)tick_after;
      (void)pulse_count;
      (void)compute_us;
      (void)schedule_output_dev_us;
#else
      if (g_phase2_wheel_n < PHASE2_WHEEL_JOBS) {
#if RT_HR_WHEEL_ONLY
        phase2_wheel_hr_rec_t *r = &g_phase2_wheel[g_phase2_wheel_n];
        r->s_exec_us = s_exec_us;
        r->finish_us = finish_us;
        r->sched_wake_tick = sched_wake_tick;
        r->iter_ord = (job > 65535U) ? 65535U : (uint16_t)job;
        r->delay_until_status = (int8_t)dly_st;
        r->flags = instr_flags;
        if (compute_us > compute_deadline_us) {
          r->flags |= 1U;
        }
        (void)window_open_us;
        (void)tick_before;
        (void)tick_after;
        (void)pulse_count;
        (void)schedule_output_dev_us;
#else
        phase2_wheel_rec_t *r = &g_phase2_wheel[g_phase2_wheel_n];
        r->actual_start_us = window_open_us;
        r->window_end_us = s_exec_us;
        r->finish_us = finish_us;
        r->sched_wake_tick = sched_wake_tick;
        r->tick_before_delay = tick_before;
        r->tick_after_delay = tick_after;
        r->pulse_count = (pulse_count > 65535U) ? 65535U : (uint16_t)pulse_count;
        r->iter_ord = (job > 65535U) ? 65535U : (uint16_t)job;
        r->delay_until_status = (int8_t)dly_st;
        r->flags = instr_flags;
        if (compute_us > compute_deadline_us) {
          r->flags |= 1U; /* existing 1 ms post-window compute check */
        }
        if (schedule_output_dev_us > (int32_t)compute_deadline_us) {
          r->flags |= 2U; /* lattice miss_output — not formal D=101ms */
        }
        r->_rsvd[0] = 0U;
        r->_rsvd[1] = 0U;
#endif
        g_phase2_wheel_n++;
      } else {
        g_phase2_wheel_overflow++;
      }
#endif /* RT_DMR_FORMAL */

      prev_start_us = s_exec_us;
      prev_finish_us = finish_us;
      have_prev = 1U;
      job++;
    }
    g_obs_tick_end = osKernelGetTickCount();
#else /* !RT_WALL_OBS_ENABLE */
    epoch_us = us_now();
    g_phase2_wheel_epoch_us = epoch_us;

    for (job = 0U; job < PHASE2_WHEEL_JOBS; job++) {
      uint32_t nominal_schedule_us;
      uint32_t nominal_window_end_us;
      uint32_t actual_start_us;
      uint32_t window_end_us;
      uint32_t finish_us;
      uint32_t edges0;
      uint32_t edges1;
      uint32_t pulse_count;
      uint32_t compute_us;
      int32_t schedule_output_dev_us;

      actual_start_us = us_now();
      phase2_note_wall_start(actual_start_us);
      nominal_schedule_us = epoch_us + job * period_us;
      nominal_window_end_us = nominal_schedule_us + period_us;
      edges0 = g_wheel_edges;

      next += period_ms;
      (void)osDelayUntil(next);

      window_end_us = us_now();
      edges1 = g_wheel_edges;
      pulse_count = edges1 - edges0;
      finish_us = us_now();
      compute_us = finish_us - window_end_us;
      schedule_output_dev_us = (int32_t)(finish_us - nominal_window_end_us);

      if (g_phase2_wheel_n < PHASE2_WHEEL_JOBS) {
#if RT_HR_WHEEL_ONLY
        /* HR profiles require RT_WALL_OBS_ENABLE; this path is unused. */
        (void)actual_start_us;
        (void)window_end_us;
        (void)finish_us;
        (void)pulse_count;
        (void)compute_us;
        (void)schedule_output_dev_us;
#else
        phase2_wheel_rec_t *r = &g_phase2_wheel[g_phase2_wheel_n];
        r->actual_start_us = actual_start_us;
        r->window_end_us = window_end_us;
        r->finish_us = finish_us;
        r->sched_wake_tick = 0U;
        r->tick_before_delay = 0U;
        r->tick_after_delay = 0U;
        r->pulse_count = (pulse_count > 65535U) ? 65535U : (uint16_t)pulse_count;
        r->iter_ord = (job > 65535U) ? 65535U : (uint16_t)job;
        r->delay_until_status = 0;
        r->flags = 0U;
        if (compute_us > compute_deadline_us) {
          r->flags |= 1U;
        }
        if (schedule_output_dev_us > (int32_t)compute_deadline_us) {
          r->flags |= 2U;
        }
        r->_rsvd[0] = 0U;
        r->_rsvd[1] = 0U;
        g_phase2_wheel_n++;
#endif
      } else {
        g_phase2_wheel_overflow++;
      }
    }
#endif /* RT_WALL_OBS_ENABLE */
  }
  g_phase2_wheel_done = 1U;
  phase2_try_dump_csv();
  for (;;) {
    osDelay(1000);
  }
#else /* !PHASE2_ENABLE */
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
    /* edges/s / 20 slots ≈ rev/s — computed RPM, not calibrated ground truth */
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
#endif /* PHASE2_ENABLE */
}
#endif /* WHEEL */

#if PHASE3_WITH_ORIGINAL_SMARM
void Phase3_Attest_Task(void *argument)
{
  (void)argument;
  uint8_t digest[32];
  uint8_t challenge[16];
  {
    char msg[160];
    int n = snprintf(msg, sizeof(msg),
                     "AttestTask: shuffle_mode=%s B=%u IRQ_HMAC=%u "
                     "(USE_SAU=%u USE_SNAP=%u). Quiet until dump.\r\n",
                     ATTEST_SHUFFLE_MODE_NAME,
                     (unsigned)ATTEST_BLOCK_SIZE,
                     (unsigned)ATTEST_IRQ_HMAC_MASK,
                     (unsigned)USE_SAU_APPROACH,
                     (unsigned)USE_SNAP);
    if (n > 0) {
      ns_uart_transmit((const uint8_t *)msg, (uint16_t)n);
    }
  }
  portALLOCATE_SECURE_CONTEXT(8192);
#if RT_DIGEST_PROBE
  /* Gate A only: one fixed-challenge attestation; not part of wall30 metrics. */
  {
    static const uint8_t k_probe_challenge[16] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10};
    memcpy(challenge, k_probe_challenge, sizeof(challenge));
    SECURE_AttestReport_t report = {
        .sau_config_avg_cycles = 0U,
        .t_disabled_avg_cycles = 0U,
        .t_disabled_max_cycles = 0U,
        .attest_phase_ptr = &g_attest_phase,
    };
    g_secure_busy = 1U;
    SECURE_ShuffledHMAC_secure(digest, challenge, sizeof(challenge), &report);
    g_secure_busy = 0U;
    {
      char line[200];
      int n = snprintf(line, sizeof(line),
                       "DIGEST_PROBE mode=%s B=%u challenge=",
                       ATTEST_SHUFFLE_MODE_NAME,
                       (unsigned)ATTEST_BLOCK_SIZE);
      if (n > 0) {
        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
      }
      for (int i = 0; i < 16; i++) {
        char hx[4];
        int hn = snprintf(hx, sizeof(hx), "%02x", challenge[i]);
        if (hn > 0) {
          ns_uart_transmit((const uint8_t *)hx, (uint16_t)hn);
        }
      }
      {
        const char mid[] = " digest=";
        ns_uart_transmit((const uint8_t *)mid, (uint16_t)(sizeof(mid) - 1U));
      }
      for (int i = 0; i < 32; i++) {
        char hx[4];
        int hn = snprintf(hx, sizeof(hx), "%02x", digest[i]);
        if (hn > 0) {
          ns_uart_transmit((const uint8_t *)hx, (uint16_t)hn);
        }
      }
      {
        char tail[96];
        int tn = snprintf(tail, sizeof(tail),
                          " tdis_avg=%lu tdis_max=%lu sau=%lu\r\n",
                          (unsigned long)report.t_disabled_avg_cycles,
                          (unsigned long)report.t_disabled_max_cycles,
                          (unsigned long)report.sau_config_avg_cycles);
        if (tn > 0) {
          ns_uart_transmit((const uint8_t *)tail, (uint16_t)tn);
        }
      }
    }
    g_phase3_attest_idle = 1U;
    for (;;) {
      osDelay(1000);
    }
  }
#endif /* RT_DIGEST_PROBE */
  /* Let Sonar/Wheel take first periods before heavy NSC work */
  osDelay(200);
#if RT_WALL_OBS_ENABLE
  /* Align attest counting to the same TIM2 T0 as sensors. */
  while (g_obs_t0 == 0U) {
    osDelay(1);
  }
#endif

  /* Do not start a new round after stop. An in-flight NSC always runs to
   * completion (counted in rounds_boundary); incomplete only if dump drain times out. */
  while (g_phase3_attest_stop == 0U) {
#if RT_WALL_OBS_ENABLE
    /* Do not begin a new NSC after the TIM2 observation cutoff. */
    if (!obs_in_window(us_now())) {
      osDelay(1);
      continue;
    }
#endif
    uint32_t seed = osKernelGetTickCount() ^ (g_phase3_attest_rounds * 0x9E3779B9U);
    for (int k = 0; k < 4; k++) {
      uint32_t rnd = seed ^ (seed << 13) ^ ((uint32_t)k * 0x5DEECE66DU);
      memcpy(&challenge[k * 4], &rnd, 4);
    }
    g_attest_phase = 0U;
    g_secure_busy = 1U;
    {
      uint32_t t0 = us_now();
      SECURE_AttestReport_t report = {
          .sau_config_avg_cycles = 0U,
          .t_disabled_avg_cycles = 0U,
          .t_disabled_max_cycles = 0U,
          .attest_phase_ptr = &g_attest_phase,
      };
      SECURE_ShuffledHMAC_secure(digest, challenge, sizeof(challenge), &report);
      uint32_t t1 = us_now();
      g_phase3_attest_last_ms = (t1 - t0) / 1000U; /* TIM2 @ 1 MHz */
      g_phase3_attest_tdis_avg = report.t_disabled_avg_cycles;
      g_phase3_attest_tdis_max = report.t_disabled_max_cycles;
      g_phase3_attest_rounds++;
#if RT_WALL_OBS_ENABLE
      if (obs_in_window(t1)) {
        g_phase3_attest_rounds_in_obs++;
      }
#endif
    }
    g_secure_busy = 0U;
  }
  g_phase3_attest_idle = 1U;
  for (;;) {
    osDelay(1000);
  }
}
#endif /* PHASE3_WITH_ORIGINAL_SMARM */
#endif /* PULSE_SMOKE_TEST */

#include <sys/time.h>
int _gettimeofday(struct timeval *tv, void *tzvp) { (void)tv; (void)tzvp; return 0; }
void SecureFault_Callback(void)
{
  const char msg[] = "\r\n[NS] SecureFault — flash Secure+NonSecure, rebuild both.\r\n";
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 500);
  (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 500);
  Error_Handler();
}
int __io_putchar(int ch)
{
  /* Direct HAL TX — must NOT call ns_uart_transmit (uart_mutex). printf is often
   * used while holding uart_mutex; nested take deadlocks FreeRTOS non-recursive mutex. */
  uint8_t c = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart3, &c, 1U, NS_UART_VCP_TIMEOUT_MS);
  return ch;
}
void SecureError_Callback(void) { Error_Handler(); }

#define TIM2_TICKS_PER_SEC  137500

static void ns_print_attest_phase(uint32_t phase)
{
  char buf[72];
  int n = 0;
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
#if TABLE_I_RUNTIME
        const char alive[] = "SMARM_Experiment_Task running (Table I runtime, no NormalTask).\r\n";
#else
        const char alive[] = "SMARM_Experiment_Task running (FAR + NormalTask 1000 Hz).\r\n";
#endif
        ns_uart_transmit((const uint8_t *)alive, (uint16_t)(sizeof(alive) - 1U));
    }
    {
        char banner[280];
        int bn = snprintf(banner, sizeof(banner),
#if TABLE_I_RUNTIME
#if ATTEST_NS_256K_REGION
            "\r\nTable I runtime shuffle=%s B=%u |M|=256KiB @0x08040000, "
            "1 pass/round, no NormalTask, HMAC IRQ-off (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE
#else
            "\r\nTable I runtime shuffle=%s B=%u |M|=512KiB @0x08000000, "
            "1 pass/round, no NormalTask, HMAC IRQ-off (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE
#endif
#elif USE_SAU_APPROACH
#if ATTEST_FAULT_DEMO
            "\r\nSMARM-Guard FAULT DEMO B=%u |M|=256KiB @0x08040000 "
            "(SAU over live NS XIP; expect fault).\r\n",
            (unsigned)ATTEST_BLOCK_SIZE
#else
            "\r\nSMARM-Guard B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @1000 Hz (SAU every block) (10 rounds).\r\n",
            (unsigned)ATTEST_BLOCK_SIZE
#endif
#elif USE_SNAP
            "\r\nSMARM-Snap B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @1000 Hz (memcpy IRQ-off) (10 rounds).\r\n",
            (unsigned)ATTEST_BLOCK_SIZE
#else
#if FAR_REF_NO_IRQ_DISABLE
#if ATTEST_NS_256K_REGION
            "\r\nSMARM FAR-REF (IRQ-ON HMAC) shuffle=%s B=%u |M|=256KiB @0x08040000, "
            "1 pass/round, FAR @%u Hz (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE, (unsigned)TARGET_FREQ_HZ
#else
            "\r\nSMARM FAR-REF (IRQ-ON HMAC) shuffle=%s B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @%u Hz (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE, (unsigned)TARGET_FREQ_HZ
#endif
#elif ATTEST_NS_256K_REGION
            "\r\nSMARM FAR/Energy shuffle=%s B=%u |M|=256KiB @0x08040000, "
            "1 pass/round, NormalTask @%u Hz, HMAC IRQ-off (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE, (unsigned)TARGET_FREQ_HZ
#elif ATTEST_TABLE_I_REGION
            /* Historical 512 KiB energy profile (preserved; not primary). */
            "\r\nSMARM Energy-pilot shuffle=%s B=%u |M|=512KiB @0x08000000, "
            "1 pass/round, NormalTask @%u Hz, HMAC IRQ-off (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE, (unsigned)TARGET_FREQ_HZ
#else
            "\r\nSMARM Baseline shuffle=%s B=%u |M|=128KiB @0x08060000, "
            "1 pass/round, FAR @%u Hz (HMAC IRQ-off) (10 rounds).\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE, (unsigned)TARGET_FREQ_HZ
#endif
#endif
            );
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
        /* Energy Table II sync markers (host timestamps Attest_Start/End ↔ FNB58). */
        {
          char mk[48];
          int mn = snprintf(mk, sizeof(mk), "Attest_Start round=%u\r\n",
                            (unsigned)(round + 1U));
          if (mn > 0 && mn < (int)sizeof(mk)) {
            ns_uart_transmit((const uint8_t *)mk, (uint16_t)mn);
          }
        }
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
        {
          char mk[48];
          int mn = snprintf(mk, sizeof(mk), "Attest_End round=%u\r\n",
                            (unsigned)(round + 1U));
          if (mn > 0 && mn < (int)sizeof(mk)) {
            ns_uart_transmit((const uint8_t *)mk, (uint16_t)mn);
          }
        }
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
        /* Use ns_uart_transmit (not printf): newlib printf can stall under FreeRTOS
         * and Round/SUMMARY would never reach the VCP capture. */
        {
            char line[220];
            int n;
#if TABLE_I_RUNTIME
            n = snprintf(line, sizeof(line),
                "Round %u: Runtime=%lu ms, NS=%lu (no NormalTask)\r\n",
                (unsigned)(round + 1U),
                (unsigned long)actual_duration_ms,
                (unsigned long)actual_run);
            if (n > 0 && n < (int)sizeof(line)) {
                ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
            }
            if (actual_run != 0U) {
                const char bad[] = "ABORT: NS activations != 0 under Table I (no NormalTask)\r\n";
                ns_uart_transmit((const uint8_t *)bad, (uint16_t)(sizeof(bad) - 1U));
                for (;;) { osDelay(10000); }
            }
#else
            if (expected_run > 0U) {
                n = snprintf(line, sizeof(line),
                    "Round %u: Runtime=%lu ms, NS=%lu/%lu activations, FAR=%lu.%04lu\r\n",
                    (unsigned)(round + 1U),
                    (unsigned long)actual_duration_ms,
                    (unsigned long)actual_run,
                    (unsigned long)expected_run,
                    (unsigned long)(far_x10000 / 10000U),
                    (unsigned long)(far_x10000 % 10000U));
            } else {
                n = snprintf(line, sizeof(line),
                    "Round %u: Runtime=%lu ms, NS=%lu/%lu activations, FAR=N/A\r\n",
                    (unsigned)(round + 1U),
                    (unsigned long)actual_duration_ms,
                    (unsigned long)actual_run,
                    (unsigned long)expected_run);
            }
            if (n > 0 && n < (int)sizeof(line)) {
                ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
            }
#endif
            if (sau_avg == 0xBAD00002U) {
                const char m[] = "SAU_config_avg_cycles=CMSE_FAIL(challenge ptr)\r\n";
                ns_uart_transmit((const uint8_t *)m, (uint16_t)(sizeof(m) - 1U));
            } else if (sau_avg == 0xBAD00001U) {
                const char m[] = "SAU_config_avg_cycles=CMSE_FAIL(digest ptr)\r\n";
                ns_uart_transmit((const uint8_t *)m, (uint16_t)(sizeof(m) - 1U));
            } else {
#if USE_SAU_APPROACH
                n = snprintf(line, sizeof(line),
                    "SAU_config_avg_cycles=%lu\r\n", (unsigned long)sau_avg);
                if (n > 0 && n < (int)sizeof(line)) {
                    ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
                }
#endif
                /* CPU = 110 MHz → time_us = cycles/110. Print µs (3 dp) or ms if ≥1 ms. */
                {
                    uint32_t avg_us_x1000 =
                        (uint32_t)(((uint64_t)tdis_avg * 1000U) / 110U);
                    uint32_t max_us_x1000 =
                        (uint32_t)(((uint64_t)tdis_max * 1000U) / 110U);
                    if (avg_us_x1000 >= 1000000U || max_us_x1000 >= 1000000U) {
                        n = snprintf(line, sizeof(line),
                            "t_disabled avg=%lu cy (%lu.%03lu ms), max=%lu cy (%lu.%03lu ms)\r\n",
                            (unsigned long)tdis_avg,
                            (unsigned long)(avg_us_x1000 / 1000000U),
                            (unsigned long)((avg_us_x1000 / 1000U) % 1000U),
                            (unsigned long)tdis_max,
                            (unsigned long)(max_us_x1000 / 1000000U),
                            (unsigned long)((max_us_x1000 / 1000U) % 1000U));
                    } else {
                        n = snprintf(line, sizeof(line),
                            "t_disabled avg=%lu cy (%lu.%03lu us), max=%lu cy (%lu.%03lu us)\r\n",
                            (unsigned long)tdis_avg,
                            (unsigned long)(avg_us_x1000 / 1000U),
                            (unsigned long)(avg_us_x1000 % 1000U),
                            (unsigned long)tdis_max,
                            (unsigned long)(max_us_x1000 / 1000U),
                            (unsigned long)(max_us_x1000 % 1000U));
                    }
                    if (n > 0 && n < (int)sizeof(line)) {
                        ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
                    }
                }
            }
        }
        osDelay(2000);
    }
    uint32_t sum = 0, min_val = durations_ms[0], max_val = durations_ms[0];
    for (int i = 0; i < 10; i++) { sum += durations_ms[i]; if (durations_ms[i] < min_val) min_val = durations_ms[i]; if (durations_ms[i] > max_val) max_val = durations_ms[i]; }
    uint32_t mean = sum / 10;
    {
        char line[192];
        int n;
#if TABLE_I_RUNTIME
#if ATTEST_NS_256K_REGION
        n = snprintf(line, sizeof(line),
            "\r\n=== SUMMARY Table I runtime shuffle=%s B=%u |M|=256KiB (10 rounds) ===\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE);
#else
        n = snprintf(line, sizeof(line),
            "\r\n=== SUMMARY Table I runtime shuffle=%s B=%u |M|=512KiB (10 rounds) ===\r\n",
            ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE);
#endif
        if (n > 0 && n < (int)sizeof(line)) {
            ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
        }
#elif USE_SAU_APPROACH
        const char hdr[] = "\r\n=== SUMMARY SMARM-Guard FAR @1000 Hz (10 rounds) ===\r\n";
        ns_uart_transmit((const uint8_t *)hdr, (uint16_t)(sizeof(hdr) - 1U));
#elif USE_SNAP
        const char hdr[] = "\r\n=== SUMMARY SMARM-Snap FAR @1000 Hz (10 rounds) ===\r\n";
        ns_uart_transmit((const uint8_t *)hdr, (uint16_t)(sizeof(hdr) - 1U));
#else
        n = snprintf(line, sizeof(line),
            "\r\n=== SUMMARY SMARM Baseline FAR @%u Hz shuffle=%s B=%u (10 rounds) ===\r\n",
            (unsigned)TARGET_FREQ_HZ, ATTEST_SHUFFLE_MODE_NAME, (unsigned)ATTEST_BLOCK_SIZE);
        if (n > 0 && n < (int)sizeof(line)) {
            ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
        }
#endif
        n = snprintf(line, sizeof(line), "Mean: %lu ms\r\n", (unsigned long)mean);
        if (n > 0 && n < (int)sizeof(line)) {
            ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
        }
        n = snprintf(line, sizeof(line), "Min:  %lu ms\r\n", (unsigned long)min_val);
        if (n > 0 && n < (int)sizeof(line)) {
            ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
        }
        n = snprintf(line, sizeof(line), "Max:  %lu ms\r\n", (unsigned long)max_val);
        if (n > 0 && n < (int)sizeof(line)) {
            ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
        }
        if (far_valid_rounds > 0U) {
            uint32_t far_mean_x10000 =
                (uint32_t)(far_sum_x10000 / far_valid_rounds);
            n = snprintf(line, sizeof(line),
                "FAR mean=%lu.%04lu, min=%lu.%04lu, max=%lu.%04lu\r\n",
                (unsigned long)(far_mean_x10000 / 10000U),
                (unsigned long)(far_mean_x10000 % 10000U),
                (unsigned long)(far_min_x10000 / 10000U),
                (unsigned long)(far_min_x10000 % 10000U),
                (unsigned long)(far_max_x10000 / 10000U),
                (unsigned long)(far_max_x10000 % 10000U));
            if (n > 0 && n < (int)sizeof(line)) {
                ns_uart_transmit((const uint8_t *)line, (uint16_t)n);
            }
        }
        {
            const char t[] = "t_disabled: DWT over each IRQ-off window (110 MHz).\r\n";
            ns_uart_transmit((const uint8_t *)t, (uint16_t)(sizeof(t) - 1U));
        }
        {
            const char t[] = "==========================================\r\n";
            ns_uart_transmit((const uint8_t *)t, (uint16_t)(sizeof(t) - 1U));
        }
        {
            const char t[] = "Done. Press RESET to run again.\r\n";
            ns_uart_transmit((const uint8_t *)t, (uint16_t)(sizeof(t) - 1U));
        }
    }
    for(;;) { osDelay(10000); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void)
{
    const char e[] = "\r\n[NS] Error_Handler (halt)\r\n";
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)e, (uint16_t)(sizeof(e) - 1U), 500);
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
