/**
 * Selectable dual-sensor RT experiment profile (NonSecure).
 *
 * Preserve prior captures:
 *   - 5Hz+1Hz: logs/phase2_1_* , phase3_B*_ (no 10Hz10Hz)
 *   - 10Hz job-count: logs/phase2_10Hz10Hz_* , phase3_10Hz10Hz_B*_
 *   - wall60: logs/phase2w_* , phase3w_*
 * New campaign: RT_RUN_WALL_OBS = 2s warm-up + 30s TIM2 obs (wall30), logs phase2r_/phase3r_.
 *
 * Does not alter Secure attestation algorithms — only NS task periods / stop rule.
 */
#ifndef RT_EXPERIMENT_PROFILE_H
#define RT_EXPERIMENT_PROFILE_H

#define RT_PROFILE_LEGACY_5HZ_1HZ       0  /* Sonar 200 ms, Wheel 1000 ms */
#define RT_PROFILE_10HZ_10HZ            1  /* Sonar 100 ms, Wheel 100 ms — validated corpus */
#define RT_PROFILE_WHEEL_100HZ          2  /* Wheel-only 10 ms; Sonar off — HR family */
#define RT_PROFILE_WHEEL_1000HZ_WALL3   3  /* Wheel-only 1 ms; 3 s obs — NOT wall30 */

#ifndef RT_SENSOR_PROFILE
#define RT_SENSOR_PROFILE  RT_PROFILE_WHEEL_1000HZ_WALL3
#endif

#define RT_RUN_FULL      0  /* stop after N jobs — legacy job-count campaign */
#define RT_RUN_SMOKE     1  /* short: 20 jobs/task */
#define RT_RUN_WALL_OBS  2  /* TIM2 warm-up + fixed wall observation */

#ifndef RT_RUN_LEN
#define RT_RUN_LEN  RT_RUN_WALL_OBS
#endif

#if (RT_SENSOR_PROFILE != RT_PROFILE_LEGACY_5HZ_1HZ) && \
    (RT_SENSOR_PROFILE != RT_PROFILE_10HZ_10HZ) && \
    (RT_SENSOR_PROFILE != RT_PROFILE_WHEEL_100HZ) && \
    (RT_SENSOR_PROFILE != RT_PROFILE_WHEEL_1000HZ_WALL3)
#error RT_SENSOR_PROFILE invalid
#endif

#if (RT_RUN_LEN != RT_RUN_FULL) && (RT_RUN_LEN != RT_RUN_SMOKE) && \
    (RT_RUN_LEN != RT_RUN_WALL_OBS)
#error RT_RUN_LEN must be FULL, SMOKE, or WALL_OBS
#endif

/* Fixed wall observation (independent of RTOS tick). */
#define RT_OBS_WARMUP_US       (2U * 1000000U)
#define RT_OBS_BINS            30U
/* Wheel: existing 1 ms post-window compute check. Formal D NOT advisor-approved. */
#define RT_WHEEL_D_COMPUTE_US  1000U

#if RT_SENSOR_PROFILE == RT_PROFILE_WHEEL_100HZ
/* HR: Wheel-primary 100 Hz wall30. Sonar not created (echo cannot sustain 100 Hz).
 * Compact 16 B Wheel records; 3100 cap covers 3000 nominal + margin.
 * Distinct from validated 10Hz10Hz dual-task — new log tag family phase4r_w100_*. */
#define RT_HR_WHEEL_ONLY           1
#define RT_PROFILE_NAME            "Wheel100Hz"
#define RT_SONAR_PERIOD_MS         100U /* unused when Sonar off */
#define RT_WHEEL_PERIOD_MS         10U
#define RT_SONAR_DEADLINE_MS       RT_SONAR_PERIOD_MS
#define RT_WHEEL_COMPUTE_DEADLINE_US  RT_WHEEL_D_COMPUTE_US
#define RT_OBS_DURATION_US         (30U * 1000000U)
#define RT_OBS_PERIOD_US           10000U
#define RT_OBS_NOMINAL_JOBS        3000U
#define RT_OBS_REC_CAP             3100U
#define RT_WALL_TARGET_US          RT_OBS_DURATION_US
#define RT_SONAR_D_US              RT_OBS_PERIOD_US
#define RT_WHEEL_D_US              (RT_OBS_PERIOD_US + RT_WHEEL_D_COMPUTE_US)
#if RT_RUN_LEN == RT_RUN_SMOKE
#define RT_SONAR_JOBS              1U
#define RT_WHEEL_JOBS              50U
#define RT_RUN_NAME                "smoke50"
#else
#define RT_SONAR_JOBS              1U /* dummy slot; Sonar task not created */
#define RT_WHEEL_JOBS              RT_OBS_REC_CAP
#define RT_RUN_NAME                "wall30"
#endif

#elif RT_SENSOR_PROFILE == RT_PROFILE_WHEEL_1000HZ_WALL3
/* HR probe: 1 kHz × 3 s ≈ 3000 jobs (RAM-feasible). NOT comparable to wall30.
 * SysTick period == P (1 ms) — expect fragile DelayUntil; report honestly. */
#define RT_HR_WHEEL_ONLY           1
#define RT_PROFILE_NAME            "Wheel1000Hz"
#define RT_SONAR_PERIOD_MS         100U
#define RT_WHEEL_PERIOD_MS         1U
#define RT_SONAR_DEADLINE_MS       RT_SONAR_PERIOD_MS
#define RT_WHEEL_COMPUTE_DEADLINE_US  RT_WHEEL_D_COMPUTE_US
#define RT_OBS_DURATION_US         (3U * 1000000U)
#define RT_OBS_PERIOD_US           1000U
#define RT_OBS_NOMINAL_JOBS        3000U
#define RT_OBS_REC_CAP             3100U
#define RT_WALL_TARGET_US          RT_OBS_DURATION_US
#define RT_SONAR_D_US              RT_OBS_PERIOD_US
#define RT_WHEEL_D_US              (RT_OBS_PERIOD_US + RT_WHEEL_D_COMPUTE_US)
#if RT_RUN_LEN == RT_RUN_SMOKE
#define RT_SONAR_JOBS              1U
#define RT_WHEEL_JOBS              100U
#define RT_RUN_NAME                "smoke100"
#else
#define RT_SONAR_JOBS              1U
#define RT_WHEEL_JOBS              RT_OBS_REC_CAP
#define RT_RUN_NAME                "wall3"
#endif

#elif RT_SENSOR_PROFILE == RT_PROFILE_10HZ_10HZ
#define RT_HR_WHEEL_ONLY           0
#define RT_PROFILE_NAME            "10Hz10Hz"
#define RT_SONAR_PERIOD_MS         100U
#define RT_WHEEL_PERIOD_MS         100U
#define RT_SONAR_DEADLINE_MS       RT_SONAR_PERIOD_MS
#define RT_WHEEL_COMPUTE_DEADLINE_US  RT_WHEEL_D_COMPUTE_US
#define RT_OBS_DURATION_US         (30U * 1000000U)
#define RT_OBS_PERIOD_US           100000U
#define RT_OBS_NOMINAL_JOBS        300U
#define RT_OBS_REC_CAP             360U
#define RT_WALL_TARGET_US          RT_OBS_DURATION_US
#define RT_SONAR_D_US              RT_OBS_PERIOD_US
#define RT_WHEEL_D_US              (RT_OBS_PERIOD_US + RT_WHEEL_D_COMPUTE_US)
#if RT_RUN_LEN == RT_RUN_SMOKE
#define RT_SONAR_JOBS              20U
#define RT_WHEEL_JOBS              20U
#define RT_RUN_NAME                "smoke20"
#elif RT_RUN_LEN == RT_RUN_WALL_OBS
#define RT_SONAR_JOBS              RT_OBS_REC_CAP
#define RT_WHEEL_JOBS              RT_OBS_REC_CAP
#define RT_RUN_NAME                "wall30"
#else
#define RT_SONAR_JOBS              600U
#define RT_WHEEL_JOBS              600U
#define RT_RUN_NAME                "60s"
#endif
#else /* LEGACY_5HZ_1HZ */
#define RT_HR_WHEEL_ONLY           0
#define RT_PROFILE_NAME            "5Hz1Hz"
#define RT_SONAR_PERIOD_MS         200U
#define RT_WHEEL_PERIOD_MS         1000U
#define RT_SONAR_DEADLINE_MS       RT_SONAR_PERIOD_MS
#define RT_WHEEL_COMPUTE_DEADLINE_US  RT_WHEEL_D_COMPUTE_US
#define RT_OBS_DURATION_US         (30U * 1000000U)
#define RT_OBS_PERIOD_US           100000U
#define RT_OBS_NOMINAL_JOBS        300U
#define RT_OBS_REC_CAP             360U
#define RT_WALL_TARGET_US          RT_OBS_DURATION_US
#define RT_SONAR_D_US              RT_OBS_PERIOD_US
#define RT_WHEEL_D_US              (RT_OBS_PERIOD_US + RT_WHEEL_D_COMPUTE_US)
#if RT_RUN_LEN == RT_RUN_SMOKE
#define RT_SONAR_JOBS              20U
#define RT_WHEEL_JOBS              20U
#define RT_RUN_NAME                "smoke20"
#elif RT_RUN_LEN == RT_RUN_WALL_OBS
#define RT_SONAR_JOBS              RT_OBS_REC_CAP
#define RT_WHEEL_JOBS              RT_OBS_REC_CAP
#define RT_RUN_NAME                "wall30"
#else
#define RT_SONAR_JOBS              300U
#define RT_WHEEL_JOBS              60U
#define RT_RUN_NAME                "60s"
#endif
#endif

#define RT_WALL_OBS_ENABLE  (RT_RUN_LEN == RT_RUN_WALL_OBS)

#ifndef RT_HR_WHEEL_ONLY
#define RT_HR_WHEEL_ONLY  0
#endif

/* Isolated Gate-A digest probe (fixed challenge + UART DIGEST_PROBE line).
 * Keep 0 for all wall30 performance / Stage-3 campaign builds. */
#ifndef RT_DIGEST_PROBE
#define RT_DIGEST_PROBE  0
#endif

/* Formal DMR ledger (author Q1–Q5). Keep 0 for frozen RT/FAR/Energy builds. */
#ifndef RT_DMR_FORMAL
#define RT_DMR_FORMAL  0
#endif
/* Validation only: busy-wait after wake before work (µs). 0 = off. */
#ifndef RT_DMR_INJECT_DELAY_US
#define RT_DMR_INJECT_DELAY_US  0
#endif

#endif /* RT_EXPERIMENT_PROFILE_H */
