# SMARM+ Gate 3 — Cross-RTOS Scientific Review (FreeRTOS primary, Zephyr TIM2-corrected support)

**Status:** READ-ONLY review. No firmware, experiment, evidence, Thesis, or manuscript files were modified except this new report.

**Date:** 2026-11-06 (review generation)

## Scope and non-goals

- FreeRTOS Sonar/Wheel audited Gate 3 results are the **primary** evaluation.
- Zephyr TIM2-corrected Gate 3 is **supporting portability evidence** only.
- Seek **same qualitative trend**, not numerical equality or RTOS superiority.
- Zephyr runtime DMR ledger is **not** compared to FreeRTOS Fixed Formal DMR.
- Zephyr Fixed Formal DMR remains **QUARANTINED**.
- SysTick-based historical Zephyr Gate 3 is excluded.

## Data provenance

### FreeRTOS (primary)

| Artifact | Path |
|---|---|
| Audited inter-start jitter | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\key_separation\ns256\additional_eval\advisor_evidence_v1\jitter_summary.csv` |
| Reproduction verification | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\key_separation\ns256\additional_eval\advisor_evidence_v1\REPRODUCTION_VERIFICATION.md` |
| COMPLETE_TASK_METRICS | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\key_separation\ns256\additional_eval\dataset_inventory_v1\COMPLETE_TASK_METRICS.csv` |
| FINAL_DMR_RESULTS (Fixed Formal) | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\key_separation\ns256\additional_eval\dmr\final\FINAL_DMR_RESULTS.csv` |
| Metric definition | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\scripts\rt_metrics_wall30.py` |
| Dual10 aggregates (supporting) | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\analysis\wall30_bsweep*\aggregate_by_config.csv` |
| Wheel100 aggregates (supporting) | `c:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\thesis\03_rt_experiment\analysis\wheel100_m3\aggregate_by_config.csv` |

Primary jitter definition (FreeRTOS): mean of `|Δs − P|` after **dropping the first in-window start** (`score_regularity`).

### Zephyr (supporting, TIM2-corrected only)

| Artifact | Path |
|---|---|
| Campaign report | `D:\smarm\SMARMPlus_Archive_20261105\Research_Evidence\Gate3_TIM2_Corrected_20261106\GATE3_TIM2_CORRECTED_REPORT.md` |
| Evidence root | `D:\smarm\SMARMPlus_Archive_20261105\Research_Evidence\Gate3_TIM2_Corrected_20261106` |
| HEAD (report) | `306b85bbb6ca5f64e12e1b9a7ba13abbdc3a01e4` |
| Timebase | TIM2 @ 1 MHz |
| Warm-up / NS256 | 2 s; attestation `0x08040000–0x0807FFFF` |

Firmware-reported Zephyr jitter: mean absolute inter-start over emitted consecutive starts (**includes first in-window pair**). This review also reports a **labeled** primary-like recomputation that drops the first in-window start from raw `RT_REC` stamps without modifying source evidence.

## Phase A — Configuration match

| Axis | FreeRTOS (audited Gate 3) | Zephyr TIM2-corrected | Match |
|---|---|---|---|
| Modes | Original, PRNG, FPE | Original, PRNG, FPE | YES |
| Block sizes B | 64…4096 (×2) | 64…4096 (×2) | YES |
| Dual Sonar/Wheel | 10 Hz, 30 s | profile=1, period 100000 µs, 30 s | YES |
| Wheel-only 100 Hz | 30 s | profile=2, period 10000 µs, 30 s | YES |
| Wheel-only 1000 Hz | 3 s | profile=3, period 1000 µs, 3 s | YES |
| Warm-up | 2 s (audited campaign) | 2 s (`warmup_us=2000000`) | YES |
| Attestation region | NS256 | NS256 `0x08040000–0x0807FFFF` | YES |
| Timestamp source | TIM2 (audited captures) | TIM2 1 MHz | YES |
| Cells accepted | 63 Mode×B×Profile (excl. noattest) | 63/63 PASS | YES |
| Repetition | typically 3 runs; some Wheel1000 single distinct timing trace | **1 run per cell** | PARTIAL |
| Record index vs release id | FreeRTOS Formal DMR uses `sched_wake_tick` / nominal ids | Zephyr sequential `RT_REC index` ≠ invented nominal ids for gaps | DISTINGUISHED |

**UART RT_OBS checks:** all 63 cells have TIM2, expected warmup/duration/period (84 sensor recompute entries).

### Reference cross-checks (Original)

| Cell | FreeRTOS audited (µs) | Expected | Zephyr report (µs) | Expected |
|---|---:|---:|---:|---:|
| Dual10 Original B1024 | 256.6 | 256.6 (MATCH) | 277 | 277 (MATCH) |
| Dual10 Original B4096 | 233418.2 | 233418.2 (MATCH) | 204979 | 204979 (MATCH) |
| Wheel1000 Original B4096 | 2642.9 | 2642.9 (MATCH) | 3080 | 3080 (MATCH) |

Note: user prompt cited Zephyr Dual10 B1024 ≈282 µs and Dual10 B4096 ≈205112 µs as focused/stress examples; the TIM2-corrected report values are **277 µs** and **204979 µs**. Wheel1000 B4096 **3080 µs** matches.

## Phase B — Metric policy used in this review

1. FreeRTOS availability/coverage from Fixed Formal DMR ledger means: `(ontime+late+incomplete)/eligible` averaged over runs (wheel task).
2. Zephyr availability/coverage: `executed/nominal` from the TIM2-corrected report (executed excludes boundary row where applicable).
3. FreeRTOS jitter: audited primary mean from `jitter_summary.csv`.
4. Zephyr jitter (campaign): firmware `mean_abs_interstart_us`.
5. Zephyr jitter (review-only comparable): drop first in-window start on raw stamps; labeled `z_j_primary_like`.
6. Missing/incomplete/right-censored preserved; no fabricated Zephyr nominal identities.
7. No Formal DMR numerical comparison FreeRTOS↔Zephyr.

## Phase C — Full 63-cell matrix

Coverage = executed/opportunity. Δj_fw = Zephyr firmware jitter − FreeRTOS primary jitter (µs). Δj_pri = Zephyr primary-like − FreeRTOS primary (µs). Δcov = coverage percentage-point difference (Z−F).

| Mode | B | Profile | FR exec/elig | FR cov | Z exec/nom | Z cov | FR j (µs) | Z j_fw | Z j_pri | Δj_fw | Δj_pri | Δcov pp |
|---|---:|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| original | 64 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 24.4 | 24.0 | 24.9 | -0.4 | 0.6 | -0.3 |
| original | 64 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 10.5 | 11.0 | 11.2 | 0.5 | 0.6 | -0.0 |
| original | 64 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 11.3 | 992.0 | 993.0 | 980.7 | 981.7 | -0.0 |
| original | 128 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 49.8 | 51.0 | 51.1 | 1.2 | 1.3 | -0.3 |
| original | 128 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 49.3 | 38.0 | 38.4 | -11.3 | -10.9 | -0.0 |
| original | 128 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 36.5 | 992.0 | 993.0 | 955.5 | 956.5 | -0.0 |
| original | 256 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 105.1 | 88.0 | 88.2 | -17.1 | -16.9 | -0.3 |
| original | 256 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 84.6 | 70.0 | 70.9 | -14.6 | -13.7 | -0.0 |
| original | 256 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 58.5 | 992.0 | 993.0 | 933.5 | 934.5 | -0.0 |
| original | 512 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 161.3 | 115.0 | 115.7 | -46.3 | -45.6 | -0.3 |
| original | 512 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 170.4 | 23.0 | 23.1 | -147.4 | -147.3 | -0.0 |
| original | 512 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 74.4 | 993.0 | 993.0 | 918.6 | 918.6 | -0.0 |
| original | 1024 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 256.6 | 277.0 | 278.0 | 20.4 | 21.4 | -0.3 |
| original | 1024 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 429.3 | 390.0 | 390.6 | -39.3 | -38.7 | -0.0 |
| original | 1024 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 85.1 | 993.0 | 993.0 | 907.9 | 907.9 | -0.0 |
| original | 2048 | Dual10 | 165.7/300.0 | 55.2% | 192/300 | 64.0% | 74733.0 | 55962.0 | 55970.1 | -18771.0 | -18762.9 | 8.8 |
| original | 2048 | Wheel100 | 1631.0/2999.0 | 54.4% | 1914/3000 | 63.8% | 8424.7 | 5671.0 | 5671.8 | -2753.7 | -2752.9 | 9.4 |
| original | 2048 | Wheel1000 | 1641.0/2999.0 | 54.7% | 1921/3000 | 64.0% | 874.9 | 1569.0 | 1569.6 | 694.1 | 694.6 | 9.3 |
| original | 4096 | Dual10 | 85.0/300.0 | 28.3% | 98/300 | 32.7% | 233418.2 | 204979.0 | 204939.3 | -28439.2 | -28478.9 | 4.3 |
| original | 4096 | Wheel100 | 828.0/2999.0 | 27.6% | 969/3000 | 32.3% | 26276.4 | 20935.0 | 20934.6 | -5341.4 | -5341.8 | 4.7 |
| original | 4096 | Wheel1000 | 844.0/2999.0 | 28.1% | 977/3000 | 32.6% | 2642.9 | 3080.0 | 3080.0 | 437.1 | 437.1 | 4.4 |
| prng | 64 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 6.7 | 3 | 4.0 | -3.7 | -2.7 | -0.3 |
| prng | 64 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 6.4 | 3 | 3.8 | -3.4 | -2.6 | 0.0 |
| prng | 64 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 6.8 | 993.0 | 993.3 | 986.2 | 986.5 | 0.0 |
| prng | 128 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 29.3 | 21.0 | 21.2 | -8.3 | -8.1 | -0.3 |
| prng | 128 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 32.7 | 14.0 | 14.2 | -18.7 | -18.5 | 0.0 |
| prng | 128 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 36.7 | 993.0 | 993.0 | 956.3 | 956.3 | 0.0 |
| prng | 256 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 84.4 | 58.0 | 58.3 | -26.4 | -26.1 | -0.3 |
| prng | 256 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 66.6 | 100.0 | 100.3 | 33.4 | 33.8 | -0.0 |
| prng | 256 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 116.8 | 992.0 | 992.7 | 875.2 | 875.9 | -0.0 |
| prng | 512 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 171.2 | 133.0 | 133.5 | -38.2 | -37.7 | -0.3 |
| prng | 512 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 216.2 | 96.0 | 96.7 | -120.2 | -119.5 | -0.0 |
| prng | 512 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 160.0 | 992.0 | 992.4 | 832.0 | 832.5 | 0.0 |
| prng | 1024 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 417.2 | 277.0 | 277.5 | -140.2 | -139.7 | -0.3 |
| prng | 1024 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 231.8 | 363.0 | 363.2 | 131.2 | 131.4 | -0.0 |
| prng | 1024 | Wheel1000 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 66.1 | 992.0 | 992.5 | 925.9 | 926.3 | -0.0 |
| prng | 2048 | Dual10 | 166.3/300.0 | 55.4% | 193/300 | 64.3% | 73524.0 | 55153.0 | 55166.3 | -18371.0 | -18357.7 | 8.9 |
| prng | 2048 | Wheel100 | 1635.0/2999.0 | 54.5% | 1925/3000 | 64.2% | 8347.8 | 5582.0 | 5582.7 | -2765.8 | -2765.1 | 9.6 |
| prng | 2048 | Wheel1000 | 1657.0/2999.0 | 55.3% | 1907/3000 | 63.6% | 919.5 | 1572.0 | 1571.6 | 652.5 | 652.1 | 8.3 |
| prng | 4096 | Dual10 | 85.3/300.0 | 28.4% | 98/300 | 32.7% | 231434.8 | 203789.0 | 203818.7 | -27645.8 | -27616.1 | 4.2 |
| prng | 4096 | Wheel100 | 829.0/2999.0 | 27.6% | 977/3000 | 32.6% | 26136.2 | 20699.0 | 20697.8 | -5437.2 | -5438.3 | 4.9 |
| prng | 4096 | Wheel1000 | 846.0/2999.0 | 28.2% | 993/3000 | 33.1% | 2675.3 | 3036.0 | 3032.8 | 360.7 | 357.5 | 4.9 |
| fpe | 64 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 7.3 | 3 | 3.6 | -4.3 | -3.7 | -0.3 |
| fpe | 64 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 8.2 | 3 | 3.4 | -5.2 | -4.8 | 0.0 |
| fpe | 64 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 5.2 | 993.0 | 993.8 | 987.8 | 988.6 | 0.0 |
| fpe | 128 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 27.4 | 10.0 | 10.6 | -17.4 | -16.8 | -0.3 |
| fpe | 128 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 27.2 | 10.0 | 10.6 | -17.2 | -16.7 | 0.0 |
| fpe | 128 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 25.3 | 993.0 | 993.8 | 967.7 | 968.6 | 0.0 |
| fpe | 256 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 40.8 | 34.0 | 34.5 | -6.8 | -6.3 | -0.3 |
| fpe | 256 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 61.5 | 39.0 | 39.1 | -22.5 | -22.3 | 0.0 |
| fpe | 256 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 93.9 | 993.0 | 993.8 | 899.1 | 899.9 | 0.0 |
| fpe | 512 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 212.2 | 113.0 | 114.0 | -99.2 | -98.2 | -0.3 |
| fpe | 512 | Wheel100 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 129.5 | 65.0 | 65.7 | -64.5 | -63.8 | 0.0 |
| fpe | 512 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 87.4 | 993.0 | 993.6 | 905.6 | 906.2 | 0.0 |
| fpe | 1024 | Dual10 | 300.0/300.0 | 100.0% | 299/300 | 99.7% | 500.4 | 126.0 | 126.6 | -374.4 | -373.8 | -0.3 |
| fpe | 1024 | Wheel100 | 2999.0/2999.0 | 100.0% | 2999/3000 | 100.0% | 454.2 | 21.0 | 21.1 | -433.2 | -433.1 | -0.0 |
| fpe | 1024 | Wheel1000 | 2999.0/2999.0 | 100.0% | 3000/3000 | 100.0% | 423.5 | 993.0 | 993.4 | 569.5 | 569.9 | 0.0 |
| fpe | 2048 | Dual10 | 188.0/300.0 | 62.7% | 229/300 | 76.3% | 54839.4 | 30790.0 | 30793.9 | -24049.4 | -24045.5 | 13.7 |
| fpe | 2048 | Wheel100 | 1853.0/2999.0 | 61.8% | 2287/3000 | 76.2% | 6208.8 | 3112.0 | 3111.8 | -3096.8 | -3097.0 | 14.4 |
| fpe | 2048 | Wheel1000 | 1764.0/2999.0 | 58.8% | 2294/3000 | 76.5% | 1071.8 | 1306.0 | 1306.1 | 234.2 | 234.3 | 17.6 |
| fpe | 4096 | Dual10 | 105.0/300.0 | 35.0% | 140/300 | 46.7% | 169940.8 | 113639.0 | 113642.7 | -56301.8 | -56298.1 | 11.7 |
| fpe | 4096 | Wheel100 | 1032.0/2999.0 | 34.4% | 1385/3000 | 46.2% | 19200.1 | 11646.0 | 11647.1 | -7554.1 | -7553.0 | 11.8 |
| fpe | 4096 | Wheel1000 | 1030.0/2999.0 | 34.3% | 1381/3000 | 46.0% | 2387.4 | 2171.0 | 2171.3 | -216.4 | -216.1 | 11.7 |

### Availability trend (selected anchors)

| Mode | Profile | FR cov@1024 | Z cov@1024 | FR cov@2048 | Z cov@2048 | FR cov@4096 | Z cov@4096 |
|---|---|---:|---:|---:|---:|---:|---:|
| original | Dual10 | 100.0% | 99.7% | 55.2% | 64.0% | 28.3% | 32.7% |
| original | Wheel100 | 100.0% | 100.0% | 54.4% | 63.8% | 27.6% | 32.3% |
| original | Wheel1000 | 100.0% | 100.0% | 54.7% | 64.0% | 28.1% | 32.6% |
| prng | Dual10 | 100.0% | 99.7% | 55.4% | 64.3% | 28.4% | 32.7% |
| prng | Wheel100 | 100.0% | 100.0% | 54.5% | 64.2% | 27.6% | 32.6% |
| prng | Wheel1000 | 100.0% | 100.0% | 55.3% | 63.6% | 28.2% | 33.1% |
| fpe | Dual10 | 100.0% | 99.7% | 62.7% | 76.3% | 35.0% | 46.7% |
| fpe | Wheel100 | 100.0% | 100.0% | 61.8% | 76.2% | 34.4% | 46.2% |
| fpe | Wheel1000 | 100.0% | 100.0% | 58.8% | 76.5% | 34.3% | 46.0% |

### Inter-start jitter trend (selected anchors, µs)

| Mode | Profile | FR@1024 | Z_fw@1024 | Z_pri@1024 | FR@2048 | Z_fw@2048 | FR@4096 | Z_fw@4096 | Z_pri@4096 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| original | Dual10 | 256.6 | 277.0 | 278.0 | 74733.0 | 55962.0 | 233418.2 | 204979.0 | 204939.3 |
| original | Wheel100 | 429.3 | 390.0 | 390.6 | 8424.7 | 5671.0 | 26276.4 | 20935.0 | 20934.6 |
| original | Wheel1000 | 85.1 | 993.0 | 993.0 | 874.9 | 1569.0 | 2642.9 | 3080.0 | 3080.0 |
| prng | Dual10 | 417.2 | 277.0 | 277.5 | 73524.0 | 55153.0 | 231434.8 | 203789.0 | 203818.7 |
| prng | Wheel100 | 231.8 | 363.0 | 363.2 | 8347.8 | 5582.0 | 26136.2 | 20699.0 | 20697.8 |
| prng | Wheel1000 | 66.1 | 992.0 | 992.5 | 919.5 | 1572.0 | 2675.3 | 3036.0 | 3032.8 |
| fpe | Dual10 | 500.4 | 126.0 | 126.6 | 54839.4 | 30790.0 | 169940.8 | 113639.0 | 113642.7 |
| fpe | Wheel100 | 454.2 | 21.0 | 21.1 | 6208.8 | 3112.0 | 19200.1 | 11646.0 | 11647.1 |
| fpe | Wheel1000 | 423.5 | 993.0 | 993.4 | 1071.8 | 1306.0 | 2387.4 | 2171.0 | 2171.3 |

## Phase C — Scientific observations

1. **B≤1024:** Both RTOS keep near-full execution coverage on Dual10 and Wheel100 for Original/PRNG/FPE. Jitter remains comparatively modest on Dual10/Wheel100 (tens–hundreds of µs), with Mode-dependent magnitude.
2. **B=2048 and B=4096:** Both RTOS show clear **execution-rate collapse** and large inter-start jitter growth on all three profiles. This is the dominant shared qualitative signature.
3. **Mode ordering at stress:** FPE typically retains higher coverage / lower stress jitter than Original/PRNG on both RTOS (e.g., Dual10 B4096), consistent with lighter attestation work — supporting mode-effect portability, not equality.
4. **Wheel1000 low-B anomaly (expected structural):** Zephyr firmware jitter ≈992–993 µs across B≤1024 while FreeRTOS primary is often ≪100 µs (Original B1024 ≈85 µs). This matches prior tick/scheduling residual analysis and is **not** treated as high-B stress agreement failure.
5. **Jitter definition gap:** FreeRTOS primary drops first in-window start; Zephyr firmware summary includes it. Primary-like recomputation usually changes Zephyr means only slightly at high N; large absolute RTOS gaps at stress remain.
6. **wall30_bsweep aggregates ≠ advisor jitter_summary** for some Dual10 cells (e.g. Original B1024 ~455 µs in wall30 vs audited 256.6 µs). This review uses **advisor_evidence_v1** as FreeRTOS primary, matching REPRODUCTION_VERIFICATION.
7. **Repetition:** FreeRTOS multi-run means vs Zephyr n=1 cell captures — independence is weaker on Zephyr; treat as supporting, not confirmatory statistics.

## Phase D — Separate scientific verdicts

### 1. DATA INTEGRITY

- FreeRTOS: advisor reproduction verification **PASS**; Fixed Formal DMR 264 rows; jitter library confirmed.
- Zephyr: TIM2-corrected campaign **63/63 PASS**; manifests PASS; one failed historical capture retained unused.
- **Verdict: PASS** for use as evidence corpora (with Zephyr Formal DMR still quarantined).

### 2. EXPERIMENT CONFIGURATION MATCH

- Mode × B × Profile × warm-up × NS256 × TIM2 × observation lengths match.
- Differences: run repetition; jitter startup sampling; Zephyr kernel-tick release vs FreeRTOS scheduling details; record index vs nominal release identity.
- **Verdict: MATCH for workload design; PARTIAL for statistics/sampling.**

### 3. EXECUTION AVAILABILITY TREND

- Shared pattern: full/near-full at B≤1024; severe loss at 2048/4096 on Dual10, Wheel100, Wheel1000 for all three modes.
- Absolute coverage at stress differs (e.g. Original Dual10 B4096 FreeRTOS ~28–30% class vs Zephyr 98/300≈32.7% executed/nominal — same regime).
- **Verdict: SAME QUALITATIVE TREND.**

### 4. INTER-START JITTER TREND

- Dual10 / Wheel100: modest at B≤1024, explosive at 2048/4096 — same qualitative trend.
- Wheel1000: high-B rise agrees; low-B magnitude disagrees due to Zephyr ~1 ms residual.
- **Verdict: SAME QUALITATIVE TREND for Dual10 & Wheel100; PARTIAL AGREEMENT for Wheel1000.**

### 5. FORMAL DMR COMPARABILITY

- FreeRTOS Fixed Formal DMR audited and available.
- Zephyr Fixed Formal DMR **QUARANTINED**; runtime ledger fields are not Formal DMR.
- **Verdict: NOT COMPARABLE / QUARANTINED — do not mark PASS.**

### 6. OVERALL SUPPORTING PORTABILITY EVIDENCE

- Zephyr TIM2-corrected Gate 3 **supports** the FreeRTOS-primary claim that attestation block size drives workload availability and inter-start regularity degradation at large B, across Original/PRNG/FPE and three profiles.
- Does **not** support numerical portability, RTOS ranking, or Formal DMR portability.

### Mode × Profile classification

| Mode | Profile | Classification | Note |
|---|---|---|---|
| original | Dual10 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| original | Wheel100 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| original | Wheel1000 | **PARTIAL AGREEMENT** | Wheel1000 low-B Zephyr jitter ~tick residual (~992 µs) vs FreeRTOS near-period lock; availability trend aligns; low-B jitter not comparable in magnitude |
| prng | Dual10 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| prng | Wheel100 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| prng | Wheel1000 | **PARTIAL AGREEMENT** | Wheel1000 low-B Zephyr jitter ~tick residual (~992 µs) vs FreeRTOS near-period lock; availability trend aligns; low-B jitter not comparable in magnitude |
| fpe | Dual10 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| fpe | Wheel100 | **SAME QUALITATIVE TREND** | stable B<=1024; degrade at 2048/4096 on both RTOS |
| fpe | Wheel1000 | **PARTIAL AGREEMENT** | Wheel1000 low-B Zephyr jitter ~tick residual (~992 µs) vs FreeRTOS near-period lock; availability trend aligns; low-B jitter not comparable in magnitude |

## Claims supported for SMARM+ manuscript / Thesis

**Supported (carefully worded):**

1. FreeRTOS remains the primary Gate 3 evaluation platform.
2. On FreeRTOS, Sonar/Wheel workloads remain available with modest inter-start jitter for B≤1024 under concurrent attestation, and show strong degradation at B=2048/4096.
3. An independent Zephyr TIM2-corrected replication of the same Mode×B×Profile matrix exhibits the **same qualitative availability and high-B jitter degradation trend**, supporting limited portability of the workload observation — not of absolute timings.
4. Mode effects (FPE typically milder stress impact than Original/PRNG at large B) appear on both RTOS.

**Not supported / must not claim:**

1. Numerical equality of jitter or coverage between FreeRTOS and Zephyr.
2. Zephyr Formal DMR results, or FreeRTOS↔Zephyr Formal DMR equivalence.
3. RTOS performance superiority.
4. Universal portability beyond this board/workload/TIM2 setup.
5. Wheel1000 low-B jitter parity.

## Remaining uncertainties

1. Zephyr n=1 vs FreeRTOS multi-run — no Zephyr repetition CI.
2. Startup-pair inclusion differs unless using review-only primary-like column.
3. Dual10 wall30 aggregate archive disagrees with advisor jitter means — prefer advisor path.
4. Zephyr unrecorded opportunities lack FreeRTOS-style nominal release IDs.
5. Formal DMR on Zephyr still needs a separate methodology review before any paper claim.

## Final verdict

**SUPPORTING PORTABILITY EVIDENCE: YES (qualitative).**

The TIM2-corrected Zephyr Gate 3 campaign is sufficient for a supporting, FreeRTOS-primary scientific statement of shared qualitative RT workload trends under SMARM+ attestation. It is **not** sufficient for Formal DMR portability or numerical cross-RTOS claims. No additional Zephyr raw cells are required for this qualitative review; optional upgrades would be multi-run Zephyr repetition and a quarantined Formal DMR methodology review.

---

*Generated by `_build_gate3_cross_rtos_review.py` from frozen FreeRTOS advisor evidence and Zephyr TIM2-corrected archive. Source metrics unchanged.*
