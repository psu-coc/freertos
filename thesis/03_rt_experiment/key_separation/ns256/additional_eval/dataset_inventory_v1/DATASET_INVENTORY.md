# Sonar and Wheel dataset inventory

Scope: every file under `thesis/03_rt_experiment/`. The metric table uses the NS256 formal traces only. Older logs and Gate 3 timing files are classified here and are not turned into new observations.

## Measured in the NS256 formal matrix

| Arrangement | Task | Rate | Algorithms | Block sizes | Capture files | Independent timing |
|---|---|---|---|---|---|---|
| Concurrent Sonar + Wheel | Sonar and Wheel, each with its own timestamp rows | 10 Hz, period 100 ms, 30 s | Original, PRNG, FPE, and no attestation | 64 through 4096 | 3 | 3 for attested cells. No-attestation Wheel has 2 distinct timing traces; Sonar has 3. |
| Wheel only | Wheel | 100 Hz, period 10 ms, 30 s | same | same | 3 | 1 distinct timing trace per cell |
| Wheel only | Wheel | 1000 Hz, period 1 ms, 3 s | same | same | 3 | 1 distinct timing trace per cell |

Formal job files on disk: 332. Folder groups: {'additional_eval': 1, 'raw': 331}. The metric table uses the 264 rows in `FINAL_DMR_RESULTS.csv` (132 captures, Sonar and Wheel counted separately at 10 Hz). Extra formal files outside that table are not added as new observations.

Sonar timing CSVs: 183. Of those, 117 are outside `freertos_rt/WheelSonar10Hz_NS256_A` (earlier logs and checkpoint copies). The Gate 3 10 Hz campaign has the remaining 66. Files whose names pair Sonar with 100 Hz or 1000 Hz: 0. Wheel timing CSVs: 447, including repeated campaign copies.

UART header census counts files, including repeated copies. `wheel_period_ms` is blank on NS256 captures that print `P_us` instead; the profile name still gives the rate. Files with no Sonar count in the first 8 KB are runtime or other logs, not a hidden Sonar campaign: `logs` (23), `key_separation` (90), and `table_i_runtime` (21). `table_i_runtime` is the Table I runtime capture and states that it has no NormalTask.

UART census (`profile`, Sonar count, Wheel period) -> file count:

- `('10Hz10Hz', 'sonar_n>0', 'wheel_period_ms=100')`: 14
- `('10Hz10Hz', 'sonar_n>0', 'wheel_period_ms=na')`: 198
- `('Wheel1000Hz', 'sonar_n=0', 'wheel_period_ms=1')`: 9
- `('Wheel1000Hz', 'sonar_n=0', 'wheel_period_ms=na')`: 189
- `('Wheel100Hz', 'sonar_n=0', 'wheel_period_ms=10')`: 9
- `('Wheel100Hz', 'sonar_n=0', 'wheel_period_ms=na')`: 189
- `('checkpoint_copy:10Hz10Hz', 'sonar_n>0', 'wheel_period_ms=100')`: 6
- `('checkpoint_copy:10Hz10Hz', 'sonar_n>0', 'wheel_period_ms=na')`: 27
- `('checkpoint_copy:pilot_sonar_5Hz_wheel_1Hz', 'sonar_n>0', 'wheel_period_ms=na')`: 3
- `('phase4_wheel100', 'sonar_n=0', 'wheel_period_ms=10')`: 4
- `('phase4_wheel100', 'sonar_n=0', 'wheel_period_ms=na')`: 63
- `('phase4_wheel1000', 'sonar_n=0', 'wheel_period_ms=1')`: 4
- `('phase4_wheel1000', 'sonar_n=0', 'wheel_period_ms=na')`: 63
- `('pilot_sonar_5Hz_wheel_1Hz', 'sonar_n>0', 'wheel_period_ms=na')`: 3
- `('unknown', 'sonar_n=na', 'wheel_period_ms=na')`: 134

## Not measured

- Sonar-only at 10, 100, or 1000 Hz. Every nonzero Sonar count at 10 Hz is in a capture that also records Wheel. The only other nonzero Sonar count is the 5 Hz pilot below, which also records Wheel.
- Wheel-only at 10 Hz.
- Sonar at 100 Hz.
- Sonar at 1000 Hz.

Wheel 100 Hz and Wheel 1000 Hz logs, including the earlier `phase4r_w100` and `phase4r_w1000` files, print `sonar_period_ms=100` while `sonar_n=0` and the Wheel period is 10 ms or 1 ms. That period field is the unused default. It is not a Sonar dataset. The NS256 Wheel-only UART line is `SonarTask not created; no sonar rows.`

The earlier `logs/phase2*` and `logs/phase3*` files named `10Hz10Hz` are concurrent 10 Hz captures (`sonar_period_ms=100`, `wheel_period_ms=100`). They are not 100 Hz or 1000 Hz Sonar.

One older pilot is a different rate and is not part of the NS256 matrix. `logs/phase2_uart_capture.txt`, `logs/phase2_1_60s_uart_capture.txt`, and `logs/phase3_B1024_uart_capture.txt` record Sonar at 200 ms (5 Hz) and a Wheel window of 1000 ms (1 Hz). The first of those is a 20-job smoke. The 60 s files have 300 Sonar jobs and 60 Wheel windows. Copies also sit under `checkpoints/`. That pilot is not Sonar at 100 Hz and not Sonar at 1000 Hz. It has no row in `FINAL_DMR_RESULTS.csv`, so this audit does not invent a formal DMR for it.

## Two NS256 capture families

Formal DMR and the sliding window use `additional_eval/dmr/raw/dmr_10Hz10Hz_*` and `additional_eval/dmr/recapture/raw/dmr_Wheel*`. Gate 3 frequency and jitter files live under `freertos_rt/WheelSonar10Hz_NS256_A`, `Wheel100Hz_NS256_A`, and `Wheel1000Hz_NS256_A`. A Gate 3 file is not an extra formal-DMR repetition. The three identical high-rate formal files are one timing trace.

## Answers

1. Wheel at 10 Hz has a complete per-block-size formal DMR and sliding result for every attested algorithm and for no attestation, taken from Wheel's own rows in the concurrent capture. There is no separate Wheel-only 10 Hz experiment.
2. Sonar at 100 Hz does not exist as a measurement. No filename, nonzero Sonar count, or Sonar timing CSV at that rate is in this tree.
3. Sonar at 1000 Hz does not exist as a measurement, on the same evidence.
