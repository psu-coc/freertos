# Final FreeRTOS NS256 DMR tables

Offline report for the post-KDF, T=1, NS256 formal deadline-miss ratio. The script only reads frozen captures and writes `FINAL_DMR_RESULTS.csv`.

## Authoritative inputs

| Profile | Raw records | Anchor |
|---|---|---|
| Concurrent Sonar + Wheel, 10 Hz, 30 s | `../raw/dmr_10Hz10Hz_*` | Logged TIM2 release `r = T0 + (sched_wake_tick - tick0) * 1000`. These captures have no `tick0_tim2` line. |
| Wheel 100 Hz, 30 s | `../recapture/raw/dmr_Wheel100Hz_*` | `DMR_ANCHOR method=systick_edge`, release origin `tick0_tim2_us` |
| Wheel 1000 Hz, 3 s | `../recapture/raw/dmr_Wheel1000Hz_*` | Same systick-edge anchor |

The original Wheel 100 Hz and Wheel 1000 Hz trees under `../raw/` are not inputs. The slot-based recovery under `../recovery/` is not an input. The delay-injection cell `dmr_10Hz10Hz_noattest_inj150000_r1` is not an input.

## Reproduce

From the repository root:

```text
python thesis/03_rt_experiment/key_separation/ns256/additional_eval/dmr/final/reproduce_final_dmr.py
```

The script rewrites `FINAL_DMR_RESULTS.csv` in this directory. Every run keeps its own `source_csv`, `csv_sha256`, `uart_sha256`, and `timing_sha256`. A configuration whose timing hashes differ is marked `table_row=yes` on every run. A configuration with one timing hash is marked `table_row=yes` on run 1 and `sibling` on runs 2 and 3.

DMR in the CSV is `miss / eligible` printed to six decimal places. The integer counts are exact.

## Documents

- `FINAL_DMR_TABLES.md` — publication tables
- `FINAL_DMR_INDEPENDENT_AUDIT.md` — audit of this recomputation
- `DMR_LIMITATIONS.md` — limits that stay attached to the numbers
- `FREERTOS_FINAL_EXPERIMENT_STATUS.md` — what this evidence does and does not support
