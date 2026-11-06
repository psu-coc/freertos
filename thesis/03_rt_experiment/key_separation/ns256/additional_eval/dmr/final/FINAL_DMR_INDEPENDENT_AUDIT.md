# Independent audit of the final DMR tables

Verdict: **PASS WITH DOCUMENTED LIMITATIONS**.

No configuration is BLOCKED. The limitations are the repetition and anchor disclosures in `DMR_LIMITATIONS.md`. They change how the Wheel numbers may be generalized. They do not change the counts recomputed from the raw rows.

The audit script is `reproduce_final_dmr.py`. It does not import the campaign analyzer and it does not read `RECAPTURE_SUMMARY.csv`, `corrected_summary.csv`, or the stored-row campaign summary. After it wrote `FINAL_DMR_RESULTS.csv`, the Wheel run-1 counts were compared with `recapture/raw/*/dmr_*_dmr_analysis.json`. All 44 Wheel run-1 count tuples matched. That comparison is a check, not the source of the table.

## Coverage

| Set | Captures | Tasks | Result rows | Configurations |
|---|---:|---:|---:|---:|
| Sonar + Wheel 10 Hz | 66 | Sonar and Wheel | 132 | 44 |
| Wheel 100 Hz | 66 | Wheel | 66 | 22 |
| Wheel 1000 Hz | 66 | Wheel | 66 | 22 |
| Total | 198 |  | 264 | 88 |

Each profile has the no-attestation control and Original, PRNG, and FPE at B = 64, 128, 256, 512, 1024, 2048, and 4096, with three capture files. The injection cell is excluded. Blocked rows: 0.

## Checks

| Check | Result |
|---|---|
| Configuration coverage | 88 of 88 configurations, three files each |
| Timing anchors | 10 Hz uses the logged T0 release. Wheel uses `systick_edge` and `tick0_tim2_us`. Every Wheel span is 30,000,000 µs or 3,000,000 µs |
| Nominal release | `r = origin + (sched_wake_tick - tick0) * 1000`, `d = r + P`. Logged `r_us` and `d_us` matched this formula on every row (`r_mis = 0`) |
| Job identity | The finish stays on the `sched_wake_tick` that the iteration stored. Wake ticks advance by exactly one period (`w_step_bad = 0`) |
| Deadline comparison | On time if `f_us <= d`. Late if `f_us > d`. Recomputed from timestamps |
| Missing obligations | A lattice release with `d <= Tend` and no row is an unexecuted miss and is inside the denominator |
| Censoring | `d > Tend` is excluded from the numerator and the denominator. Logged rows in that region are the censored count |
| Duplicate matching | No repeated `sched_wake_tick` (`dup = 0`) |
| Per-task separation | Sonar and Wheel are scored from their own rows. At 10 Hz the run-1 DMR values agree; the tasks are still separate results |
| Raw provenance | Each row records the formal-CSV path and the SHA-256 of that CSV, of the UART file, and of the timing string |
| Capture validity | No fault, overflow, identity, denominator, or span failure in the recomputation |
| Boot counter | Wheel 100 Hz no-attestation boots are 2, 2, and 4. Documented. Not used as proof of a new timing outcome |
| Repetitions | 10 Hz timing hashes differ across the three files, except Wheel no-attestation runs 2 and 3, which share one Wheel timing hash. Both Wheel rates have one timing hash per configuration |
| Arithmetic | For every row, `eligible = on time + late + unexecuted + incomplete` and `miss` is the last three of those terms |

## What this audit does not pass

It does not pass a claim of three statistically independent Wheel timing outcomes. All 44 Wheel configurations have `n_distinct_timing = 1`. The three files remain in `recapture/raw/`.

It does not pass the obsolete stored-row DMR or the slot-based recovery. Those files are unchanged and are not the tables in this directory.
