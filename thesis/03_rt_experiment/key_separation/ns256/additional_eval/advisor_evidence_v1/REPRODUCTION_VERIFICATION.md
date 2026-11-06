# Reproduction verification

Status: **PASS**.

Command, from the repository root:

```
python thesis/03_rt_experiment/key_separation/ns256/additional_eval/advisor_evidence_v1/build_advisor_evidence.py
```

The script writes only in this directory. `reproduce_post_advisor_bc.py` is executed with its output directory redirected here, so `post_advisor_bc_v1` is not rewritten. `reproduce_final_dmr.py` is executed with `OUT` redirected to `recomputed_formal_dmr.csv`, so `FINAL_DMR_RESULTS.csv` is not rewritten.

## Checks

- **FAR independent recomputation.** PASS, 69 configurations × 10 rounds
- **FAR versus post_advisor_bc_v1.** PASS, means match the existing directory
- **Sliding rules and existing tables.** PASS, synthetic gap handling matched; measured gap_excluded total 0; tables match post_advisor_bc_v1
- **Fixed formal DMR.** PASS, 264 rows match FINAL_DMR_RESULTS.csv including late, unexecuted, and right-censored counts
- **Inter-start jitter.** PASS, library matches |Δs−P|; 46 configurations have three identical files and were kept as one trace
- **Charts versus CSVs.** PASS, 384 plotted points match the CSVs written here

## Gate 3 sample comparison

The coverage matrix gives nine rounded or verbal jitter samples. The exact statistic is the mean of `|Δs − P|` after dropping the first in-window start. A sample marked MISMATCH is a difference from that rounded cell. The exact mean is kept.

- MATCH 10Hz10Hz noattest B-: published ~0; sonar=0.0 us, wheel=0.0 us
- MATCH 10Hz10Hz original B1024: published ~255; sonar=256.6 us, wheel=256.6 us
- MATCH 10Hz10Hz original B4096: published large; sonar=233418.2 us, wheel=233418.2 us
- MATCH Wheel100Hz noattest B-: published 0; wheel=0.0 us
- MATCH Wheel100Hz original B1024: published ~429; wheel=429.3 us
- MATCH Wheel100Hz original B4096: published large; wheel=26276.4 us
- MATCH Wheel1000Hz noattest B-: published 0; wheel=0.0 us
- MATCH Wheel1000Hz original B1024: published ~85; wheel=85.1 us
- MATCH Wheel1000Hz original B4096: published ~2.6k; wheel=2642.9 us

## Formulas confirmed

- FAR original: `actual / floor(runtime_ms × f / 1000)`, then the mean of ten ratios.
- FAR continuous: `actual / (f × runtime_ms / 1000)`, same mean. No clip at 1.
- Duration source: integer millisecond floor. Raw TIM2 difference is absent. Tick length `1/137500` s is unused precision, and the missing piece is under 1 ms.
- Fixed DMR: nominal id `sched_wake_tick`; late if finish `>` `r+P`; unexecuted if no row; right-censored if `d > Tend`, omitted from numerator and denominator.
- Sliding: `d = s[i-1] + 2P` only for nominally consecutive jobs with valid timestamps. First job excluded. Gap after a missing predecessor excluded. Unexecuted jobs are not successes.
- Jitter: `J = |Δs − P|` from `rt_metrics_wall30.score_regularity`, population SD (`/n`), first in-window start dropped. UART `inter_start avg` matches the mean interval, which is a different number.

## What was not done

No hardware run, no firmware edit, no manuscript edit, no Zephyr run, and no change to the frozen FAR, DMR, or Gate 3 raw files.
