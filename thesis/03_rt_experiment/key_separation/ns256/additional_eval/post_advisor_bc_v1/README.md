# Post-advisor FAR and sliding-reference results (v1)

Derived from existing NS256 logs. This directory does not replace `dmr/final/` or the FAR aggregate CSVs.

## Reproduce

From the repository root:

```
python thesis/03_rt_experiment/key_separation/ns256/additional_eval/post_advisor_bc_v1/reproduce_post_advisor_bc.py
```

The script only writes files in this directory. It reads:

- `freertos_rt/FarSweep_NS256_{10,100,1000}Hz_A/analysis/*_rounds.csv`
- `additional_eval/dmr/raw/dmr_10Hz10Hz_*` and `additional_eval/dmr/recapture/raw/dmr_Wheel*`
- `additional_eval/dmr/final/FINAL_DMR_RESULTS.csv` (fixed DMR, read-only)

## Outputs

| File | Contents |
|---|---|
| `far_rounds_continuous.csv` | Every round: original FAR and continuous FAR |
| `far_comparison.csv` | Mean per configuration, including FAR-REF |
| `far_continuous_10hz.png`, `far_continuous_100hz.png`, `far_continuous_1000hz.png` | Bars are continuous FAR. Dots are the original FAR. Diamonds are FAR-REF. |
| `sliding_reference_results.csv` | Every capture and task |
| `sliding_comparison_run1.csv` | Rows marked `table_row=yes`, excluding no-attestation |
| `sliding_worked_examples.csv` | Timestamp examples at Wheel 100 Hz, B = 1024, 2048, 4096 |
| `sliding_vs_fixed_run1.png` | Run 1 only |
| `METHODOLOGY_AND_LIMITATIONS.md` | Definitions, checks, and limits |

Continuous FAR is not a nominal-job fulfillment ratio. The sliding ratio is not formal DMR.
