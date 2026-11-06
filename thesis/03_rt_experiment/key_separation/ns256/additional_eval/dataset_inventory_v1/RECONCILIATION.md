# Reconciliation with previous reports

## Statements checked

The exact sentences "No Sonar-only 100 Hz trace", "No Sonar-only 1000 Hz trace", and "Wheel 10 Hz data exists, but per-block-size results must be extracted from CSV" are not in `advisor_report/SMARMplus_FreeRTOS_Advisor_Report.html`.

What that HTML does say matches the files:

- 10 Hz is Sonar and Wheel together.
- 100 Hz and 1000 Hz are Wheel alone.
- Wheel 10 Hz DMR values are present in the page script. The detail-row object is assigned from Sonar with `dmr["Wheel 10 Hz"].rows = dmr["Sonar 10 Hz"].rows`.

That assignment is a display alias. The CSV already has Wheel's own rows. Across all three runs, 0 concurrent cells differ between Sonar and Wheel in fixed DMR, late count, unexecuted count, or sliding count. The timestamp rows are not copies: in the Original B = 1024 run 1 formal file, Sonar job 0 finishes at 2032451 µs and Wheel job 0 finishes at 2030509 µs.

## Classification

| Case | Finding |
|---|---|
| Experiment not performed | Sonar-only at any rate; Wheel-only at 10 Hz; Sonar at 100 Hz; Sonar at 1000 Hz. |
| Experiment performed but raw trace missing | None for the NS256 formal matrix. Every FINAL_DMR row points at a formal CSV that is present. |
| Raw trace exists but derived metric not calculated | Gate 3 timing CSVs are frequency and jitter captures. They are not the formal job-identity DMR source. The formal traces do have both DMR and the one-period sliding window. |
| Metric available but omitted from a report | The first advisor HTML has no sliding-window table. Those ratios are in `advisor_evidence_v1/sliding_reference_results.csv` and `sliding_window_revalidation_v1/sliding_window_vs_prior.csv`. Wheel 10 Hz per-B DMR was not missing from the CSV or from the HTML chart. |

## Joins

- `FINAL_DMR_RESULTS.csv`: 264 task-runs.
- `sliding_reference_results.csv`: same keys; predecessor-window misses match `sliding_miss`.
- `sliding_window_jobs.csv`: task values present: 10Hz10Hz:sonar, 10Hz10Hz:wheel, Wheel1000Hz:wheel, Wheel100Hz:wheel. No Sonar row is tagged 100 Hz or 1000 Hz.

Sliding evaluated count uses `window_start = previous actual start + P` and `window_end = window_start + P`. It is not a different deadline.
