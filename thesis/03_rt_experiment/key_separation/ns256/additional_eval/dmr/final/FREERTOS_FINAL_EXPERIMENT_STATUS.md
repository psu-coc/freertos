# FreeRTOS NS256 formal DMR status

Verdict: **PASS WITH DOCUMENTED LIMITATIONS**.

No configuration is blocked. No further FreeRTOS hardware run is required to support the descriptive results in `FINAL_DMR_TABLES.md`.

## What was finalized

The authoritative job-level results are:

- Concurrent Sonar and Wheel at 10 Hz, from `additional_eval/dmr/raw/dmr_10Hz10Hz_*`.
- Wheel at 100 Hz and Wheel at 1000 Hz, from `additional_eval/dmr/recapture/raw/`, using the measured `tick0_tim2_us` anchor.

Both use the same deadline rule. Relative deadline equals the period. Each iteration is scored against the release stored for that iteration. Eligible releases that the loop never recorded are misses. Releases whose deadlines fall after Tend are excluded.

Coverage is 198 captures and 88 configurations: no attestation, and Original, PRNG, and FPE at seven block sizes, for each profile above. Sonar and Wheel are separate 10 Hz results. The recomputation is `FINAL_DMR_RESULTS.csv`.

## What a paper can state

These statements match the measured counts.

- With D = P, no-attestation DMR is 0 on Sonar 10 Hz, Wheel 10 Hz, Wheel 100 Hz, and Wheel 1000 Hz.
- At 10 Hz, Original, PRNG, and FPE stay at DMR 0 through B = 1024. At B = 2048 the DMR is 0.993333. At B = 4096 it is 0.996667. Sonar and Wheel were scored separately and these run-1 values agree. Three distinct captures support the 10 Hz DMR. A few captures move one job between the late and unexecuted counts without changing the DMR.
- At 100 Hz, DMR is 0 through B = 1024. At B = 2048 it is 0.999333. At B = 4096 it is 0.999667.
- At 1000 Hz, with the measured tick0 anchor, DMR is 0 through B = 1024, including Original B1024. At B = 2048 and B = 4096 the DMR is 0.998666 to 0.999333, as listed per mode in the table.
- The high block sizes are dominated by iterations the periodic loop never executed, plus late completions of the iterations it did run.

## What a paper has to disclose

- Wheel 100 Hz and Wheel 1000 Hz are one timing trace per configuration, retained as three capture files. They are not three independent outcomes, and they do not support a confidence interval.
- The Wheel 100 Hz no-attestation boot numbers are 2, 2, and 4. The counter does not identify a session across a flash.
- The 10 Hz release is the logged T0 map. Those captures do not contain the later systick-edge anchor line.
- The Wheel denominator is 2999 because the tick0 anchor right-censors the final period.

## What this status leaves untouched

The manuscript was not edited. Frozen Energy, Runtime, FAR, and the frozen Wheel and Sonar corpora were not used as inputs and were not modified. Zephyr, T = 3, and T = 7 were not touched. No board was flashed or reset for this report.

A later experiment would be a new decision if a paper needs statistically independent Wheel repetitions. It is not required for the descriptive DMR above, and it was not started.
