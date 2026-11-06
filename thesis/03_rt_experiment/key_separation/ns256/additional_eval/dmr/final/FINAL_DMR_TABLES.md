# Formal DMR results

Post-KDF NS256, T=1, relative deadline D = P. Each stored iteration keeps the release named by its `sched_wake_tick`. A completion is on time when its own finish is at or before that release plus P. An eligible release with no stored iteration is an unexecuted miss. A release whose deadline is after Tend is right-censored and is out of both the numerator and the denominator.

DMR is printed to six decimal places. The counts are exact. `miss = late + unexecuted + incomplete`.

Hashes and paths for every capture are in `FINAL_DMR_RESULTS.csv` (`source_csv`, `csv_sha256`, `uart_sha256`, `timing_sha256`).

Traces means distinct job-timing hashes. Files means capture files retained on disk.

## A. Concurrent Sonar and Wheel, 10 Hz

Source: `additional_eval/dmr/raw/dmr_10Hz10Hz_*`. Observation 30 s. Period 100 ms. Eligible obligations: 300. The three capture files of each configuration are distinct. Run 1 is shown. Where another run moves one job between late and unexecuted, the DMR is unchanged and the other composition is in the footnote.

### Sonar

| Mode | B | Eligible | On time | Late | Unexecuted | Incomplete | Censored | DMR | Traces |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| No attestation | — | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 2048 | 300 | 2 | 163 | 134 | 1 | 0 | 0.993333 | 3 |
| Original | 4096 | 300 | 1 | 83 | 215 | 1 | 0 | 0.996667 | 3 |
| PRNG | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 2048 | 300 | 2 | 163 | 134 | 1 | 0 | 0.993333 | 3 |
| PRNG | 4096 | 300 | 1 | 83 | 215 | 1 | 0 | 0.996667 | 3 |
| FPE | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 2048 | 300 | 2 | 185 | 112 | 1 | 0 | 0.993333 | 3 |
| FPE | 4096 | 300 | 1 | 103 | 195 | 1 | 0 | 0.996667 | 3 |

### Wheel

| Mode | B | Eligible | On time | Late | Unexecuted | Incomplete | Censored | DMR | Traces |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| No attestation | — | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 2 |
| Original | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| Original | 2048 | 300 | 2 | 163 | 134 | 1 | 0 | 0.993333 | 3 |
| Original | 4096 | 300 | 1 | 83 | 215 | 1 | 0 | 0.996667 | 3 |
| PRNG | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| PRNG | 2048 | 300 | 2 | 163 | 134 | 1 | 0 | 0.993333 | 3 |
| PRNG | 4096 | 300 | 1 | 83 | 215 | 1 | 0 | 0.996667 | 3 |
| FPE | 64 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 128 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 256 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 512 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 1024 | 300 | 300 | 0 | 0 | 0 | 1 | 0.000000 | 3 |
| FPE | 2048 | 300 | 2 | 185 | 112 | 1 | 0 | 0.993333 | 3 |
| FPE | 4096 | 300 | 1 | 103 | 195 | 1 | 0 | 0.996667 | 3 |

No-attestation Wheel has three UART files and two Wheel timing traces: runs 2 and 3 share one Wheel timing hash, and run 1 differs. Sonar in those same files has three timing traces. DMR is 0.000000 on every one of them.

These run-1 counts move by one job in another trace. The miss total, and therefore the DMR, stays the same.

| Configuration | Other trace |
|---|---|
| Sonar and Wheel, Original B2048 | Run 2: on time 2, late 162, unexecuted 135, incomplete 1 |
| Sonar and Wheel, PRNG B2048 | Run 3: on time 2, late 164, unexecuted 133, incomplete 1 |
| Sonar and Wheel, PRNG B4096 | Run 2: on time 1, late 84, unexecuted 214, incomplete 1 |

## B. Wheel 100 Hz

Source: `additional_eval/dmr/recapture/raw/dmr_Wheel100Hz_*`. Observation 30 s. Period 10 ms. Eligible obligations: 2999. Three capture files are retained for every configuration. Each configuration has one timing trace, so the table reports run 1 and does not treat the three files as three outcomes.

| Mode | B | Eligible | On time | Late | Unexecuted | Incomplete | Censored | DMR | Traces |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| No attestation | — | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 256 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 512 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 1024 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 2048 | 2999 | 2 | 1628 | 1368 | 1 | 0 | 0.999333 | 1 |
| Original | 4096 | 2999 | 1 | 826 | 2171 | 1 | 0 | 0.999667 | 1 |
| PRNG | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 256 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 512 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 1024 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 2048 | 2999 | 2 | 1632 | 1364 | 1 | 0 | 0.999333 | 1 |
| PRNG | 4096 | 2999 | 1 | 827 | 2170 | 1 | 0 | 0.999667 | 1 |
| FPE | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 256 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 512 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 1024 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 2048 | 2999 | 2 | 1850 | 1146 | 1 | 0 | 0.999333 | 1 |
| FPE | 4096 | 2999 | 1 | 1030 | 1967 | 1 | 0 | 0.999667 | 1 |

The no-attestation boot counters are 2, 2, and 4. See `DMR_LIMITATIONS.md`.

## C. Wheel 1000 Hz

Source: `additional_eval/dmr/recapture/raw/dmr_Wheel1000Hz_*`. Observation 3 s. Period 1 ms. Eligible obligations: 2999. Three capture files, one timing trace, run 1 reported.

| Mode | B | Eligible | On time | Late | Unexecuted | Incomplete | Censored | DMR | Traces |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| No attestation | — | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| Original | 256 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| Original | 512 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| Original | 1024 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| Original | 2048 | 2999 | 4 | 1636 | 1358 | 1 | 0 | 0.998666 | 1 |
| Original | 4096 | 2999 | 3 | 840 | 2155 | 1 | 0 | 0.999000 | 1 |
| PRNG | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| PRNG | 256 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| PRNG | 512 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| PRNG | 1024 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| PRNG | 2048 | 2999 | 3 | 1653 | 1342 | 1 | 0 | 0.999000 | 1 |
| PRNG | 4096 | 2999 | 2 | 843 | 2153 | 1 | 0 | 0.999333 | 1 |
| FPE | 64 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 128 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 256 | 2999 | 2999 | 0 | 0 | 0 | 2 | 0.000000 | 1 |
| FPE | 512 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| FPE | 1024 | 2999 | 2999 | 0 | 0 | 0 | 1 | 0.000000 | 1 |
| FPE | 2048 | 2999 | 2 | 1761 | 1235 | 1 | 0 | 0.999333 | 1 |
| FPE | 4096 | 2999 | 2 | 1027 | 1969 | 1 | 0 | 0.999333 | 1 |
