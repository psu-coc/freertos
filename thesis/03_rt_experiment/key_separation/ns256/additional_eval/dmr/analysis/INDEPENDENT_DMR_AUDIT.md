# Independent Formal DMR Audit

Cells audited (campaign): **198**
Inject validation cells: **1** (excluded from campaign table)
Failures: **0**

## Instrumentation limitations
- 16 B on-device ledger; `r_us`/`d_us` derived at dump via Q1 map.
- Wheel `window_open_us` reconstructed from `s_us - before_ms*1000` (±0.5 ms).
- Right-censored jobs excluded from DMR denom; overflow invalidates capture.
- TIM2 µs + FreeRTOS tick 1000 Hz; NS RAM 96 KiB caps HR at ~3100 jobs.
- No UART/FS in hot path; post-obs dump only.

## Per-task DMR (independent recompute)

| tag | task | eligible | miss | DMR | late | incomplete | censored | valid |
|-----|------|----------|------|-----|------|------------|----------|-------|
| `dmr_10Hz10Hz_fpe_B1024_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B1024_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B1024_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B1024_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B1024_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B1024_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B128_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B2048_r1` | sonar | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B2048_r1` | wheel | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B2048_r2` | sonar | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B2048_r2` | wheel | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B2048_r3` | sonar | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B2048_r3` | wheel | 188 | 186 | 0.989362 | 185 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B256_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B256_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B256_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B256_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B256_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B256_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B4096_r1` | sonar | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B4096_r1` | wheel | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B4096_r2` | sonar | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B4096_r2` | wheel | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B4096_r3` | sonar | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B4096_r3` | wheel | 105 | 104 | 0.990476 | 103 | 1 | 0 | True |
| `dmr_10Hz10Hz_fpe_B512_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B512_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B512_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B512_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B512_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B512_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_fpe_B64_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_noattest_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B1024_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B128_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B2048_r1` | sonar | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B2048_r1` | wheel | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B2048_r2` | sonar | 165 | 163 | 0.987879 | 162 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B2048_r2` | wheel | 165 | 163 | 0.987879 | 162 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B2048_r3` | sonar | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B2048_r3` | wheel | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B256_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B256_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B256_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B256_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B256_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B256_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B4096_r1` | sonar | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B4096_r1` | wheel | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B4096_r2` | sonar | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B4096_r2` | wheel | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B4096_r3` | sonar | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B4096_r3` | wheel | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_original_B512_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B512_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B512_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B512_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B512_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B512_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_original_B64_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B1024_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B128_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B2048_r1` | sonar | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B2048_r1` | wheel | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B2048_r2` | sonar | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B2048_r2` | wheel | 166 | 164 | 0.987952 | 163 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B2048_r3` | sonar | 167 | 165 | 0.988024 | 164 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B2048_r3` | wheel | 167 | 165 | 0.988024 | 164 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B256_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B256_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B256_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B256_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B256_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B256_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B4096_r1` | sonar | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B4096_r1` | wheel | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B4096_r2` | sonar | 86 | 85 | 0.988372 | 84 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B4096_r2` | wheel | 86 | 85 | 0.988372 | 84 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B4096_r3` | sonar | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B4096_r3` | wheel | 85 | 84 | 0.988235 | 83 | 1 | 0 | True |
| `dmr_10Hz10Hz_prng_B512_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B512_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B512_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B512_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B512_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B512_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r1` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r1` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r2` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r2` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r3` | sonar | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_10Hz10Hz_prng_B64_r3` | wheel | 300 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r1` | wheel | 3000 | 1888 | 0.629333 | 1888 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r2` | wheel | 3000 | 1888 | 0.629333 | 1888 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B1024_r3` | wheel | 3000 | 1888 | 0.629333 | 1888 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B128_r1` | wheel | 3000 | 206 | 0.068667 | 206 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B128_r2` | wheel | 3000 | 206 | 0.068667 | 206 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B128_r3` | wheel | 3000 | 206 | 0.068667 | 206 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r1` | wheel | 1780 | 1778 | 0.998876 | 1777 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r2` | wheel | 1780 | 1778 | 0.998876 | 1777 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B2048_r3` | wheel | 1780 | 1778 | 0.998876 | 1777 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B256_r1` | wheel | 3000 | 695 | 0.231667 | 695 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B256_r2` | wheel | 3000 | 695 | 0.231667 | 695 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B256_r3` | wheel | 3000 | 695 | 0.231667 | 695 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r1` | wheel | 1030 | 1028 | 0.998058 | 1027 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r2` | wheel | 1030 | 1028 | 0.998058 | 1027 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B4096_r3` | wheel | 1030 | 1028 | 0.998058 | 1027 | 1 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B512_r1` | wheel | 3000 | 1297 | 0.432333 | 1297 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B512_r2` | wheel | 3000 | 1297 | 0.432333 | 1297 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B512_r3` | wheel | 3000 | 1297 | 0.432333 | 1297 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_fpe_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_fpe_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_noattest_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_noattest_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_noattest_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_noattest_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_noattest_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_noattest_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B1024_r1` | wheel | 3000 | 2667 | 0.889000 | 2666 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B1024_r2` | wheel | 3000 | 2667 | 0.889000 | 2666 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B1024_r3` | wheel | 3000 | 2667 | 0.889000 | 2666 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r1` | wheel | 3000 | 775 | 0.258333 | 774 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r2` | wheel | 3000 | 775 | 0.258333 | 774 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B128_r3` | wheel | 3000 | 775 | 0.258333 | 774 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r1` | wheel | 1643 | 1639 | 0.997565 | 1638 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r2` | wheel | 1643 | 1639 | 0.997565 | 1638 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B2048_r3` | wheel | 1643 | 1639 | 0.997565 | 1638 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B256_r1` | wheel | 3000 | 1751 | 0.583667 | 1751 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B256_r2` | wheel | 3000 | 1751 | 0.583667 | 1751 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B256_r3` | wheel | 3000 | 1751 | 0.583667 | 1751 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B4096_r1` | wheel | 842 | 839 | 0.996437 | 838 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B4096_r2` | wheel | 842 | 839 | 0.996437 | 838 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B4096_r3` | wheel | 842 | 839 | 0.996437 | 838 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r1` | wheel | 3000 | 2358 | 0.786000 | 2357 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r2` | wheel | 3000 | 2358 | 0.786000 | 2357 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B512_r3` | wheel | 3000 | 2358 | 0.786000 | 2357 | 1 | 0 | True |
| `dmr_Wheel1000Hz_original_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_original_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_original_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_prng_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B1024_r1` | wheel | 3000 | 2595 | 0.865000 | 2594 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B1024_r2` | wheel | 3000 | 2595 | 0.865000 | 2594 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B1024_r3` | wheel | 3000 | 2595 | 0.865000 | 2594 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r1` | wheel | 3000 | 302 | 0.100667 | 301 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r2` | wheel | 3000 | 302 | 0.100667 | 301 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B128_r3` | wheel | 3000 | 302 | 0.100667 | 301 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r1` | wheel | 1663 | 1661 | 0.998797 | 1660 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r2` | wheel | 1663 | 1661 | 0.998797 | 1660 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B2048_r3` | wheel | 1663 | 1661 | 0.998797 | 1660 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r1` | wheel | 3000 | 1249 | 0.416333 | 1248 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r2` | wheel | 3000 | 1249 | 0.416333 | 1248 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B256_r3` | wheel | 3000 | 1249 | 0.416333 | 1248 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r1` | wheel | 842 | 840 | 0.997625 | 839 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r2` | wheel | 842 | 840 | 0.997625 | 839 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B4096_r3` | wheel | 842 | 840 | 0.997625 | 839 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r1` | wheel | 3000 | 2120 | 0.706667 | 2119 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r2` | wheel | 3000 | 2120 | 0.706667 | 2119 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B512_r3` | wheel | 3000 | 2120 | 0.706667 | 2119 | 1 | 0 | True |
| `dmr_Wheel1000Hz_prng_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_prng_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel1000Hz_prng_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel1000Hz_prng_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B1024_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B1024_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B1024_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B128_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B128_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B128_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B2048_r1` | wheel | 1853 | 1851 | 0.998921 | 1850 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B2048_r2` | wheel | 1853 | 1851 | 0.998921 | 1850 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B2048_r3` | wheel | 1853 | 1851 | 0.998921 | 1850 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B256_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B256_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B256_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B4096_r1` | wheel | 1029 | 1028 | 0.999028 | 1027 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B4096_r2` | wheel | 1029 | 1028 | 0.999028 | 1027 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B4096_r3` | wheel | 1029 | 1028 | 0.999028 | 1027 | 1 | 0 | True |
| `dmr_Wheel100Hz_fpe_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B512_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B512_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B512_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_fpe_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_fpe_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_noattest_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_noattest_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_noattest_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_noattest_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_noattest_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_noattest_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B1024_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B1024_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B1024_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B128_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B128_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B128_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B2048_r1` | wheel | 1631 | 1629 | 0.998774 | 1628 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B2048_r2` | wheel | 1631 | 1629 | 0.998774 | 1628 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B2048_r3` | wheel | 1631 | 1629 | 0.998774 | 1628 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B256_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B256_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B256_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B4096_r1` | wheel | 829 | 828 | 0.998794 | 827 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B4096_r2` | wheel | 829 | 828 | 0.998794 | 827 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B4096_r3` | wheel | 829 | 828 | 0.998794 | 827 | 1 | 0 | True |
| `dmr_Wheel100Hz_original_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B512_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B512_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B512_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_original_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_original_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B1024_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B1024_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B1024_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B1024_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B1024_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B1024_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B128_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B128_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B128_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B128_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B128_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B128_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B2048_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B2048_r1` | wheel | 1636 | 1634 | 0.998778 | 1633 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B2048_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B2048_r2` | wheel | 1636 | 1634 | 0.998778 | 1633 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B2048_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B2048_r3` | wheel | 1636 | 1634 | 0.998778 | 1633 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B256_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B256_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B256_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B256_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B256_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B256_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B4096_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B4096_r1` | wheel | 832 | 831 | 0.998798 | 830 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B4096_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B4096_r2` | wheel | 832 | 831 | 0.998798 | 830 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B4096_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B4096_r3` | wheel | 832 | 831 | 0.998798 | 830 | 1 | 0 | True |
| `dmr_Wheel100Hz_prng_B512_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B512_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B512_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B512_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B512_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B512_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B64_r1` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B64_r1` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B64_r2` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B64_r2` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |
| `dmr_Wheel100Hz_prng_B64_r3` | sonar | 0 | 0 |  | 0 | 0 | 0 | True |
| `dmr_Wheel100Hz_prng_B64_r3` | wheel | 3000 | 0 | 0.000000 | 0 | 0 | 1 | True |

## Callouts (zero DMR and severe degradation)
- Zero-DMR task rows: **156** (reported, not hidden)
- Severe DMR (≥5%) task rows: **108**
  - `dmr_10Hz10Hz_fpe_B2048_r1` sonar: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B2048_r1` wheel: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B2048_r2` sonar: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B2048_r2` wheel: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B2048_r3` sonar: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B2048_r3` wheel: DMR=0.9894 miss=186/188
  - `dmr_10Hz10Hz_fpe_B4096_r1` sonar: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_fpe_B4096_r1` wheel: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_fpe_B4096_r2` sonar: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_fpe_B4096_r2` wheel: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_fpe_B4096_r3` sonar: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_fpe_B4096_r3` wheel: DMR=0.9905 miss=104/105
  - `dmr_10Hz10Hz_original_B2048_r1` sonar: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_original_B2048_r1` wheel: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_original_B2048_r2` sonar: DMR=0.9879 miss=163/165
  - `dmr_10Hz10Hz_original_B2048_r2` wheel: DMR=0.9879 miss=163/165
  - `dmr_10Hz10Hz_original_B2048_r3` sonar: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_original_B2048_r3` wheel: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_original_B4096_r1` sonar: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_original_B4096_r1` wheel: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_original_B4096_r2` sonar: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_original_B4096_r2` wheel: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_original_B4096_r3` sonar: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_original_B4096_r3` wheel: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_prng_B2048_r1` sonar: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_prng_B2048_r1` wheel: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_prng_B2048_r2` sonar: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_prng_B2048_r2` wheel: DMR=0.9880 miss=164/166
  - `dmr_10Hz10Hz_prng_B2048_r3` sonar: DMR=0.9880 miss=165/167
  - `dmr_10Hz10Hz_prng_B2048_r3` wheel: DMR=0.9880 miss=165/167
  - `dmr_10Hz10Hz_prng_B4096_r1` sonar: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_prng_B4096_r1` wheel: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_prng_B4096_r2` sonar: DMR=0.9884 miss=85/86
  - `dmr_10Hz10Hz_prng_B4096_r2` wheel: DMR=0.9884 miss=85/86
  - `dmr_10Hz10Hz_prng_B4096_r3` sonar: DMR=0.9882 miss=84/85
  - `dmr_10Hz10Hz_prng_B4096_r3` wheel: DMR=0.9882 miss=84/85
  - `dmr_Wheel1000Hz_fpe_B1024_r1` wheel: DMR=0.6293 miss=1888/3000
  - `dmr_Wheel1000Hz_fpe_B1024_r2` wheel: DMR=0.6293 miss=1888/3000
  - `dmr_Wheel1000Hz_fpe_B1024_r3` wheel: DMR=0.6293 miss=1888/3000
  - `dmr_Wheel1000Hz_fpe_B128_r1` wheel: DMR=0.0687 miss=206/3000
  - `dmr_Wheel1000Hz_fpe_B128_r2` wheel: DMR=0.0687 miss=206/3000
  - `dmr_Wheel1000Hz_fpe_B128_r3` wheel: DMR=0.0687 miss=206/3000
  - `dmr_Wheel1000Hz_fpe_B2048_r1` wheel: DMR=0.9989 miss=1778/1780
  - `dmr_Wheel1000Hz_fpe_B2048_r2` wheel: DMR=0.9989 miss=1778/1780
  - `dmr_Wheel1000Hz_fpe_B2048_r3` wheel: DMR=0.9989 miss=1778/1780
  - `dmr_Wheel1000Hz_fpe_B256_r1` wheel: DMR=0.2317 miss=695/3000
  - `dmr_Wheel1000Hz_fpe_B256_r2` wheel: DMR=0.2317 miss=695/3000
  - `dmr_Wheel1000Hz_fpe_B256_r3` wheel: DMR=0.2317 miss=695/3000
  - `dmr_Wheel1000Hz_fpe_B4096_r1` wheel: DMR=0.9981 miss=1028/1030
  - `dmr_Wheel1000Hz_fpe_B4096_r2` wheel: DMR=0.9981 miss=1028/1030
