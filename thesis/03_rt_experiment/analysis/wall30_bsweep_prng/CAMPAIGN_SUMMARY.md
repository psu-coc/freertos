# wall30 B-sweep aggregate

Primary David jitter: **excludes first in-window sample** (and its interval).
Frequency: **all** in-window starts. Formal DMR: **QUARANTINED**.

Per-run rows found: 21 / 24 expected.

| config | n | sonar Hz mean±SD | wheel Hz mean±SD | sonar J mean±SD (us) | tick_delta mean±SD |
|--------|---|------------------|------------------|----------------------|--------------------|
| B64 | 3 | 10.000±0.000 | 10.000±0.000 | 11.5±1.6 | 30000±0 |
| B128 | 3 | 10.000±0.000 | 10.000±0.000 | 39.1±2.3 | 30000±0 |
| B256 | 3 | 10.000±0.000 | 10.000±0.000 | 81.5±2.6 | 30000±0 |
| B512 | 3 | 10.000±0.000 | 10.000±0.000 | 233.8±2.4 | 30000±0 |
| B1024 | 3 | 10.000±0.000 | 10.000±0.000 | 441.2±9.5 | 30000±0 |
| B2048 | 3 | 5.467±0.000 | 5.467±0.000 | 83058.8±107.2 | 16400±0 |
| B4096 | 3 | 2.800±0.000 | 2.800±0.000 | 259900.4±653.4 | 8400±0 |

Plots: fig_B_vs_freq.png, fig_B_vs_jitter.png, fig_B_vs_jitter_p95.png, fig_B_vs_tick_delta.png, fig_B_vs_tdis.png

## Per-run tags present
- `phase3r_prng_B64_r1`: sonar_n=300 Hz=10.000 Jmean=11.7 (sens=11.7) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B64_r2`: sonar_n=300 Hz=10.000 Jmean=9.8 (sens=9.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B64_r3`: sonar_n=300 Hz=10.000 Jmean=13.1 (sens=13.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B128_r1`: sonar_n=300 Hz=10.000 Jmean=40.0 (sens=39.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B128_r2`: sonar_n=300 Hz=10.000 Jmean=40.9 (sens=40.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B128_r3`: sonar_n=300 Hz=10.000 Jmean=36.5 (sens=36.4) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B256_r1`: sonar_n=300 Hz=10.000 Jmean=81.9 (sens=81.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B256_r2`: sonar_n=300 Hz=10.000 Jmean=83.9 (sens=83.7) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B256_r3`: sonar_n=300 Hz=10.000 Jmean=78.7 (sens=78.5) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B512_r1`: sonar_n=300 Hz=10.000 Jmean=231.8 (sens=231.5) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B512_r2`: sonar_n=300 Hz=10.000 Jmean=236.4 (sens=236.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B512_r3`: sonar_n=300 Hz=10.000 Jmean=233.2 (sens=232.9) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B1024_r1`: sonar_n=300 Hz=10.000 Jmean=431.7 (sens=430.7) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B1024_r2`: sonar_n=300 Hz=10.000 Jmean=450.7 (sens=449.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B1024_r3`: sonar_n=300 Hz=10.000 Jmean=441.3 (sens=440.3) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_prng_B2048_r1`: sonar_n=164 Hz=5.467 Jmean=83082.1 (sens=83086.9) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_prng_B2048_r2`: sonar_n=164 Hz=5.467 Jmean=83152.6 (sens=83156.9) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_prng_B2048_r3`: sonar_n=164 Hz=5.467 Jmean=82941.9 (sens=82947.5) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_prng_B4096_r1`: sonar_n=84 Hz=2.800 Jmean=259500.9 (sens=259500.2) tick_delta=8400 first_imm=1/1 short_win=1
- `phase3r_prng_B4096_r2`: sonar_n=84 Hz=2.800 Jmean=260654.5 (sens=260639.9) tick_delta=8400 first_imm=1/1 short_win=1
- `phase3r_prng_B4096_r3`: sonar_n=84 Hz=2.800 Jmean=259545.9 (sens=259544.6) tick_delta=8400 first_imm=1/1 short_win=1
