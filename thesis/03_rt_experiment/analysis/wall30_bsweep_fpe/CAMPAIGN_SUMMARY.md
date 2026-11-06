# wall30 B-sweep aggregate

Primary David jitter: **excludes first in-window sample** (and its interval).
Frequency: **all** in-window starts. Formal DMR: **QUARANTINED**.

Per-run rows found: 21 / 24 expected.

| config | n | sonar Hz mean±SD | wheel Hz mean±SD | sonar J mean±SD (us) | tick_delta mean±SD |
|--------|---|------------------|------------------|----------------------|--------------------|
| B64 | 3 | 10.000±0.000 | 10.000±0.000 | 3.9±0.3 | 30000±0 |
| B128 | 3 | 10.000±0.000 | 10.000±0.000 | 20.5±0.7 | 30000±0 |
| B256 | 3 | 10.000±0.000 | 10.000±0.000 | 33.9±0.5 | 30000±0 |
| B512 | 3 | 10.000±0.000 | 10.000±0.000 | 95.3±2.1 | 30000±0 |
| B1024 | 3 | 10.000±0.000 | 10.000±0.000 | 411.9±2.2 | 30000±0 |
| B2048 | 3 | 6.100±0.000 | 6.100±0.000 | 64322.7±84.2 | 18300±0 |
| B4096 | 3 | 3.400±0.000 | 3.400±0.000 | 195575.3±278.2 | 10200±0 |

Plots: fig_B_vs_freq.png, fig_B_vs_jitter.png, fig_B_vs_jitter_p95.png, fig_B_vs_tick_delta.png, fig_B_vs_tdis.png

## Per-run tags present
- `phase3r_fpe_B64_r1`: sonar_n=300 Hz=10.000 Jmean=3.6 (sens=3.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B64_r2`: sonar_n=300 Hz=10.000 Jmean=4.2 (sens=4.2) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B64_r3`: sonar_n=300 Hz=10.000 Jmean=3.9 (sens=3.9) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B128_r1`: sonar_n=300 Hz=10.000 Jmean=21.0 (sens=21.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B128_r2`: sonar_n=300 Hz=10.000 Jmean=19.7 (sens=19.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B128_r3`: sonar_n=300 Hz=10.000 Jmean=20.7 (sens=20.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B256_r1`: sonar_n=300 Hz=10.000 Jmean=33.4 (sens=33.3) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B256_r2`: sonar_n=300 Hz=10.000 Jmean=33.9 (sens=33.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B256_r3`: sonar_n=300 Hz=10.000 Jmean=34.5 (sens=34.4) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B512_r1`: sonar_n=300 Hz=10.000 Jmean=93.0 (sens=94.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B512_r2`: sonar_n=300 Hz=10.000 Jmean=95.9 (sens=97.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B512_r3`: sonar_n=300 Hz=10.000 Jmean=96.9 (sens=98.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B1024_r1`: sonar_n=300 Hz=10.000 Jmean=411.0 (sens=412.2) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B1024_r2`: sonar_n=300 Hz=10.000 Jmean=414.4 (sens=415.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B1024_r3`: sonar_n=300 Hz=10.000 Jmean=410.2 (sens=411.4) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_fpe_B2048_r1`: sonar_n=183 Hz=6.100 Jmean=64240.6 (sens=64222.7) tick_delta=18300 first_imm=1/1 short_win=1
- `phase3r_fpe_B2048_r2`: sonar_n=183 Hz=6.100 Jmean=64409.0 (sens=64390.1) tick_delta=18300 first_imm=1/1 short_win=1
- `phase3r_fpe_B2048_r3`: sonar_n=183 Hz=6.100 Jmean=64318.6 (sens=64300.3) tick_delta=18300 first_imm=1/1 short_win=1
- `phase3r_fpe_B4096_r1`: sonar_n=102 Hz=3.400 Jmean=195588.6 (sens=195593.4) tick_delta=10200 first_imm=1/1 short_win=1
- `phase3r_fpe_B4096_r2`: sonar_n=102 Hz=3.400 Jmean=195290.7 (sens=195298.5) tick_delta=10200 first_imm=1/1 short_win=1
- `phase3r_fpe_B4096_r3`: sonar_n=102 Hz=3.400 Jmean=195846.7 (sens=195891.1) tick_delta=10200 first_imm=1/1 short_win=1
