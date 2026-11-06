# wall30 B-sweep aggregate

Primary David jitter: **excludes first in-window sample** (and its interval).
Frequency: **all** in-window starts. Formal DMR: **QUARANTINED**.

Per-run rows found: 24 / 24 expected.

| config | n | sonar Hz mean±SD | wheel Hz mean±SD | sonar J mean±SD (us) | tick_delta mean±SD |
|--------|---|------------------|------------------|----------------------|--------------------|
| noattest | 3 | 10.000±0.000 | 10.000±0.000 | 0.0±0.0 | 30000±0 |
| B64 | 3 | 10.000±0.000 | 10.000±0.000 | 26.6±0.4 | 30000±0 |
| B128 | 3 | 10.000±0.000 | 10.000±0.000 | 44.5±0.7 | 30000±0 |
| B256 | 3 | 10.000±0.000 | 10.000±0.000 | 72.3±3.0 | 30000±0 |
| B512 | 3 | 10.000±0.000 | 10.000±0.000 | 189.9±0.5 | 30000±0 |
| B1024 | 3 | 10.000±0.000 | 10.000±0.000 | 454.7±0.8 | 30000±0 |
| B2048 | 3 | 5.467±0.000 | 5.467±0.000 | 83924.1±81.0 | 16400±0 |
| B4096 | 3 | 2.800±0.000 | 2.800±0.000 | 260982.5±368.3 | 8400±0 |

Plots: fig_B_vs_freq.png, fig_B_vs_jitter.png, fig_B_vs_jitter_p95.png, fig_B_vs_tick_delta.png, fig_B_vs_tdis.png

## Per-run tags present
- `phase2r_noattest_r1`: sonar_n=300 Hz=10.000 Jmean=0.0 (sens=0.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase2r_noattest_r2`: sonar_n=300 Hz=10.000 Jmean=0.0 (sens=0.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase2r_noattest_r3`: sonar_n=300 Hz=10.000 Jmean=0.0 (sens=0.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B64_r1`: sonar_n=300 Hz=10.000 Jmean=26.2 (sens=26.2) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B64_r2`: sonar_n=300 Hz=10.000 Jmean=27.0 (sens=27.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B64_r3`: sonar_n=300 Hz=10.000 Jmean=26.4 (sens=26.4) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B128_r1`: sonar_n=300 Hz=10.000 Jmean=45.2 (sens=45.4) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B128_r2`: sonar_n=300 Hz=10.000 Jmean=43.9 (sens=44.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B128_r3`: sonar_n=300 Hz=10.000 Jmean=44.4 (sens=44.6) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B256_r1`: sonar_n=300 Hz=10.000 Jmean=69.7 (sens=69.7) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B256_r2`: sonar_n=300 Hz=10.000 Jmean=71.7 (sens=71.7) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B256_r3`: sonar_n=300 Hz=10.000 Jmean=75.6 (sens=75.5) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B512_r1`: sonar_n=300 Hz=10.000 Jmean=189.3 (sens=190.1) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B512_r2`: sonar_n=300 Hz=10.000 Jmean=190.1 (sens=190.8) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B512_r3`: sonar_n=300 Hz=10.000 Jmean=190.2 (sens=191.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B1024_r1`: sonar_n=300 Hz=10.000 Jmean=455.1 (sens=454.2) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B1024_r2`: sonar_n=300 Hz=10.000 Jmean=453.8 (sens=453.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B1024_r3`: sonar_n=300 Hz=10.000 Jmean=455.4 (sens=454.5) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B2048_r1`: sonar_n=164 Hz=5.467 Jmean=83995.8 (sens=83994.2) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_B2048_r2`: sonar_n=164 Hz=5.467 Jmean=83836.2 (sens=83847.2) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_B2048_r3`: sonar_n=164 Hz=5.467 Jmean=83940.2 (sens=83950.6) tick_delta=16400 first_imm=1/1 short_win=1
- `phase3r_B4096_r1`: sonar_n=84 Hz=2.800 Jmean=260588.0 (sens=260617.1) tick_delta=8400 first_imm=1/1 short_win=1
- `phase3r_B4096_r2`: sonar_n=84 Hz=2.800 Jmean=261042.0 (sens=261065.7) tick_delta=8400 first_imm=1/1 short_win=1
- `phase3r_B4096_r3`: sonar_n=84 Hz=2.800 Jmean=261317.3 (sens=261337.7) tick_delta=8400 first_imm=1/1 short_win=1
