# wall30 B-sweep aggregate

Primary David jitter: **excludes first in-window sample** (and its interval).
Frequency: **all** in-window starts. Formal DMR: **QUARANTINED**.

Per-run rows found: 2 / 24 expected.

| config | n | sonar Hz mean±SD | wheel Hz mean±SD | sonar J mean±SD (us) | tick_delta mean±SD |
|--------|---|------------------|------------------|----------------------|--------------------|
| noattest | 1 | 10.000±0.000 | 10.000±0.000 | 0.0±0.0 | 30000±0 |
| B2048 | 1 | 5.467±0.000 | 5.467±0.000 | 83995.8±0.0 | 16400±0 |

Plots: fig_B_vs_freq.png, fig_B_vs_jitter.png, fig_B_vs_jitter_p95.png, fig_B_vs_tick_delta.png, fig_B_vs_tdis.png

## Per-run tags present
- `phase2r_noattest_r1`: sonar_n=300 Hz=10.000 Jmean=0.0 (sens=0.0) tick_delta=30000 first_imm=1/1 short_win=1
- `phase3r_B2048_r1`: sonar_n=164 Hz=5.467 Jmean=83995.8 (sens=83994.2) tick_delta=16400 first_imm=1/1 short_win=1
