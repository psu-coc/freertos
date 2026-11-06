# SMARM+ Thesis Data Guide — Workload Source + Raw Timing Logs

## Branch / commit (FreeRTOS primary)

| Item | Value |
|---|---|
| Remote | `git@github.com:psu-coc/freertos.git` |
| Branch | **`smarmplus-key-separation`** |
| Tip used for this pack | see `git log -1` after pull |
| Clone | `git clone -b smarmplus-key-separation git@github.com:psu-coc/freertos.git` |

Zephyr Gate 3 TIM2 evidence lives on the Windows share  
`D:\smarm\SMARMPlus_Archive_20261105\...` (not in this git remote). Cross-RTOS review notes are under `thesis/03_rt_experiment/key_separation/ns256/additional_eval/advisor_evidence_v1/`.

---

## 1. Workload source (Sonar / Wheel / Attest)

| Role | Path on this branch |
|---|---|
| Sonar + Wheel task loops, TIM2, observation window, UART dump | `NonSecure/Src/main.c` (`PulseSmoke_Task`, `WheelSmoke_Task`, `obs_sync_t0`, DMR store) |
| Profile: Dual10 / Wheel100 / Wheel1000 periods, warmup, duration | `NonSecure/Inc/rt_experiment_profile.h` |
| Formal DMR record layout / anchors | `NonSecure/Inc/rt_dmr_formal.h` |
| FreeRTOS tick (1000 Hz), preemption | `NonSecure/Inc/FreeRTOSConfig.h` |
| CMSIS `osDelayUntil` | `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c` |
| Host recompute helpers | `thesis/03_rt_experiment/scripts/rt_metrics_wall30.py` |

**Timebase:** TIM2 configured for **1 µs / count** (`us_now()`). All `*_us` fields are **microseconds** on that counter unless noted.

---

## 2. Raw timing log schema (Formal DMR CSV)

Primary per-job files:

`thesis/03_rt_experiment/key_separation/ns256/additional_eval/dmr/raw/dmr_<Profile>_<mode>_B<B>_r<run>/dmr_*_formal_dmr.csv`

Example: `dmr_Wheel1000Hz_original_B64_r1/dmr_Wheel1000Hz_original_B64_r1_formal_dmr.csv`

### Column map (thesis vocabulary)

| Your term | CSV column | Unit / meaning |
|---|---|---|
| Configuration / profile | folder name + `period_us` | `10Hz10Hz`, `Wheel100Hz`, `Wheel1000Hz` |
| Mode / scheme | folder `original` / `prng` / `fpe` / `noattest` | attestation algorithm |
| Block size | folder `B64`…`B4096` | bytes |
| Run id | `_r1` / `_r2` / `_r3` | repetition |
| Task name | `task` | `sonar` or `wheel` |
| Job number | `job_id` / `iter_ord` | sequential stored index (not invented gap IDs) |
| Nominal wake / release tick | `sched_wake_tick` | FreeRTOS tick count passed to `osDelayUntil` |
| Nominal release time | `r_us` | µs (mapped from tick anchor) |
| Deadline | `d_us` | µs; Formal DMR uses **D = P** (`D_equals_P=1`) |
| **Actual start** | **`s_us`** | µs TIM2, post-`osDelayUntil` |
| **Completion** | **`f_us`** | µs TIM2 finish |
| Window open (Wheel) | `window_open_us` | µs before delay (0 if N/A) |
| Observation start | **`T0_us`** | µs; experiment window start |
| Observation end | **`Tend_us`** | µs; `T0 + duration` |
| Tick at T0 | `tick0` | kernel tick latched at observation entry |
| Period | `period_us` | 100000 (Dual10), 10000 (Wheel100), 1000 (Wheel1000) |
| Status | `status` / `status_code` | ontime / late / … |
| Flags | `flags` | instrumentation bits (immediate wake, short window, …) |

### Aggregates (not per-job)

| File | Use |
|---|---|
| `.../dmr/final/FINAL_DMR_RESULTS.csv` | Per run Fixed Formal DMR summary (eligible, late, unexec, censored, dmr, …) |
| `.../dataset_inventory_v1/COMPLETE_TASK_METRICS.csv` | Inventory table across configs |
| `.../advisor_evidence_v1/jitter_summary.csv` | Audited inter-start jitter (primary drops first in-window start) |
| `.../advisor_evidence_v1/GATE3_CROSS_RTOS_SCIENTIFIC_REVIEW.md` | FreeRTOS vs Zephyr qualitative review |
| `.../advisor_evidence_v1/WHEEL1000_CROSS_RTOS_IMPLEMENTATION_EQUIVALENCE_AUDIT.md` | 1000 Hz scheduling non-equivalence audit |

### How to compute Fixed Formal DMR (high level)

- Eligible jobs: nominal releases with deadline inside `[T0, Tend)` (right-censored excluded from denom).
- Miss if finish `f_us > d_us` (late) or job unexecuted; see `FINAL_DMR_RESULTS.csv` and `REPRODUCTION_VERIFICATION.md`.
- Zephyr runtime DMR ledger ≠ FreeRTOS Fixed Formal DMR (Zephyr Formal still quarantined).

### Inter-start jitter (audited)

`J = mean |s[k] − s[k−1] − P|` after dropping the first in-window start (`rt_metrics_wall30.py`).

---

## 3. What was uploaded vs kept local

**Pushed in this pack (approx.):**

- Workload / DMR headers already on branch + `thesis/` guide, manuscript `02_smarmplus`, analysis CSVs, scripts, `dmr/raw` formal CSVs + companion txt/json, `dmr/final`, advisor evidence.

**Not pushed (too large / rebuildable):**

- `thesis/03_rt_experiment/builds/`, `checkpoints/`, `dmr/builds/`, `dmr/recapture/` (~GB of ELF/hex)
- STM32CubeIDE `Debug/` object files

Those stay on the lab machine; raw **CSV timing** needed for thesis tables is under `dmr/raw`.

---

## 4. Quick open checklist

```bash
git fetch origin
git checkout smarmplus-key-separation
git pull
```

Then open:

1. `NonSecure/Src/main.c` — Sonar/Wheel loops  
2. `thesis/03_rt_experiment/key_separation/ns256/additional_eval/dmr/raw/` — per-job timing  
3. `thesis/THESIS_DATA_GUIDE.md` — this file  
