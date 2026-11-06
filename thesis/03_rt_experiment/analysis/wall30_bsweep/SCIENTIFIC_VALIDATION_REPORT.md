# wall30 B-sweep — READ-ONLY Scientific Validation

**Date:** 2026-10-20  
**Scope:** Final validation before paper reporting.  
**Constraint:** No raw data, source, or script modifications; no flash/HW.

---

## Verdict summary

| # | Item | Verdict |
|---|------|---------|
| 1 | Independent hardware captures (3 runs/config) | **PASS with WARNING** (noattest timing-path determinism) |
| 2 | Noattest Wheel CSV byte-identical r1/r2/r3 | **WARNING** (not file-reuse of full run; limited Wheel independence) |
| 3 | Independent recompute freq + David jitter (startup exclusion) | **PASS** |
| 4 | Aggregate mean/SD vs recompute | **PASS** |
| 5 | t_disabled ↔ SysTick ↔ frequency relationship | **PASS** (associational; see wording limits) |
| 6 | Real sensor execution / Wheel limitations | **WARNING** |
| 7 | Formal DMR quarantined | **PASS** |

**Overall for paper use of frequency/jitter/tdis/tick tables:** usable with stated warnings. Do not over-claim Wheel as physical rotation sensing or noattest triplicates as independent timing noise samples.

---

## 1. Independent captures — PASS with WARNING

### Evidence of separate capture sessions

| Evidence | Finding |
|----------|---------|
| `logs/_campaign_wall30_bsweep_console.txt` (UTF-16) | **22** `==== CAPTURE` + **22** `VALIDATE_OK`; ends with `CAMPAIGN_CAPTURES_DONE count=22` |
| Capture order in console | noattest r2→r3, then B64…B1024 ×3, B2048 r2→r3, B4096 ×3 |
| Smoke r1 (prior sessions) | `phase2r_noattest_r1` (21:42), `phase3r_B2048_r1` (smoke gate); not overwritten |
| Original SMARM B64–B4096 | Sonar `actual_start_us` sequences **differ** across r1/r2/r3 (`r1==r2` False for all) |
| B2048/B4096 file SHA256 | All three runs unique for UART/Sonar/Wheel despite equal sizes |

### WARNING — No Attestation release-timing path is highly deterministic

Across `phase2r_noattest_r{1,2,3}`:

- Identical `T0=2028336`, `Tend=32028336`, `epoch=28336`
- **Identical** Sonar `actual_start_us` sequences (300/300 equal)
- **Identical** Wheel TIM2 sequences and **byte-identical** Wheel CSV (SHA256 `9fe7cc864fd518f8…`)
- **Non-identical** Sonar `finish_us` / `echo_us` / `distance_cm` / `exec_us` (hundreds of row diffs) and unique UART/Sonar SHA256

**Interpretation:** Not a wholesale copy of one capture reused as three files (sensor echo/finish fields change; UART hashes differ; mtimes differ; campaign console records distinct r2/r3 flashes). Consistent with **bit-stable DelayUntil/TIM2 wake schedule after identical reset**, while the ultrasonic echo path still varies. Therefore noattest **triplicates are weak for estimating across-run timing variance** (SD of jitter ≈ 0), but remain valid for confirming the 10 Hz / zero primary-jitter baseline.

---

## 2. Noattest Wheel CSV identity — WARNING

| Check | Result |
|-------|--------|
| SHA256 r1=r2=r3 | Yes (same digest) |
| `pulse_count` | All **0** every row |
| Full run reused? | **No** — Sonar/UART content differ as above |
| Mechanism | Zero edge interrupts + deterministic `window_open`→`osDelayUntil`→`s_exec` TIM2 stamps matching across resets |

**Do not** treat identical Wheel files as proof of accidental log cloning. **Do** treat them as evidence that Wheel noattest replicates add **no independent timing information** under this bench setup.

---

## 3. Independent recompute — PASS

From raw CSV TIM2 stamps + UART `T0`/`Tend`:

- Frequency: `n_starts / ((Tend-T0)/1e6)` with all in-window starts  
- Primary David jitter: drop first in-window stamp; `abs(Δs_exec − 100000)` on remaining pairs  
- Sensitivity: all consecutive in-window pairs  

Compared to `analysis/wall30_bsweep/per_run_metrics.csv` for all **24** tags (Sonar/Wheel n, Hz, mean/SD/P95/max): **0 mismatches**.

---

## 4. Aggregate agreement — PASS

Recomputed mean ± sample SD across 3 runs matches `aggregate_by_config.csv` for Sonar Hz, primary J mean, and `tick_delta` for every configuration (numeric agreement within floating tolerance).

---

## 5. t_disabled, SysTick, frequency — PASS (associational)

Device fields: `tdis_avg` (UART), `tick_delta`, observed Hz (from TIM2 window starts).

| Config | tdis_avg | tdis/P | tick_delta | tick/30000 | Hz |
|--------|----------|--------|------------|------------|-----|
| noattest | — | — | 30000 | 1.00 | 10.000 |
| B64 | 6647 | 0.07 | 30000 | 1.00 | 10.000 |
| B128 | 13059 | 0.13 | 30000 | 1.00 | 10.000 |
| B256 | 25882 | 0.26 | 30000 | 1.00 | 10.000 |
| B512 | 51528 | 0.52 | 30000 | 1.00 | 10.000 |
| B1024 | 102825 | 1.03 | 30000 | 1.00 | 10.000 |
| B2048 | 205363 | 2.05 | 16400 | **0.547** | **5.467** |
| B4096 | 410439 | 4.10 | 8400 | **0.280** | **2.800** |

**Supported by measurements:**  
(1) `tdis_avg` scales roughly linearly with B.  
(2) When `tick_delta` falls below 30000, observed Hz falls in the **same proportion** (`tick/30000 ≈ Hz/10` at B2048 and B4096).  
(3) Full tick advancement and 10 Hz coexist through B1024 even though `tdis/P` exceeds 1 at B1024 — so **tdis/P > 1 alone does not imply frequency collapse** in this dataset.

**Not claimed:** a unique causal proof beyond the Original SMARM design (IRQ-masked HMAC can stall SysTick while TIM2 free-runs). Paper text should state co-occurrence / consistency with that mechanism, not a fitted causal model.

---

## 6. Real sensor execution / limitations — WARNING

| Signal | Evidence | Implication |
|--------|----------|-------------|
| Sonar | Varying `echo_us` / `distance_cm` across jobs and across noattest runs | Real ultrasonic exec (fixed-range bench: distance often constant cm bin) |
| Wheel | `pulse_count` sum **0** on inspected runs for **all** configs | No wheel edges; “Wheel” metrics are **schedule/window timing**, not RPM/edge sensing |
| Dual-task load | Both tasks log TIM2 wakes at matched rates | Valid for RT interference study; not a claim of active wheel hardware stimulus |

---

## 7. Formal DMR — PASS (quarantined)

`per_run_metrics.csv` and `aggregate_by_config.csv` column `dmr` = **QUARANTINED** for all rows. No floor/spill or lattice flags used as formal deadline-miss results in these aggregates.

---

## Concrete file evidence index

- Campaign execution: `logs/_campaign_wall30_bsweep_console.txt`  
- Raw runs: `logs/phase2r_noattest_r{1..3}_*`, `logs/phase3r_B*_r{1..3}_*`  
- Prior integrity: `analysis/wall30_bsweep/RAW_INTEGRITY_AUDIT.md`  
- Aggregates: `per_run_metrics.csv`, `aggregate_by_config.csv`, `CAMPAIGN_SUMMARY.md`

---

## STOP

Scientific validation complete. No data/code/hardware changes performed.
