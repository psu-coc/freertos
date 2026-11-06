# Total attestation runtime (no real-time task)

Switch modes in `Secure_nsclib/attest_overhead_cfg.h` only (`USE_SAU_APPROACH`, `USE_SNAP`, `ATTEST_BLOCK_SIZE`). Rebuild Secure + NonSecure.

Numbers below feed `thesis/main.tex` Table~\ref{tab:attest_runtime}. At $B{=}128$, Baseline/Guard are from the unified harness (567 / 559). Other cells are from the archived no-`NormalTask` sweeps in this file. Do not claim Guard is faster than Baseline.

Goal: wall-clock time of one full `SECURE_ShuffledHMAC_secure` call **without** `NormalTask`, so copy / SAU reconfiguration overhead is not mixed with a 1000 Hz workload.

- Platform: STM32L552ZE, Cortex-M33 at 110 MHz, FreeRTOS / TrustZone-M
- Timer: TIM2 @ 137500 ticks/s, start immediately before NSC call, stop immediately after return
- Rounds: 10 per configuration
- Window (all three modes): `|M|=128` KiB @ `0x08060000–0x0807FFFF`, `ATTEST_PASSES=4` → **512 KiB hashed / round**
- `NS=0` is expected (no `NormalTask`)

## Summary (mean Runtime, ms) — used in `main.tex` `tab:attest_runtime`

Ten rounds, no `NormalTask`. Sample SD over rounds is `$<1$` ms (often 0). Paper text does **not** claim Guard is faster than Baseline.

| $B$ (bytes) | Baseline SMARM | SMARM-Snap | SMARM-Guard |
|---:|---:|---:|---:|
| 64 | **644** | **654** | **635** |
| 128 | **567** (unified harness) | **592** | **559** (unified harness; replaces old 556) |
| 256 | **521** | **550** | **517** |
| 512 | **500** | **529** | **497** |
| 1024 | **490** | **519** | **487** |
| 2048 | **484** | **513** | **482** |
| 4096 | **481** | **511** | **480** |

With 1000 Hz `NormalTask` (`sau_far_raw_data.md`): Guard $B{=}64$ was 645 ms, $B{=}128$ was 565 ms, $B{=}256$ was 525 ms, $B{=}512$ was 505 ms, $B{=}1024$ was 495 ms, $B{=}2048$ was 489 ms, $B{=}4096$ was 487 ms. No-RT Guard runs are 7–10 ms shorter.

---

## Historical (not for paper)

The following Baseline/Snap UART dumps were measured on **thinned** `smarm` firmware (different binary from Guard). They stay here as an archive only.

---

## Baseline SMARM — $B = 4096$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=4096`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/4096 = 128$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/4` (`TARGET_FREQ_HZ=10` so expected $=481\times10/1000$)
- **Note:** lines above `FreeRTOS tasks created (SMARM Baseline...` are leftover Guard boot text in the ST-Link buffer from the previous image

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **481 ms** |
| Runtime min / max | **481 / 481 ms** |
| NS activations | **0 / 4** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 481 | 0/4 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=481 ms, NS=0/4 cycles
Round 2: Runtime=481 ms, NS=0/4 cycles
Round 3: Runtime=481 ms, NS=0/4 cycles
Round 4: Runtime=481 ms, NS=0/4 cycles
Round 5: Runtime=481 ms, NS=0/4 cycles
Round 6: Runtime=481 ms, NS=0/4 cycles
Round 7: Runtime=481 ms, NS=0/4 cycles
Round 8: Runtime=481 ms, NS=0/4 cycles
Round 9: Runtime=481 ms, NS=0/4 cycles
Round 10: Runtime=481 ms, NS=0/4 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 481 ms
Min:  481 ms
Max:  481 ms
==========================================
```

---

## Baseline SMARM — $B = 2048$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=2048`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/2048 = 256$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/4`
- **Note:** duplicated boot lines are resets mid-print; the 10-round block below is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **484 ms** |
| Runtime min / max | **484 / 484 ms** |
| NS activations | **0 / 4** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 484 | 0/4 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=484 ms, NS=0/4 cycles
Round 2: Runtime=484 ms, NS=0/4 cycles
Round 3: Runtime=484 ms, NS=0/4 cycles
Round 4: Runtime=484 ms, NS=0/4 cycles
Round 5: Runtime=484 ms, NS=0/4 cycles
Round 6: Runtime=484 ms, NS=0/4 cycles
Round 7: Runtime=484 ms, NS=0/4 cycles
Round 8: Runtime=484 ms, NS=0/4 cycles
Round 9: Runtime=484 ms, NS=0/4 cycles
Round 10: Runtime=484 ms, NS=0/4 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 484 ms
Min:  484 ms
Max:  484 ms
==========================================
```

---

## Baseline SMARM — $B = 1024$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=1024`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/1024 = 512$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/4`
- **Note:** the two `484 ms` rounds before the resets are leftover $B{=}2048$; the complete $B{=}1024$ run is the 10-round `490 ms` block

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **490 ms** |
| Runtime min / max | **490 / 490 ms** |
| NS activations | **0 / 4** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 490 | 0/4 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=490 ms, NS=0/4 cycles
Round 2: Runtime=490 ms, NS=0/4 cycles
Round 3: Runtime=490 ms, NS=0/4 cycles
Round 4: Runtime=490 ms, NS=0/4 cycles
Round 5: Runtime=490 ms, NS=0/4 cycles
Round 6: Runtime=490 ms, NS=0/4 cycles
Round 7: Runtime=490 ms, NS=0/4 cycles
Round 8: Runtime=490 ms, NS=0/4 cycles
Round 9: Runtime=490 ms, NS=0/4 cycles
Round 10: Runtime=490 ms, NS=0/4 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 490 ms
Min:  490 ms
Max:  490 ms
==========================================
```

---

## Baseline SMARM — $B = 512$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=512`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/512 = 1024$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/5` (`TARGET_FREQ_HZ=10` so expected $=500\times10/1000$)
- **Note:** duplicated boot lines are resets mid-print; the complete run is the 10-round `500 ms` block

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **500 ms** |
| Runtime min / max | **500 / 500 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 500 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=500 ms, NS=0/5 cycles
Round 2: Runtime=500 ms, NS=0/5 cycles
Round 3: Runtime=500 ms, NS=0/5 cycles
Round 4: Runtime=500 ms, NS=0/5 cycles
Round 5: Runtime=500 ms, NS=0/5 cycles
Round 6: Runtime=500 ms, NS=0/5 cycles
Round 7: Runtime=500 ms, NS=0/5 cycles
Round 8: Runtime=500 ms, NS=0/5 cycles
Round 9: Runtime=500 ms, NS=0/5 cycles
Round 10: Runtime=500 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 500 ms
Min:  500 ms
Max:  500 ms
==========================================
```

---

## Baseline SMARM — $B = 256$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=256`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/256 = 2048$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/5`
- **Note:** duplicated boot lines are resets mid-print; the complete run is the 10-round `521 ms` block

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **521 ms** |
| Runtime min / max | **521 / 521 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 521 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=521 ms, NS=0/5 cycles
Round 2: Runtime=521 ms, NS=0/5 cycles
Round 3: Runtime=521 ms, NS=0/5 cycles
Round 4: Runtime=521 ms, NS=0/5 cycles
Round 5: Runtime=521 ms, NS=0/5 cycles
Round 6: Runtime=521 ms, NS=0/5 cycles
Round 7: Runtime=521 ms, NS=0/5 cycles
Round 8: Runtime=521 ms, NS=0/5 cycles
Round 9: Runtime=521 ms, NS=0/5 cycles
Round 10: Runtime=521 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 521 ms
Min:  521 ms
Max:  521 ms
==========================================
```

---

## Baseline SMARM — $B = 128$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=128`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/128 = 4096$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/5`
- **Note:** duplicated boot lines are resets mid-print; the complete run is the 10-round block (round 2 is 563 ms, others 562 ms)

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **562 ms** |
| Runtime min / max | **562 / 563 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1 | 562 | 0/5 |
| 2 | 563 | 0/5 |
| 3–10 | 562 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=562 ms, NS=0/5 cycles
Round 2: Runtime=563 ms, NS=0/5 cycles
Round 3: Runtime=562 ms, NS=0/5 cycles
Round 4: Runtime=562 ms, NS=0/5 cycles
Round 5: Runtime=562 ms, NS=0/5 cycles
Round 6: Runtime=562 ms, NS=0/5 cycles
Round 7: Runtime=562 ms, NS=0/5 cycles
Round 8: Runtime=562 ms, NS=0/5 cycles
Round 9: Runtime=562 ms, NS=0/5 cycles
Round 10: Runtime=562 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 562 ms
Min:  562 ms
Max:  563 ms
==========================================
```

---

## Baseline SMARM — $B = 64$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=0`, `BLOCK_SIZE=64`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/64 = 8192$
- **UART check:** `SMARM Baseline, no NormalTask`; `NS=0/6`
- **Note:** $B{=}64$ is extra (paper eval table starts at 128). Duplicated boot lines are resets mid-print.

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **644 ms** |
| Runtime min / max | **643 / 646 ms** |
| NS activations | **0 / 6** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1 | 645 | 0/6 |
| 2 | 643 | 0/6 |
| 3 | 646 | 0/6 |
| 4 | 645 | 0/6 |
| 5 | 644 | 0/6 |
| 6 | 644 | 0/6 |
| 7 | 645 | 0/6 |
| 8 | 645 | 0/6 |
| 9 | 643 | 0/6 |
| 10 | 646 | 0/6 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=645 ms, NS=0/6 cycles
Round 2: Runtime=643 ms, NS=0/6 cycles
Round 3: Runtime=646 ms, NS=0/6 cycles
Round 4: Runtime=645 ms, NS=0/6 cycles
Round 5: Runtime=644 ms, NS=0/6 cycles
Round 6: Runtime=644 ms, NS=0/6 cycles
Round 7: Runtime=645 ms, NS=0/6 cycles
Round 8: Runtime=645 ms, NS=0/6 cycles
Round 9: Runtime=643 ms, NS=0/6 cycles
Round 10: Runtime=646 ms, NS=0/6 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 644 ms
Min:  643 ms
Max:  646 ms
==========================================
```

---

## SMARM-Snap — $B = 4096$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=4096`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/4096 = 128$
- **UART check:** NonSecure banner still prints `SMARM Baseline` (NS `printf` is not mode-aware); identity of this run is `USE_SNAP=1` + $B{=}4096$
- **Note:** $+30$ ms vs Baseline $481$ ms matches an extra 512 KiB Secure `memcpy` (~$17$ MB/s)

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **511 ms** |
| Runtime min / max | **511 / 511 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 511 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=511 ms, NS=0/5 cycles
Round 2: Runtime=511 ms, NS=0/5 cycles
Round 3: Runtime=511 ms, NS=0/5 cycles
Round 4: Runtime=511 ms, NS=0/5 cycles
Round 5: Runtime=511 ms, NS=0/5 cycles
Round 6: Runtime=511 ms, NS=0/5 cycles
Round 7: Runtime=511 ms, NS=0/5 cycles
Round 8: Runtime=511 ms, NS=0/5 cycles
Round 9: Runtime=511 ms, NS=0/5 cycles
Round 10: Runtime=511 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 511 ms
Min:  511 ms
Max:  511 ms
==========================================
```

---

## SMARM-Snap — $B = 2048$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=2048`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/2048 = 256$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}2048$
- **Note:** duplicated boot lines are resets mid-print; $+29$ ms vs Baseline $484$ ms

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **513 ms** |
| Runtime min / max | **513 / 513 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 513 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=513 ms, NS=0/5 cycles
Round 2: Runtime=513 ms, NS=0/5 cycles
Round 3: Runtime=513 ms, NS=0/5 cycles
Round 4: Runtime=513 ms, NS=0/5 cycles
Round 5: Runtime=513 ms, NS=0/5 cycles
Round 6: Runtime=513 ms, NS=0/5 cycles
Round 7: Runtime=513 ms, NS=0/5 cycles
Round 8: Runtime=513 ms, NS=0/5 cycles
Round 9: Runtime=513 ms, NS=0/5 cycles
Round 10: Runtime=513 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 513 ms
Min:  513 ms
Max:  513 ms
==========================================
```

---

## SMARM-Snap — $B = 1024$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=1024`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/1024 = 512$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}1024$
- **Note:** duplicated boot lines are resets mid-print; $+29$ ms vs Baseline $490$ ms

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **519 ms** |
| Runtime min / max | **519 / 519 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 519 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=519 ms, NS=0/5 cycles
Round 2: Runtime=519 ms, NS=0/5 cycles
Round 3: Runtime=519 ms, NS=0/5 cycles
Round 4: Runtime=519 ms, NS=0/5 cycles
Round 5: Runtime=519 ms, NS=0/5 cycles
Round 6: Runtime=519 ms, NS=0/5 cycles
Round 7: Runtime=519 ms, NS=0/5 cycles
Round 8: Runtime=519 ms, NS=0/5 cycles
Round 9: Runtime=519 ms, NS=0/5 cycles
Round 10: Runtime=519 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 519 ms
Min:  519 ms
Max:  519 ms
```

---

## SMARM-Snap — $B = 512$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=512`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/512 = 1024$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}512$
- **Note:** $+29$ ms vs Baseline $500$ ms

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **529 ms** |
| Runtime min / max | **529 / 529 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–10 | 529 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=529 ms, NS=0/5 cycles
Round 2: Runtime=529 ms, NS=0/5 cycles
Round 3: Runtime=529 ms, NS=0/5 cycles
Round 4: Runtime=529 ms, NS=0/5 cycles
Round 5: Runtime=529 ms, NS=0/5 cycles
Round 6: Runtime=529 ms, NS=0/5 cycles
Round 7: Runtime=529 ms, NS=0/5 cycles
Round 8: Runtime=529 ms, NS=0/5 cycles
Round 9: Runtime=529 ms, NS=0/5 cycles
Round 10: Runtime=529 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 529 ms
Min:  529 ms
Max:  529 ms
==========================================
```

---

## SMARM-Snap — $B = 256$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=256`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/256 = 2048$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}256$
- **Note:** duplicated boot lines are resets mid-print; round 5 is 551 ms; $+29$ ms vs Baseline $521$ ms

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **550 ms** |
| Runtime min / max | **550 / 551 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–4, 6–10 | 550 | 0/5 |
| 5 | 551 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=550 ms, NS=0/5 cycles
Round 2: Runtime=550 ms, NS=0/5 cycles
Round 3: Runtime=550 ms, NS=0/5 cycles
Round 4: Runtime=550 ms, NS=0/5 cycles
Round 5: Runtime=551 ms, NS=0/5 cycles
Round 6: Runtime=550 ms, NS=0/5 cycles
Round 7: Runtime=550 ms, NS=0/5 cycles
Round 8: Runtime=550 ms, NS=0/5 cycles
Round 9: Runtime=550 ms, NS=0/5 cycles
Round 10: Runtime=550 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 550 ms
Min:  550 ms
Max:  551 ms
==========================================
```

---

## SMARM-Snap — $B = 128$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=128`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/128 = 4096$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}128$
- **Note:** duplicated boot lines are resets mid-print; round 6 is 593 ms; $+30$ ms vs Baseline $562$ ms

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **592 ms** |
| Runtime min / max | **592 / 593 ms** |
| NS activations | **0 / 5** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1–5, 7–10 | 592 | 0/5 |
| 6 | 593 | 0/5 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=592 ms, NS=0/5 cycles
Round 2: Runtime=592 ms, NS=0/5 cycles
Round 3: Runtime=592 ms, NS=0/5 cycles
Round 4: Runtime=592 ms, NS=0/5 cycles
Round 5: Runtime=592 ms, NS=0/5 cycles
Round 6: Runtime=593 ms, NS=0/5 cycles
Round 7: Runtime=592 ms, NS=0/5 cycles
Round 8: Runtime=592 ms, NS=0/5 cycles
Round 9: Runtime=592 ms, NS=0/5 cycles
Round 10: Runtime=592 ms, NS=0/5 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 592 ms
Min:  592 ms
Max:  593 ms
==========================================
```

---

## SMARM-Snap — $B = 64$, no NormalTask

- **Date:** 2026-08-19
- **Branch:** `smarm`
- **Firmware:** `USE_SNAP=1`, `BLOCK_SIZE=64`, `TOTAL_SIZE=0x80000` (512 KiB @ `0x08000000`, one pass)
- **Blocks / round:** $512\,\mathrm{KiB}/64 = 8192$
- **UART check:** NonSecure banner still prints `SMARM Baseline`; identity of this run is `USE_SNAP=1` + $B{=}64$
- **Note:** $B{=}64$ is extra (paper eval table starts at 128). Firmware integer mean is $654$ ($6549/10 = 654.9$). $+10$ ms vs Baseline $644$ ms — smaller than the $\sim 29$ ms Snap tax at $B\geq128$, consistent with HMAC-from-RAM offsetting more of the extra `memcpy` when flash is read in 64-byte chunks. Duplicated boot lines are resets mid-print.

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **654 ms** |
| Runtime min / max | **654 / 656 ms** |
| NS activations | **0 / 6** (all 10 rounds) |

| Round | Runtime (ms) | NS |
|---:|---:|---|
| 1 | 655 | 0/6 |
| 2 | 654 | 0/6 |
| 3 | 654 | 0/6 |
| 4 | 656 | 0/6 |
| 5 | 656 | 0/6 |
| 6 | 654 | 0/6 |
| 7 | 655 | 0/6 |
| 8 | 654 | 0/6 |
| 9 | 656 | 0/6 |
| 10 | 655 | 0/6 |

### Raw UART

```text
FreeRTOS tasks created (SMARM Baseline, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (baseline, no NormalTask, |M|=512KiB).
Round 1: Runtime=655 ms, NS=0/6 cycles
Round 2: Runtime=654 ms, NS=0/6 cycles
Round 3: Runtime=654 ms, NS=0/6 cycles
Round 4: Runtime=656 ms, NS=0/6 cycles
Round 5: Runtime=656 ms, NS=0/6 cycles
Round 6: Runtime=654 ms, NS=0/6 cycles
Round 7: Runtime=655 ms, NS=0/6 cycles
Round 8: Runtime=654 ms, NS=0/6 cycles
Round 9: Runtime=656 ms, NS=0/6 cycles
Round 10: Runtime=655 ms, NS=0/6 cycles

=== SUMMARY SMARM Baseline, no NormalTask (10 rounds) ===
Mean: 654 ms
Min:  654 ms
Max:  656 ms
==========================================
```

---

## Guard — $B = 4096$, no NormalTask

- **Date:** 2026-08-18
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=4096`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/4096 = 32$; **per NS round:** $32\times 4 = 128$
- **UART check:** `FreeRTOS tasks created (SMARM, no NormalTask)` and no `.` heartbeat between call and return

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **480 ms** |
| Runtime min / max | **480 / 480 ms** |
| SAU_config_avg_cycles | **142–143** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1 | 480 | 0/480 | 0.0000 | 143 |
| 2 | 480 | 0/480 | 0.0000 | 143 |
| 3 | 480 | 0/480 | 0.0000 | 143 |
| 4 | 480 | 0/480 | 0.0000 | 143 |
| 5 | 480 | 0/480 | 0.0000 | 142 |
| 6 | 480 | 0/480 | 0.0000 | 142 |
| 7 | 480 | 0/480 | 0.0000 | 143 |
| 8 | 480 | 0/480 | 0.0000 | 142 |
| 9 | 480 | 0/480 | 0.0000 | 143 |
| 10 | 480 | 0/480 | 0.0000 | 143 |

### Raw UART

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 2: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 3: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 4: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 5: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=142
Round 6: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=142
Round 7: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 8: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=142
Round 9: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143
Round 10: Runtime=480 ms, NS=0/480 activations, FAR=0.0000
SAU_config_avg_cycles=143

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 480 ms
Min:  480 ms
Max:  480 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 2048$, no NormalTask

- **Date:** 2026-08-18
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=2048`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/2048 = 64$; **per NS round:** $64\times 4 = 256$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **482 ms** |
| Runtime min / max | **482 / 482 ms** |
| SAU_config_avg_cycles | **115** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 482 | 0/482 | 0.0000 | 115 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 2: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 3: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 4: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 5: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 6: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 7: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 8: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 9: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115
Round 10: Runtime=482 ms, NS=0/482 activations, FAR=0.0000
SAU_config_avg_cycles=115

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 482 ms
Min:  482 ms
Max:  482 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 1024$, no NormalTask

- **Date:** 2026-08-18
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=1024`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/1024 = 128$; **per NS round:** $128\times 4 = 512$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **487 ms** |
| Runtime min / max | **487 / 487 ms** |
| SAU_config_avg_cycles | **100–101** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 487 | 0/487 | 0.0000 | 100–101 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 2: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=100
Round 3: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 4: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 5: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 6: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=100
Round 7: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=100
Round 8: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 9: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101
Round 10: Runtime=487 ms, NS=0/487 activations, FAR=0.0000
SAU_config_avg_cycles=101

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 487 ms
Min:  487 ms
Max:  487 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 512$, no NormalTask

- **Date:** 2026-08-18
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=512`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/512 = 256$; **per NS round:** $256\times 4 = 1024$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **497 ms** |
| Runtime min / max | **497 / 497 ms** |
| SAU_config_avg_cycles | **93–94** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 497 | 0/497 | 0.0000 | 93–94 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 2: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 3: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 4: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=93
Round 5: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 6: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 7: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 8: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 9: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94
Round 10: Runtime=497 ms, NS=0/497 activations, FAR=0.0000
SAU_config_avg_cycles=94

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 497 ms
Min:  497 ms
Max:  497 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 256$, no NormalTask

- **Date:** 2026-08-18
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=256`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/256 = 512$; **per NS round:** $512\times 4 = 2048$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **517 ms** |
| Runtime min / max | **517 / 517 ms** |
| SAU_config_avg_cycles | **90** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 517 | 0/517 | 0.0000 | 90 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 2: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 3: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 4: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 5: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 6: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 7: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 8: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 9: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90
Round 10: Runtime=517 ms, NS=0/517 activations, FAR=0.0000
SAU_config_avg_cycles=90

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 517 ms
Min:  517 ms
Max:  517 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 128$, no NormalTask

- **Date:** 2026-08-19
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=128`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/128 = 1024$; **per NS round:** $1024\times 4 = 4096$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **556 ms** |
| Runtime min / max | **556 / 556 ms** |
| SAU_config_avg_cycles | **88** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 556 | 0/556 | 0.0000 | 88 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 2: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 3: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 4: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 5: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 6: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 7: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 8: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 9: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88
Round 10: Runtime=556 ms, NS=0/556 activations, FAR=0.0000
SAU_config_avg_cycles=88

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 556 ms
Min:  556 ms
Max:  556 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```

---

## Guard — $B = 64$, no NormalTask

- **Date:** 2026-08-19
- **Firmware:** `USE_SAU_APPROACH=1`, `BLOCK_SIZE=64`, `ATTEST_PASSES=4`
- **Blocks / pass:** $128\,\mathrm{KiB}/64 = 2048$; **per NS round:** $2048\times 4 = 8192$
- **UART check:** `SMARM, no NormalTask`; first paste was a reset mid-wait, second paste is the complete run
- **Note:** $B{=}64$ is extra (paper eval table starts at 128)

### Summary

| Metric | Value |
|---|---|
| Runtime mean | **635 ms** |
| Runtime min / max | **635 / 635 ms** |
| SAU_config_avg_cycles | **87** |

| Round | Runtime (ms) | NS activations | FAR | SAU avg. cycles/block |
|---:|---:|---|---:|---:|
| 1–10 | 635 | 0/635 | 0.0000 | 87 |

### Raw UART (complete run)

```text
[NS boot] SMARM+SAU pure Guard |M|=128KiB @0x08060000 x4 passes (=512KiB work).
UART log: ST-Link VCP=USART3 (COMx) + LPUART1 PG7 — 115200. Open terminal BEFORE reset.
Starting FreeRTOS (E1: SMARM+SAU FAR)...
FreeRTOS tasks created (SMARM, no NormalTask). Starting scheduler...
SMARM_Experiment_Task running (overhead, no NormalTask).

SMARM+SAU pure Guard |M|=128KiB @0x08060000, 4 passes/round (=512KiB hashed; SAU every block) (10 rounds).
Allocating secure context (8 KiB)...
Secure context OK. Waiting 1 s then starting rounds...
  NS: SECURE_LEDToggle OK (SG path alive)
Calling SECURE_ShuffledHMAC_secure (round 1)...
  NS: returned from SECURE_ShuffledHMAC_secure
Returned from secure attestation (round 1).
Round 1: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 2: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 3: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 4: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 5: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 6: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 7: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 8: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 9: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87
Round 10: Runtime=635 ms, NS=0/635 activations, FAR=0.0000
SAU_config_avg_cycles=87

=== SUMMARY SMARM+SAU (10 rounds) ===
Mean: 635 ms
Min:  635 ms
Max:  635 ms
FAR mean=0.0000, min=0.0000, max=0.0000
==========================================
Done. Press RESET to run again.
```
