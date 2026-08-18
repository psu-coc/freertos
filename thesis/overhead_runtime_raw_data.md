# Total attestation runtime (no real-time task)

Branch: **`SAU-ATOMICCOPY-OVERHEAD`**.  
Goal: wall-clock time of one full `SECURE_ShuffledHMAC_secure` call **without** `NormalTask`, so copy / SAU reconfiguration overhead is not mixed with a 1000 Hz workload.

- Platform: STM32L552ZE, Cortex-M33 at 110 MHz, FreeRTOS / TrustZone-M
- Timer: TIM2 @ 137500 ticks/s, start immediately before NSC call, stop immediately after return
- Rounds: 10 per configuration
- Guard window: `|M|=128` KiB @ `0x08060000–0x0807FFFF`, `ATTEST_PASSES=4` → **512 KiB hashed / round**
- `NS=0` and `FAR=0.0000` are expected (no `NormalTask`)

## Summary (mean Runtime, ms)

| $B$ (bytes) | Baseline SMARM | SMARM-Snap | SMARM-Guard |
|---:|---:|---:|---:|
| 64 |  |  | **635** |
| 128 |  |  | **556** |
| 256 |  |  | **517** |
| 512 |  |  | **497** |
| 1024 |  |  | **487** |
| 2048 |  |  | **482** |
| 4096 |  |  | **480** |

With 1000 Hz `NormalTask` (`sau_far_raw_data.md`): Guard $B{=}64$ was 645 ms, $B{=}128$ was 565 ms, $B{=}256$ was 525 ms, $B{=}512$ was 505 ms, $B{=}1024$ was 495 ms, $B{=}2048$ was 489 ms, $B{=}4096$ was 487 ms. No-RT runs are 7–10 ms shorter.

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
