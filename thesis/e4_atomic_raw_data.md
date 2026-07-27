# E4 — Atomic Copy `t_disabled` (TIM2, NS mirror)

Branch: **`SAU-ATOMICCOPY`**. Date: **2026-07-27**.

- **Measured:** IRQ-masked **memcpy** (flash → `ns_hmac_copy_buf`), TIM2 delta @ **137500 ticks/s**.
- **Memory:** 128 KiB @ `0x08060000` (`NS_ATTEST_DATA_BYTES=0x20000`).
- **Per config:** 10 rounds × 32 timed samples (shuffled block indices).
- **Baseline column** (`tab:tdisabled`): not remeasured here — use existing paper / `SAU-SMARM` later.

## Summary table (Atomic Copy — for `tab:tdisabled`)

| B (bytes) | Mean (ms) | Std (ms) | Mean (ticks) | Notes |
|---:|---:|---:|---:|---|
| 64 | **&lt;0.007** | ~0 | 0* | *integer mean 0; max 1 tick = 0.007272 ms |
| 128 | **0.007272** | 0 | 1 | |
| 256 | **0.014545** | 0 | 2 | |
| 512 | **0.029090** | 0 | 4 | |
| 1024 | **0.058181** | 0 | 8 | |
| 2048 | **0.116363** | 0 | 16 | |
| 4096 | **0.240000** | 0 | 33 | |

Roughly **~2 ticks per 128 bytes** copied (flash read + RAM write under IRQ off). Matches prior paper atomic column within ~1 TIM2 tick.

**E4 Atomic (NS):** `[x]` 7/7 B archived below.

---

## B = 64

```text
=== E4 Atomic Copy SUMMARY (B=64, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=0 ticks (0.000000 ms), std~=0 ticks, min=0 max=0 ticks
(per-round max often 1 tick = 0.007272 ms)
```

## B = 128

```text
=== E4 Atomic Copy SUMMARY (B=128, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=1 ticks (0.007272 ms), std~=0, min=1 max=1 ticks
```

## B = 256

```text
=== E4 Atomic Copy SUMMARY (B=256, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=2 ticks (0.014545 ms), std~=0, min=2 max=2 ticks
```

## B = 512

```text
=== E4 Atomic Copy SUMMARY (B=512, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=4 ticks (0.029090 ms), std~=0, min=4 max=4 ticks
```

## B = 1024

```text
=== E4 Atomic Copy SUMMARY (B=1024, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=8 ticks (0.058181 ms), std~=0, min=8 max=8 ticks
```

## B = 2048

```text
=== E4 Atomic Copy SUMMARY (B=2048, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=16 ticks (0.116363 ms), std~=0, min=16 max=16 ticks
```

## B = 4096

```text
=== E4 Atomic Copy SUMMARY (B=4096, 10 rounds) ===
Per-update memcpy (IRQ-off): mean=33 ticks (0.240000 ms), std~=0, min=33 max=33 ticks
```

---

*Do not edit `main.tex` until baseline + paper sync agreed.*
