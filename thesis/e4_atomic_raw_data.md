# E4 — Atomic Copy `t_disabled` (TIM2, NS mirror)

Branch: **`SAU-ATOMICCOPY`**. Firmware: `NS_APP_MODE_E4_ATOMIC=1` in `NonSecure/Src/main.c`.

- **Measured:** IRQ-masked **memcpy** flash → `ns_hmac_copy_buf` (TIM2 delta).
- **Not measured here:** Baseline SMARM (IRQ-off full HMAC) — use **`SAU-SMARM`** / table legacy / separate run.
- **Memory:** 128 KiB @ `0x08060000` (same as E1).
- **Per run:** 10 rounds × 32 samples (or all blocks if fewer).

## Run checklist (each B)

1. Edit `NS_HMAC_BLOCK_SIZE` in `main.c` (64 … 4096).
2. **Clean + Build** NonSecure (Secure image unchanged but **flash both**).
3. USART3 115200, reset, wait for `=== E4 Atomic Copy SUMMARY ===`.
4. Paste SUMMARY below.

| B | Mean (ms) | Std (ms) | Notes |
|---:|---:|---:|---|
| 64 | | | |
| 128 | | | |
| 256 | | | |
| 512 | | | |
| 1024 | | | |
| 2048 | | | |
| 4096 | | | |

---

(Paste UART SUMMARY blocks below.)
