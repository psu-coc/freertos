# Snap/Guard — สถานะ Overleaf + raw t_disabled

อัปเดต: 2026-10-14  
Branch: `snapguard-paper-reviewer`  
Overleaf: `snapguard_overleaf_10062026.tex` (ยังไม่แก้ tex จนกว่าได้ raw)

---

## Instrumented (done)

`TDIS_RAW_DUMP=1` in `Secure_nsclib/attest_overhead_cfg.h`

- Secure: each IRQ-off window → NS buffer + tag `L`/`U`/`W`; also online min/avg/max
- NS UART: after each of 10 rounds prints summary + raw CSV block
- Parse: `parse_tdis_raw.py` → mean/min/max/std + Guard `max(L,U)` per block
- Sweep how-to: `tdis_raw_sweep.md`

Format:
```
TDIS_RAW round=1 B=4096 count=64
L,72
U,71
TDIS_RAW_END
```

---

## Next (you)

1. Rebuild Secure + NonSecure in CubeIDE, flash both
2. Sweep modes × B∈{128…4096}, 10 rounds each; save UART logs
3. `python thesis/01_snapguard/parse_tdis_raw.py <log>`
4. Then update Table / answer `\oak{}` (max, std, Guard max(L,U), benefit prose)

---

## Oak reminders (after data)

- Guard tdis = max(lock, unlock)? → use `max(L,U)/blk` from parser
- Max / std missing from table — add once numbers stable
- Expand Snap/Guard benefit vs Baseline
