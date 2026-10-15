# t_disabled raw dump — sweep notes

Branch: `snapguard-paper-reviewer`  
Flags: `Secure_nsclib/attest_overhead_cfg.h`

## Enable (already on)

```c
#define TDIS_RAW_DUMP  1
#define TDIS_RAW_CAP   2048U   /* Guard @ B=128 → 2×1024 windows */
```

After paper numbers are locked, set `TDIS_RAW_DUMP` back to `0` (UART volume).

## Modes (rebuild Secure + NonSecure each time)

**Order:** Baseline → Snap → Guard (not Guard-first).

| Design   | `USE_SAU_APPROACH` | `USE_SNAP` |
|----------|--------------------|------------|
| Baseline | 0                  | 0          |
| Snap     | 0                  | 1          |
| Guard    | 1                  | 0          |

Old `snapguard_remeasure_log.md` has Baseline **mean/max only** — **no raw → no std**. Re-run with `TDIS_RAW_DUMP=1`.

Sweep `ATTEST_BLOCK_SIZE`: `128, 256, 512, 1024, 2048, 4096`  
Firmware always runs **10 rounds** per flash.  
Current cfg is set to **Baseline @ B=128**.

## Capture

1. Flash Secure then NonSecure ELF.
2. Open ST-Link VCP; enable **session logging** (PuTTY / Tera Term).
3. Reset; wait for all 10 rounds + `TDIS_RAW` / `TDIS_RAW_END` blocks.
4. Save as e.g. `logs/guard_B4096.txt`.

UART line format:

```
TDIS_RAW round=1 B=4096 count=64
L,72
U,71
...
TDIS_RAW_END
```

- `L` / `U` — Guard SAU lock / unlock  
- `W` — Baseline or Snap single IRQ-off window  

## Offline stats

```bash
python thesis/01_snapguard/parse_tdis_raw.py logs/guard_B4096.txt
```

Reports mean / min / max / **std** (µs @ 110 MHz), plus Guard `max(L,U)` per block (advisor definition).

## Advisor oak answers (after data)

- Guard `tdis` = max(lock, unlock) per block — confirm with `max(L,U)/blk` column.  
- Put **max** (and ideally **std**) in Table or a second row; prose already cites max for Guard @ B=4096.  
- Expand benefit of Snap/Guard vs Baseline once numbers are stable.
