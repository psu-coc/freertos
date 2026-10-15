#!/usr/bin/env python3
"""Compare Guard lock (L) vs unlock (U) t_disabled from TDIS_RAW UART logs.

Hypothesis: mean U grows with B more than mean L (post-hash microarch effect),
while SAU work itself is O(1).

Usage:
  python parse_tdis_lu_compare.py logs/guard_B128_raw.txt logs/guard_B4096_raw.txt
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

# Reuse parser from sibling script
sys.path.insert(0, str(Path(__file__).resolve().parent))
from parse_tdis_raw import parse_log, cy_to_us  # noqa: E402


def pop_stats(xs: list[float]) -> tuple[float, float, float, float, int]:
    n = len(xs)
    if n == 0:
        return 0.0, 0.0, 0.0, 0.0, 0
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n
    return mean, math.sqrt(var), min(xs), max(xs), n


def summarize(path: Path, mhz: float = 110.0) -> dict:
    rounds = parse_log(path.read_text(encoding="utf-8", errors="replace"))
    if not rounds:
        raise SystemExit(f"No TDIS_RAW in {path}")
    B = rounds[0].block_size
    L: list[float] = []
    U: list[float] = []
    per_round = []
    for r in rounds:
        rl = [float(c) for t, c in r.samples if t == "L"]
        ru = [float(c) for t, c in r.samples if t == "U"]
        L.extend(rl)
        U.extend(ru)
        lm, ls, *_ = pop_stats(rl)
        um, us, *_ = pop_stats(ru)
        per_round.append((r.round_id, lm, um, ls, us, len(rl), len(ru)))
    return {
        "path": str(path),
        "B": B,
        "rounds": len(rounds),
        "L": pop_stats(L),
        "U": pop_stats(U),
        "all": pop_stats(L + U),
        "per_round": per_round,
        "mhz": mhz,
    }


def fmt(st, mhz):
    mean, std, mn, mx, n = st
    return (
        f"n={n:5d}  mean={cy_to_us(mean, mhz):7.4f} us "
        f"({mean:6.1f} cy)  std={cy_to_us(std, mhz):7.4f} us  "
        f"min={cy_to_us(mn, mhz):7.4f}  max={cy_to_us(mx, mhz):7.4f}"
    )


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    rows = [summarize(Path(p)) for p in sys.argv[1:]]
    for s in rows:
        print(f"\n=== {s['path']}  B={s['B']}  rounds={s['rounds']} ===")
        print(f"  LOCK   (L): {fmt(s['L'], s['mhz'])}")
        print(f"  UNLOCK (U): {fmt(s['U'], s['mhz'])}")
        print(f"  ALL windows: {fmt(s['all'], s['mhz'])}")
        print("  per-round mean L / mean U (cy):")
        for rid, lm, um, ls, us, nl, nu in s["per_round"]:
            print(
                f"    r{rid:02d}: L={lm:6.1f}±{ls:5.1f} ({nl})  "
                f"U={um:6.1f}±{us:5.1f} ({nu})  U-L={um-lm:+6.1f}"
            )

    if len(rows) >= 2:
        a, b = rows[0], rows[1]
        print("\n=== Delta B_lo -> B_hi (mean us) ===")
        print(
            f"  L: {cy_to_us(a['L'][0], a['mhz']):.4f} -> "
            f"{cy_to_us(b['L'][0], b['mhz']):.4f}  "
            f"d={cy_to_us(b['L'][0]-a['L'][0], b['mhz']):+.4f}"
        )
        print(
            f"  U: {cy_to_us(a['U'][0], a['mhz']):.4f} -> "
            f"{cy_to_us(b['U'][0], b['mhz']):.4f}  "
            f"d={cy_to_us(b['U'][0]-a['U'][0], b['mhz']):+.4f}"
        )
        dL = b["L"][0] - a["L"][0]
        dU = b["U"][0] - a["U"][0]
        if abs(dU) > abs(dL) * 1.5 and dU > 0:
            print("  → Supports post-hash hypothesis: UNLOCK grows more than LOCK.")
        elif abs(dL - dU) < max(1.0, 0.2 * abs(dL + dU) / 2):
            print("  → L and U grow similarly; shared microarch / measurement effect.")
        else:
            print("  → Mixed; inspect per-round rows and max/min tails.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
