#!/usr/bin/env python3
"""Parse UART TDIS_RAW dumps → mean/min/max/std (and Guard max(L,U) per block).

Usage:
  python parse_tdis_raw.py uart_log.txt
  python parse_tdis_raw.py uart_log.txt --mhz 110

Log markers (firmware):
  TDIS_RAW round=N B=B count=C
  L,cycles | U,cycles | W,cycles
  TDIS_RAW_END

Tags: L/U = Guard SAU lock/unlock; W = Baseline/Snap single IRQ-off window.
For Guard paper definition (advisor): per-block tdis = max(L, U) of each pair.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


HDR_RE = re.compile(
    r"^TDIS_RAW\s+round=(\d+)\s+B=(\d+)\s+count=(\d+)\s*$"
)
ROW_RE = re.compile(r"^([LUW]),(\d+)\s*$")


@dataclass
class RoundDump:
    round_id: int
    block_size: int
    samples: List[Tuple[str, int]] = field(default_factory=list)


def parse_log(text: str) -> List[RoundDump]:
    rounds: List[RoundDump] = []
    cur: Optional[RoundDump] = None
    for line in text.splitlines():
        line = line.strip()
        m = HDR_RE.match(line)
        if m:
            cur = RoundDump(int(m.group(1)), int(m.group(2)))
            rounds.append(cur)
            continue
        if line == "TDIS_RAW_END":
            cur = None
            continue
        if cur is None:
            continue
        rm = ROW_RE.match(line)
        if rm:
            cur.samples.append((rm.group(1), int(rm.group(2))))
    return rounds


def stats(xs: List[float]) -> Dict[str, float]:
    n = len(xs)
    if n == 0:
        return {"n": 0, "mean": 0.0, "min": 0.0, "max": 0.0, "std": 0.0}
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n  # population std (all windows)
    return {
        "n": float(n),
        "mean": mean,
        "min": min(xs),
        "max": max(xs),
        "std": math.sqrt(var),
    }


def cy_to_us(cy: float, mhz: float) -> float:
    return cy / mhz


def guard_max_lu(samples: List[Tuple[str, int]]) -> List[int]:
    """Pair consecutive L,U → max(L,U) per block. Unpaired samples skipped."""
    out: List[int] = []
    i = 0
    while i + 1 < len(samples):
        t0, c0 = samples[i]
        t1, c1 = samples[i + 1]
        if t0 == "L" and t1 == "U":
            out.append(max(c0, c1))
            i += 2
        else:
            i += 1
    return out


def fmt_row(label: str, st: Dict[str, float], mhz: float) -> str:
    return (
        f"  {label:16s} n={int(st['n']):5d}  "
        f"mean={cy_to_us(st['mean'], mhz):10.4f} us  "
        f"min={cy_to_us(st['min'], mhz):10.4f} us  "
        f"max={cy_to_us(st['max'], mhz):10.4f} us  "
        f"std={cy_to_us(st['std'], mhz):10.4f} us"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logfile", help="UART capture with TDIS_RAW blocks")
    ap.add_argument("--mhz", type=float, default=110.0, help="CPU MHz (default 110)")
    args = ap.parse_args()

    text = open(args.logfile, "r", encoding="utf-8", errors="replace").read()
    rounds = parse_log(text)
    if not rounds:
        print("No TDIS_RAW blocks found.", file=sys.stderr)
        return 1

    by_b: Dict[int, List[RoundDump]] = defaultdict(list)
    for r in rounds:
        by_b[r.block_size].append(r)

    for b in sorted(by_b):
        dumps = by_b[b]
        print(f"\n=== B={b}  rounds={len(dumps)}  @ {args.mhz:g} MHz ===")
        all_w: List[float] = []
        all_l: List[float] = []
        all_u: List[float] = []
        all_max_lu: List[float] = []
        for d in dumps:
            tags = {t for t, _ in d.samples}
            for t, c in d.samples:
                all_w.append(float(c))
                if t == "L":
                    all_l.append(float(c))
                elif t == "U":
                    all_u.append(float(c))
            if "L" in tags or "U" in tags:
                all_max_lu.extend(float(x) for x in guard_max_lu(d.samples))

        print(fmt_row("all windows", stats(all_w), args.mhz))
        if all_l:
            print(fmt_row("Guard L only", stats(all_l), args.mhz))
        if all_u:
            print(fmt_row("Guard U only", stats(all_u), args.mhz))
        if all_max_lu:
            print(fmt_row("max(L,U)/blk", stats(all_max_lu), args.mhz))
            print("  (paper Guard tdis definition: max of lock/unlock per block)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
