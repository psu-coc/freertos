#!/usr/bin/env python3
"""Independent Wheel1000Hz wall3 metrics from compact CSV + UART."""
from __future__ import annotations

import csv
import math
import re
import statistics
import sys
from pathlib import Path

P = 1_000  # us
OBS_S = 3.0
LOG = Path(__file__).resolve().parents[1] / "logs"


def pct(xs: list[float], p: float) -> float:
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    f, c = math.floor(k), math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return ys[f] * (c - k) + ys[c] * (k - f)


def analyze_tag(tag: str) -> dict:
    uart = (LOG / f"{tag}_uart_capture.txt").read_text(encoding="utf-8", errors="replace")
    rows = list(csv.DictReader((LOG / f"{tag}_wheel_timing.csv").open(newline="")))
    starts = [int(r["s_exec_us"]) for r in rows]
    rem = starts[1:]
    iv = [rem[i] - rem[i - 1] for i in range(1, len(rem))]
    J = [abs(x - P) for x in iv]
    signed = [x - P for x in iv]
    dly: dict[int, int] = {}
    for r in rows:
        d = int(r["delay_until_status"])
        dly[d] = dly.get(d, 0) + 1

    def g(pat: str, cast=str):
        m = re.search(pat, uart)
        return cast(m.group(1)) if m else None

    span_us = (starts[-1] - starts[0]) if len(starts) >= 2 else 0
    hz_span = ((len(starts) - 1) / (span_us / 1e6)) if span_us > 0 else float("nan")

    return {
        "tag": tag,
        "n": len(starts),
        "hz": len(starts) / OBS_S,
        "hz_span": hz_span,
        "nJ": len(J),
        "j_mean": statistics.fmean(J) if J else float("nan"),
        "j_med": statistics.median(J) if J else float("nan"),
        "j_p95": pct(J, 95) if J else float("nan"),
        "j_max": max(J) if J else float("nan"),
        "signed_mean": statistics.fmean(signed) if signed else float("nan"),
        "pos": sum(1 for x in signed if x > 0),
        "neg": sum(1 for x in signed if x < 0),
        "exact_P": sum(1 for x in iv if x == P),
        "mono": all(starts[i] < starts[i + 1] for i in range(len(starts) - 1)),
        "dly": dly,
        "imm": sum(1 for r in rows if int(r["immediate_wake"])),
        "wheel_n_uart": g(r"wheel_n=(\d+)", int),
        "ovf": g(r"wheel_ovf=(\d+)", int),
        "tick_delta": g(r"tick_delta=(\d+)", int),
        "attest_rounds": g(r"attest_rounds=(\d+)", int),
        "tdis_avg": g(r"tdis_avg(?:_cy|_cycles)?=(\d+)", int) or g(r"tdis_avg=(\d+)", int),
        "incomplete": g(r"incomplete=(\d+)", int),
        "fault": ("SecureFault" in uart) or ("HardFault" in uart),
        "profile_ok": ("Wheel1000Hz" in uart) and ("wall3" in uart),
    }


def main(tags: list[str]) -> None:
    print(
        "tag,n,hz,hz_span,nJ,j_mean,j_med,j_p95,j_max,signed_mean,"
        "tick_delta,ovf,rounds,tdis,mono,fault,profile_ok"
    )
    for tag in tags:
        a = analyze_tag(tag)
        print(
            f"{a['tag']},{a['n']},{a['hz']:.4f},{a['hz_span']:.4f},{a['nJ']},"
            f"{a['j_mean']:.3f},{a['j_med']:.1f},{a['j_p95']:.1f},{a['j_max']:.0f},"
            f"{a['signed_mean']:.1f},{a['tick_delta']},{a['ovf']},"
            f"{a['attest_rounds']},{a['tdis_avg']},{a['mono']},{a['fault']},"
            f"{a['profile_ok']}"
        )


if __name__ == "__main__":
    main(sys.argv[1:] if len(sys.argv) > 1 else [])
