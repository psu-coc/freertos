#!/usr/bin/env python3
"""Independent FAR recompute from far_pilot_*_rounds.csv + UART."""
from __future__ import annotations

import csv
import re
import statistics
import sys
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "logs"


def analyze(tag: str) -> dict:
    uart = (LOG / f"{tag}_uart_capture.txt").read_text(encoding="utf-8", errors="replace")
    rows = list(csv.DictReader((LOG / f"{tag}_rounds.csv").open(newline="")))
    fars = [float(r["far"]) for r in rows]
    acts = [int(r["actual"]) for r in rows]
    exps = [int(r["expected"]) for r in rows]
    # independent FAR = actual/expected (no firmware rounding)
    indep = [a / e if e else float("nan") for a, e in zip(acts, exps)]
    m = re.search(r"FAR mean=(\d+)\.(\d{4})", uart)
    fw_mean = (int(m.group(1)) + int(m.group(2)) / 10000.0) if m else None
    td = re.findall(r"t_disabled avg=(\d+) cy", uart)
    tdis = [int(x) for x in td]
    return {
        "tag": tag,
        "n": len(rows),
        "far_fw_mean": fw_mean,
        "far_indep_mean": statistics.fmean(indep) if indep else None,
        "far_indep_min": min(indep) if indep else None,
        "far_indep_max": max(indep) if indep else None,
        "runtime_mean": statistics.fmean(int(r["runtime_ms"]) for r in rows),
        "tdis_avg_mean_cy": statistics.fmean(tdis) if tdis else None,
        "fault": ("SecureFault" in uart) or ("HardFault" in uart),
    }


def main(tags: list[str]) -> None:
    print("tag,n,far_fw_mean,far_indep_mean,far_indep_min,far_indep_max,runtime_mean_ms,tdis_avg_cy,fault")
    for tag in tags:
        a = analyze(tag)
        tdis = f"{a['tdis_avg_mean_cy']:.0f}" if a["tdis_avg_mean_cy"] is not None else "NA"
        print(
            f"{a['tag']},{a['n']},{a['far_fw_mean']:.4f},{a['far_indep_mean']:.4f},"
            f"{a['far_indep_min']:.4f},{a['far_indep_max']:.4f},"
            f"{a['runtime_mean']:.1f},{tdis},{a['fault']}"
        )


if __name__ == "__main__":
    main(sys.argv[1:] if len(sys.argv) > 1 else [])
