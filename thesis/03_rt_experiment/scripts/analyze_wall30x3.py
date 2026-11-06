#!/usr/bin/env python3
"""Aggregate wall30 x3 runs: per-run metrics + mean/std.

PROVISIONAL / DO NOT CITE: any DMR / missed_slots from map_releases()
(floor+spill) are NOT formal deadline misses for the paper.
Jitter (David) and frequency fields may be used when s_exec columns are correct.

Does not flash or open COM ports.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from rt_metrics_wall30 import JobStamp, map_releases, summarize_jitter

P_US = 100_000
N_NOM = 300


def parse_obs_header(text: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for key in (
        "T0",
        "Tend",
        "epoch",
        "warmup_us",
        "duration_us",
        "nominal_jobs",
        "P_us",
        "sonar_D_us",
        "wheel_D_us",
        "tick_delta",
        "sonar_starts",
        "wheel_starts",
    ):
        m = re.search(rf"{key}=(\d+)", text)
        if m:
            out[key] = int(m.group(1))
    return out


def load_sonar_csv(path: Path) -> List[JobStamp]:
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(
                JobStamp(
                    s_exec=int(row["actual_start_us"]),
                    finish=int(row["finish_us"]),
                )
            )
    return rows


def load_wheel_csv(path: Path) -> List[JobStamp]:
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        for row in r:
            # Prefer new names; fall back to legacy column names
            s = row.get("s_exec_us") or row.get("window_end_us")
            wo = row.get("window_open_us") or row.get("actual_start_us")
            rows.append(
                JobStamp(
                    s_exec=int(s),
                    finish=int(row["finish_us"]),
                    window_open=int(wo) if wo is not None else None,
                )
            )
    return rows


def mean_std(xs: List[float]) -> Tuple[float, float]:
    if not xs:
        return float("nan"), float("nan")
    m = sum(xs) / len(xs)
    if len(xs) == 1:
        return m, 0.0
    v = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return m, math.sqrt(v)


def score_run(
    uart: Path,
    sonar_csv: Path,
    wheel_csv: Path,
) -> Dict:
    text = uart.read_text(encoding="utf-8", errors="replace")
    hdr = parse_obs_header(text)
    t0 = hdr["T0"]
    tend = hdr.get("Tend", t0 + hdr.get("duration_us", 30_000_000))
    sonar_d = hdr.get("sonar_D_us", P_US)
    wheel_d = hdr.get("wheel_D_us", P_US + 1000)
    s_jobs = load_sonar_csv(sonar_csv)
    w_jobs = load_wheel_csv(wheel_csv)
    ss = map_releases(s_jobs, t0, tend, P_US, N_NOM, sonar_d)
    ww = map_releases(w_jobs, t0, tend, P_US, N_NOM, wheel_d)
    sj_mean, sj_std, sj_min, sj_max = summarize_jitter(ss.jitters_us)
    wj_mean, wj_std, wj_min, wj_max = summarize_jitter(ww.jitters_us)
    return {
        "tick_delta": hdr.get("tick_delta"),
        "sonar_hz": ss.obs_hz,
        "sonar_ratio": ss.freq_ratio,
        "sonar_missed_slots": ss.n_missed_slots,
        "sonar_dmr": ss.dmr,
        "sonar_j_mean_us": sj_mean,
        "sonar_j_std_us": sj_std,
        "sonar_j_max_us": sj_max,
        "wheel_hz": ww.obs_hz,
        "wheel_ratio": ww.freq_ratio,
        "wheel_missed_slots": ww.n_missed_slots,
        "wheel_dmr": ww.dmr,
        "wheel_j_mean_us": wj_mean,
        "wheel_j_std_us": wj_std,
        "wheel_j_max_us": wj_max,
        "hdr": hdr,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", type=Path, required=True)
    ap.add_argument("--config", required=True, help="e.g. noattest or B1024")
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    prefix = (
        "phase2r_noattest"
        if args.config == "noattest"
        else f"phase3r_{args.config}"
    )
    lines = [f"# wall30x3 aggregate — {args.config}", ""]
    per: List[Dict] = []
    for i in range(1, args.runs + 1):
        uart = args.log_dir / f"{prefix}_r{i}_uart_capture.txt"
        sonar = args.log_dir / f"{prefix}_r{i}_sonar_timing.csv"
        wheel = args.log_dir / f"{prefix}_r{i}_wheel_timing.csv"
        if not (uart.exists() and sonar.exists() and wheel.exists()):
            lines.append(f"- run{i}: MISSING files under {prefix}_r{i}_*")
            continue
        m = score_run(uart, sonar, wheel)
        per.append(m)
        lines.append(f"## run{i}")
        lines.append(
            f"- sonar Hz={m['sonar_hz']:.4f} ratio={m['sonar_ratio']:.4f} "
            f"Jmean={m['sonar_j_mean_us']:.1f}us Jmax={m['sonar_j_max_us']}"
        )
        lines.append(
            f"- wheel Hz={m['wheel_hz']:.4f} ratio={m['wheel_ratio']:.4f} "
            f"Jmean={m['wheel_j_mean_us']:.1f}us Jmax={m['wheel_j_max_us']}"
        )
        lines.append(
            "- formal_DMR=QUARANTINED (floor/spill disabled; DO NOT CITE)"
        )
        lines.append(f"- tick_delta={m['tick_delta']}")
        lines.append("")

    if per:
        lines.append("## mean ± std (n=%d)" % len(per))
        for key in (
            "sonar_hz",
            "sonar_ratio",
            "sonar_dmr",
            "sonar_j_mean_us",
            "wheel_hz",
            "wheel_ratio",
            "wheel_dmr",
            "wheel_j_mean_us",
            "tick_delta",
        ):
            vals = []
            for p in per:
                v = p.get(key)
                if v is None:
                    continue
                vals.append(float(v))
            m, s = mean_std(vals)
            lines.append(f"- {key}: {m:.6g} ± {s:.6g}")

    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
