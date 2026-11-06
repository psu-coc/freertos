#!/usr/bin/env python3
"""Analyze one Phase3 Original SMARM UART/CSV capture. Does not invent values."""
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


def pct(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    x = (len(sorted_vals) - 1) * (p / 100.0)
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return float(sorted_vals[lo])
    w = x - lo
    return float(sorted_vals[lo] * (1.0 - w) + sorted_vals[hi] * w)


def parse_header(text: str) -> dict[str, str]:
    keys = [
        "B",
        "USE_SAU",
        "USE_SNAP",
        "attest_rounds",
        "rounds_at_stop",
        "rounds_boundary",
        "incomplete",
        "tdis_avg",
        "tdis_max",
        "last_round_ms",
        "sonar_n",
        "wheel_n",
        "sonar_ovf",
        "wheel_ovf",
        "wall_start_us",
        "wall_sensor_end_us",
        "wall_sensor_elapsed_us",
        "wall_dump_us",
        "wall_end_us",
        "wall_elapsed_us",
        "wall_target_us",
    ]
    out: dict[str, str] = {}
    for k in keys:
        m = re.search(rf"{k}=(\d+)", text)
        if m:
            out[k] = m.group(1)
    return out


def load_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def abs_devs(rows: list[dict], key: str) -> list[float]:
    vals = []
    for r in rows:
        if key not in r or r[key] == "":
            continue
        vals.append(abs(int(r[key])))
    return vals


def summarize_sensor(name: str, rows: list[dict], miss_keys: list[str]) -> dict:
    if not rows:
        return {"task": name, "n": 0, "missing_csv": True}
    interval = abs_devs(rows, "interval_dev_us")
    out_iv = []
    # output interval deviation vs nominal period inferred from median actual_interval
    periods = [int(r["actual_interval_us"]) for r in rows[1:] if r.get("actual_interval_us")]
    if periods:
        # Prefer fixed periods from experiment: caller passes via miss context
        pass
    stf = [int(r["schedule_to_finish_us"]) for r in rows if r.get("schedule_to_finish_us")]
    miss = {k: sum(1 for r in rows if int(r.get(k, "0"))) for k in miss_keys}
    interval_s = sorted(interval)
    return {
        "task": name,
        "n": len(rows),
        "miss": miss,
        "interval_abs_max_us": max(interval) if interval else None,
        "interval_abs_p95_us": pct(interval_s, 95) if interval else None,
        "schedule_to_finish_us_min": min(stf) if stf else None,
        "schedule_to_finish_us_avg": (sum(stf) // len(stf)) if stf else None,
        "schedule_to_finish_us_max": max(stf) if stf else None,
        "note_stf": "NOT formal release-to-completion",
        "output_interval_us_min": min(int(r["output_interval_us"]) for r in rows[1:])
        if len(rows) > 1
        else None,
        "output_interval_us_max": max(int(r["output_interval_us"]) for r in rows[1:])
        if len(rows) > 1
        else None,
        "output_interval_dev_abs_max_us": None,
        "output_interval_dev_abs_p95_us": None,
    }


def add_output_interval_dev(summary: dict, rows: list[dict], period_us: int) -> None:
    if len(rows) < 2:
        return
    devs = []
    for r in rows[1:]:
        oi = int(r["output_interval_us"])
        devs.append(abs(oi - period_us))
    summary["output_interval_dev_abs_max_us"] = max(devs) if devs else None
    summary["output_interval_dev_abs_p95_us"] = pct(sorted(devs), 95) if devs else None
    summary["output_nominal_period_us"] = period_us


def extract_embedded_csv(text: str, begin: str, end: str, dest: Path) -> bool:
    i = text.find(begin)
    j = text.find(end)
    if i < 0 or j < 0 or j <= i:
        return False
    body = text[i + len(begin) : j].replace("\r\n", "\n").strip() + "\n"
    dest.write_text(body, encoding="utf-8")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", type=Path, required=True)
    ap.add_argument("--sonar-csv", type=Path, default=None)
    ap.add_argument("--wheel-csv", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--sonar-period-us", type=int, default=200_000)
    ap.add_argument("--wheel-period-us", type=int, default=1_000_000)
    args = ap.parse_args()

    text = args.uart.read_text(encoding="utf-8", errors="replace")
    hdr = parse_header(text)

    sonar_csv = args.sonar_csv
    wheel_csv = args.wheel_csv
    if sonar_csv is None:
        sonar_csv = args.uart.with_name(args.uart.stem.replace("_uart_capture", "") + "_sonar_timing.csv")
        if "uart_capture" in args.uart.name:
            sonar_csv = args.uart.with_name(args.uart.name.replace("_uart_capture.txt", "_sonar_timing.csv"))
    if wheel_csv is None:
        wheel_csv = args.uart.with_name(args.uart.name.replace("_uart_capture.txt", "_wheel_timing.csv"))

    # Extract from UART if CSV files missing
    if not sonar_csv.exists():
        extract_embedded_csv(text, "BEGIN sonar_timing.csv\r\n", "END sonar_timing.csv", sonar_csv) or extract_embedded_csv(
            text, "BEGIN sonar_timing.csv\n", "END sonar_timing.csv", sonar_csv
        )
    if not wheel_csv.exists():
        extract_embedded_csv(text, "BEGIN wheel_timing.csv\r\n", "END wheel_timing.csv", wheel_csv) or extract_embedded_csv(
            text, "BEGIN wheel_timing.csv\n", "END wheel_timing.csv", wheel_csv
        )

    sonar = load_csv(sonar_csv)
    wheel = load_csv(wheel_csv)
    s_sum = summarize_sensor("sonar", sonar, ["deadline_miss"])
    add_output_interval_dev(s_sum, sonar, args.sonar_period_us)
    w_sum = summarize_sensor("wheel", wheel, ["miss_compute", "miss_output"])
    add_output_interval_dev(w_sum, wheel, args.wheel_period_us)

    # Legacy B=1024 header used wall_elapsed_us (included attest drain).
    wall_sensor = hdr.get("wall_sensor_elapsed_us") or hdr.get("wall_elapsed_us")
    legacy_wall = "wall_sensor_elapsed_us" not in hdr and "wall_elapsed_us" in hdr

    lines = []
    lines.append(f"# Phase3 analysis — {args.uart.name}")
    lines.append("")
    lines.append("## Dump header")
    for k in sorted(hdr):
        lines.append(f"- {k}={hdr[k]}")
    if legacy_wall:
        lines.append(
            "- NOTE: legacy capture — `wall_elapsed_us` includes post-sensor attest drain; "
            "`rounds_at_stop` / `rounds_boundary` not present. Do not invent splits."
        )
    lines.append("")
    lines.append("## Attestation")
    lines.append(f"- attest_rounds (completed NSC returns)={hdr.get('attest_rounds', 'MISSING')}")
    lines.append(f"- rounds_at_stop={hdr.get('rounds_at_stop', 'MISSING')}")
    lines.append(f"- rounds_boundary={hdr.get('rounds_boundary', 'MISSING')}")
    lines.append(f"- incomplete={hdr.get('incomplete', 'MISSING')}")
    lines.append(f"- tdis_avg={hdr.get('tdis_avg', 'MISSING')} tdis_max={hdr.get('tdis_max', 'MISSING')}")
    lines.append(f"- last_round_ms={hdr.get('last_round_ms', 'MISSING')}")
    lines.append(f"- wall_sensor_elapsed_us={wall_sensor if wall_sensor else 'MISSING'}")
    lines.append("")
    lines.append("## Sonar")
    for k, v in s_sum.items():
        lines.append(f"- {k}={v}")
    lines.append("")
    lines.append("## Wheel")
    for k, v in w_sum.items():
        lines.append(f"- {k}={v}")
    lines.append("")
    lines.append("## Errors / gaps")
    gaps = []
    if not sonar:
        gaps.append("sonar CSV missing/empty")
    if not wheel:
        gaps.append("wheel CSV missing/empty")
    if hdr.get("sonar_ovf", "0") != "0":
        gaps.append(f"sonar_ovf={hdr.get('sonar_ovf')}")
    if hdr.get("wheel_ovf", "0") != "0":
        gaps.append(f"wheel_ovf={hdr.get('wheel_ovf')}")
    if hdr.get("incomplete") == "1":
        gaps.append("attest drain incomplete=1")
    if not gaps:
        gaps.append("none observed in available fields")
    for g in gaps:
        lines.append(f"- {g}")

    report = "\n".join(lines) + "\n"
    out = args.out or args.uart.with_name(args.uart.name.replace("_uart_capture.txt", "_SUMMARY.md"))
    out.write_text(report, encoding="utf-8")
    print(report)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
