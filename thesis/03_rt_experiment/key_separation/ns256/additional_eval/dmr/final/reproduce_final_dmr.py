#!/usr/bin/env python3
"""Recompute formal job-identity DMR from frozen raw captures.

Reads only:
  additional_eval/dmr/raw/dmr_10Hz10Hz_*          (recovered 10 Hz job identity)
  additional_eval/dmr/recapture/raw/dmr_Wheel*    (tick0 TIM2 anchor recapture)

Writes FINAL_DMR_RESULTS.csv next to this script. Does not flash or edit raw files.
"""
from __future__ import annotations

import csv
import hashlib
import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
DMR = HERE.parent
RAW_10 = DMR / "raw"
RAW_WHEEL = DMR / "recapture" / "raw"
OUT = HERE / "FINAL_DMR_RESULTS.csv"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def parse_tag(tag: str):
    m = re.match(
        r"dmr_(10Hz10Hz|Wheel100Hz|Wheel1000Hz)_(noattest|original|prng|fpe)"
        r"(?:_(B\d+))?_r(\d+)$",
        tag,
    )
    if not m:
        raise ValueError(tag)
    return m.group(1), m.group(2), (m.group(3) or "-").lstrip("B") if m.group(3) else "-", int(m.group(4))


def classify(rows: list[dict], origin: int, tick0: int, tend: int, t0: int, first_w: int) -> dict:
    """Obligation W keeps release origin+(W-tick0)*1000. A finish is never moved."""
    period = int(rows[0]["period_us"])
    p_ticks = period // 1000
    by_w: dict[int, dict] = {}
    dup = r_mis = 0
    ordered = sorted(rows, key=lambda r: int(r["sched_wake_tick"]))
    prev = None
    step_bad = 0
    for r in ordered:
        w = int(r["sched_wake_tick"])
        if w in by_w:
            dup += 1
        by_w[w] = r
        expect = origin + (w - tick0) * 1000
        if int(r["r_us"]) != expect or int(r["d_us"]) != expect + period:
            r_mis += 1
        if prev is not None and (w - prev) != p_ticks:
            step_bad += 1
        prev = w
    ontime = late = incomplete = unexec = 0
    w = first_w
    seen = 0
    while seen < 20000:
        seen += 1
        r_us = origin + (w - tick0) * 1000
        d_us = r_us + period
        if d_us > tend:
            break
        if d_us <= t0:
            w += p_ticks
            continue
        row = by_w.get(w)
        if row is None:
            unexec += 1
        else:
            f = int(row["f_us"])
            s = int(row["s_us"])
            if f == 0 or s >= tend:
                incomplete += 1
            elif f <= d_us:
                ontime += 1
            else:
                late += 1
        w += p_ticks
    censored = 0
    for ww in by_w:
        d_us = origin + (ww - tick0) * 1000 + period
        if d_us > tend:
            censored += 1
    eligible = ontime + late + incomplete + unexec
    miss = late + incomplete + unexec
    timing = "\n".join(
        f"{int(r['sched_wake_tick'])},{int(r['s_us'])},{int(r['f_us'])}" for r in ordered
    ).encode()
    return {
        "period_us": period,
        "eligible": eligible,
        "ontime": ontime,
        "late": late,
        "incomplete": incomplete,
        "unexec": unexec,
        "censored": censored,
        "miss": miss,
        "dmr": (miss / eligible) if eligible else None,
        "dup": dup,
        "r_mis": r_mis,
        "w_step_bad": step_bad,
        "timing_sha256": hashlib.sha256(timing).hexdigest(),
    }


def uart_text(cell: Path, tag: str) -> str:
    return (cell / f"{tag}_uart_capture.txt").read_text(encoding="utf-8", errors="replace")


def load_10hz(cell: Path, tag: str) -> list[dict]:
    text = uart_text(cell, tag)
    obs = re.search(r"T0=(\d+)\s+Tend=(\d+)", text)
    tick = re.search(r"tick0=(\d+)", text)
    if not obs or not tick:
        raise ValueError(f"missing OBS anchor {tag}")
    t0, tend, tick0 = int(obs.group(1)), int(obs.group(2)), int(tick.group(1))
    rows = list(csv.DictReader((cell / f"{tag}_formal_dmr.csv").open(encoding="utf-8")))
    out = []
    for task in ("sonar", "wheel"):
        task_rows = [r for r in rows if r["task"] == task]
        if not task_rows:
            raise ValueError(f"no {task} rows {tag}")
        stats = classify(task_rows, t0, tick0, tend, t0, tick0)
        stats.update({
            "task": task,
            "anchor": "T0_logged_release",
            "T0": t0,
            "Tend": tend,
            "tick0": tick0,
            "tick0_tim2": "",
            "boot": "",
            "continuity_err_us": "",
            "span": tend - t0,
        })
        out.append(stats)
    return out


def load_wheel(cell: Path, tag: str) -> list[dict]:
    text = uart_text(cell, tag)
    anc = re.search(
        r"DMR_ANCHOR method=(\S+) tick0=(\d+) tick0_tim2_us=(\d+) .* T0_us=(\d+) Tend_us=(\d+)",
        text,
    )
    if not anc or anc.group(1) != "systick_edge":
        raise ValueError(f"missing systick_edge anchor {tag}")
    tick0 = int(anc.group(2))
    origin = int(anc.group(3))
    t0 = int(anc.group(4))
    tend = int(anc.group(5))
    clk = re.search(r"continuity_err_us=(-?\d+)", text)
    boot = re.search(r"DMR_SESSION boot=(\d+)", text)
    entry = re.search(r"wheel_tick_entry=(\d+)", text)
    if not clk or not boot or not entry:
        raise ValueError(f"missing clock/session/entry {tag}")
    rows = list(csv.DictReader((cell / f"{tag}_formal_dmr.csv").open(encoding="utf-8")))
    wheel = [r for r in rows if r["task"] == "wheel"]
    if not wheel:
        raise ValueError(f"no wheel rows {tag}")
    stats = classify(wheel, origin, tick0, tend, t0, int(entry.group(1)))
    stats.update({
        "task": "wheel",
        "anchor": "tick0_tim2_systick_edge",
        "T0": t0,
        "Tend": tend,
        "tick0": tick0,
        "tick0_tim2": origin,
        "boot": int(boot.group(1)),
        "continuity_err_us": int(clk.group(1)),
        "span": tend - t0,
        "entry_delta": int(entry.group(1)) - tick0,
    })
    return [stats]


def main() -> None:
    records = []
    cells = []
    for p in sorted(RAW_10.glob("dmr_10Hz10Hz_*")):
        if not p.is_dir() or "_inj" in p.name:
            continue
        cells.append((p, "10hz"))
    for p in sorted(RAW_WHEEL.glob("dmr_Wheel*")):
        if p.is_dir():
            cells.append((p, "wheel"))
    for cell, kind in cells:
        tag = cell.name
        profile, mode, block, run = parse_tag(tag)
        stats_list = load_10hz(cell, tag) if kind == "10hz" else load_wheel(cell, tag)
        csv_path = cell / f"{tag}_formal_dmr.csv"
        uart_path = cell / f"{tag}_uart_capture.txt"
        for stats in stats_list:
            blocked = []
            if stats["dup"] or stats["r_mis"] or stats["w_step_bad"]:
                blocked.append("identity")
            if stats["eligible"] <= 0 or stats["dmr"] is None:
                blocked.append("denominator")
            if stats["span"] not in (3_000_000, 30_000_000):
                blocked.append("span")
            if kind == "wheel" and abs(int(stats["continuity_err_us"])) > 50 and mode == "noattest":
                blocked.append("continuity")
            if kind == "wheel" and stats.get("entry_delta", 0) > 5:
                blocked.append("entry")
            records.append({
                "profile": profile,
                "task": stats["task"],
                "mode": mode,
                "B": block,
                "run": run,
                "source_csv": str(csv_path),
                "csv_sha256": sha256(csv_path),
                "uart_sha256": sha256(uart_path),
                "timing_sha256": stats["timing_sha256"],
                "anchor": stats["anchor"],
                "T0": stats["T0"],
                "Tend": stats["Tend"],
                "tick0": stats["tick0"],
                "tick0_tim2": stats["tick0_tim2"],
                "boot": stats["boot"],
                "continuity_err_us": stats["continuity_err_us"],
                "eligible": stats["eligible"],
                "ontime": stats["ontime"],
                "late": stats["late"],
                "unexec": stats["unexec"],
                "incomplete": stats["incomplete"],
                "censored": stats["censored"],
                "miss": stats["miss"],
                "dmr": f"{stats['dmr']:.6f}",
                "dup": stats["dup"],
                "r_mis": stats["r_mis"],
                "w_step_bad": stats["w_step_bad"],
                "blocked": ";".join(blocked),
            })
    groups = defaultdict(list)
    for r in records:
        groups[(r["profile"], r["task"], r["mode"], r["B"])].append(r)
    for rows in groups.values():
        timings = {r["timing_sha256"] for r in rows}
        for r in rows:
            r["n_capture_files"] = len(rows)
            r["n_distinct_timing"] = len(timings)
            r["table_row"] = "yes" if (len(timings) > 1 or r["run"] == 1) else "sibling"
    fields = list(records[0].keys())
    with OUT.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(records)
    blocked = [r for r in records if r["blocked"]]
    print(f"rows={len(records)} configs={len(groups)} blocked={len(blocked)} -> {OUT}")
    shown = [r for r in records if r["table_row"] == "yes"]
    print(f"table_rows={len(shown)}")
    for r in shown:
        if r["profile"] != "10Hz10Hz" and r["B"] not in ("-", "64", "1024", "2048", "4096"):
            continue
        if r["mode"] not in ("noattest", "original", "fpe"):
            continue
        print(
            f"{r['profile']:12} {r['task']:5} {r['mode']:8} B={r['B']:>4} r={r['run']} "
            f"dmr={r['dmr']} elig={r['eligible']} late={r['late']} unexec={r['unexec']} "
            f"cens={r['censored']} traces={r['n_distinct_timing']}"
        )


if __name__ == "__main__":
    main()
