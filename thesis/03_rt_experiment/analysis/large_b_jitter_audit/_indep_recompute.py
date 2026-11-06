#!/usr/bin/env python3
"""One-shot independent David-jitter recompute for LARGE_B_JITTER_AUDIT.
Does not import analyze_bsweep_wall30 or other campaign helpers."""
from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

LOG = Path(__file__).resolve().parents[2] / "logs"
P = 100_000
OUT = Path(__file__).resolve().parent


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return ys[f] * (c - k) + ys[c] * (k - f)


def load_wheel(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            s = r.get("s_exec_us") or r.get("window_end_us")
            rows.append(
                {
                    "job": int(r["job_id"]),
                    "s": int(s),
                    "open": int(r["window_open_us"]),
                    "fin": int(r["finish_us"]),
                    "iter": int(r["iter_ord"]),
                    "wake": int(r["sched_wake_tick"]),
                    "tbd": int(r["tick_before_delay"]),
                    "tad": int(r["tick_after_delay"]),
                    "dly": int(r["delay_until_status"]),
                    "imm": int(r["immediate_wake"]),
                    "short": int(r.get("short_window", 0) or 0),
                }
            )
    return rows


def load_sonar(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            rows.append(
                {
                    "job": int(r["job_id"]),
                    "s": int(r["actual_start_us"]),
                    "fin": int(r["finish_us"]),
                    "iter": int(r["iter_ord"]),
                    "wake": int(r["sched_wake_tick"]),
                    "tbd": int(r["tick_before_delay"]),
                    "tad": int(r["tick_after_delay"]),
                    "dly": int(r["delay_until_status"]),
                    "imm": int(r["immediate_wake"]),
                }
            )
    return rows


def summarize(xs: list[float]) -> dict:
    return {
        "n": len(xs),
        "mean": statistics.fmean(xs),
        "med": statistics.median(xs),
        "p95": pct(xs, 95),
        "mx": max(xs),
        "mn": min(xs),
        "sd": statistics.pstdev(xs) if len(xs) > 1 else 0.0,
    }


def analyze(tag: str, rows: list[dict]) -> dict:
    n = len(rows)
    starts = [r["s"] for r in rows]
    mono = all(starts[i] < starts[i + 1] for i in range(n - 1))
    dups = n - len(set(starts))
    wraps = sum(1 for i in range(n - 1) if starts[i + 1] < starts[i])

    # Primary: drop first in-window stamp; pairs among remaining
    rem = starts[1:]
    intervals = [rem[i] - rem[i - 1] for i in range(1, len(rem))]
    Js = [abs(iv - P) for iv in intervals]
    signed = [iv - P for iv in intervals]

    all_iv = [starts[i] - starts[i - 1] for i in range(1, n)]
    Js_sens = [abs(iv - P) for iv in all_iv]

    mult: dict[int, int] = {}
    for iv in intervals:
        m = round(iv / P)
        mult[m] = mult.get(m, 0) + 1

    wakes = [r["wake"] for r in rows]
    wake_steps = [wakes[i] - wakes[i - 1] for i in range(1, n)]
    wake_ok = all(w == 100 for w in wake_steps) if wake_steps else True

    # Gap pattern: consecutive stretches > 1.5P
    stretch_runs = []
    run = 0
    for iv in intervals:
        if iv > 1.5 * P:
            run += 1
        else:
            if run:
                stretch_runs.append(run)
            run = 0
    if run:
        stretch_runs.append(run)

    # TIM2 vs wake: for each job after first, wall delta vs tick delta*1000
    wall_vs_tick = []
    for i in range(1, n):
        wall = starts[i] - starts[i - 1]
        tick = wakes[i] - wakes[i - 1]
        wall_vs_tick.append(wall - tick * 1000)

    return {
        "tag": tag,
        "n": n,
        "hz": n / 30.0,
        "mono": mono,
        "dups": dups,
        "wraps": wraps,
        "prim": summarize(Js) if Js else None,
        "sens": summarize(Js_sens) if Js_sens else None,
        "signed_mean": statistics.fmean(signed) if signed else None,
        "signed_pos": sum(1 for x in signed if x > 0),
        "signed_neg": sum(1 for x in signed if x < 0),
        "signed_zero": sum(1 for x in signed if x == 0),
        "signed_sum": sum(signed) if signed else 0,
        "mult": dict(sorted(mult.items())),
        "wake_ok": wake_ok,
        "wake_step_set": sorted(set(wake_steps)),
        "n_imm": sum(1 for r in rows if r["imm"]),
        "imm_jobs": [r["job"] for r in rows if r["imm"]],
        "dly_neg4": sum(1 for r in rows if r["dly"] == -4),
        "dly_0": sum(1 for r in rows if r["dly"] == 0),
        "mean_iv_all": statistics.fmean(all_iv) if all_iv else float("nan"),
        "span": starts[-1] - starts[0] if n > 1 else 0,
        "first_s": starts[0],
        "last_s": starts[-1],
        "starts": starts,
        "intervals": intervals,
        "signed": signed,
        "Js": Js,
        "rows": rows,
        "stretch_runs": stretch_runs,
        "wall_minus_tick_ms_mean": statistics.fmean(wall_vs_tick) if wall_vs_tick else None,
        "job_ids_ok": [r["job"] for r in rows] == list(range(n)),
        "iter_ok": [r["iter"] for r in rows] == list(range(n)),
    }


def table15(a: dict, label: str) -> str:
    """15 consecutive mid-run starts (skip first 5 after startup)."""
    starts = a["starts"]
    rows = a["rows"]
    # pick mid window: start at index 20 if possible else 5
    i0 = 20 if len(starts) > 40 else (5 if len(starts) > 20 else 1)
    lines = [
        f"### {label}: 15 consecutive post-DelayUntil starts (from index {i0})",
        "",
        "| k | job | s_exec_us | Δs_us | J_k=|Δ−P| | wake_tick | dly | imm |",
        "|---|-----|-----------|-------|-------------|-----------|-----|-----|",
    ]
    for k in range(15):
        i = i0 + k
        if i >= len(starts):
            break
        if i == 0:
            ds = ""
            jk = ""
        else:
            ds = starts[i] - starts[i - 1]
            jk = abs(ds - P)
        r = rows[i]
        lines.append(
            f"| {i} | {r['job']} | {starts[i]} | {ds} | {jk} | {r['wake']} | {r['dly']} | {r['imm']} |"
        )
    return "\n".join(lines)


def main() -> None:
    configs = {
        "B1024": [f"phase3r_B1024_r{i}" for i in (1, 2, 3)],
        "B2048": [f"phase3r_B2048_r{i}" for i in (1, 2, 3)],
        "B4096": [f"phase3r_B4096_r{i}" for i in (1, 2, 3)],
    }
    published = {}
    pub_path = (
        Path(__file__).resolve().parents[1]
        / "wall30_bsweep_original_recompute"
        / "per_run_metrics.csv"
    )
    with pub_path.open(newline="") as f:
        for r in csv.DictReader(f):
            published[r["tag"]] = r

    out_lines: list[str] = []
    out_lines.append("# Independent recompute dump\n")

    for task, loader in (("wheel", load_wheel), ("sonar", load_sonar)):
        out_lines.append(f"\n## {task.upper()} Original\n")
        for B, tags in configs.items():
            for tag in tags:
                path = LOG / f"{tag}_{task}_timing.csv"
                a = analyze(tag, loader(path))
                p = a["prim"]
                pub = published.get(tag, {})
                if task == "wheel":
                    pub_j = float(pub.get("wheel_j_mean", "nan"))
                    pub_hz = float(pub.get("wheel_hz", "nan"))
                    pub_n = int(float(pub.get("wheel_n", "nan")))
                else:
                    pub_j = float(pub.get("sonar_j_mean", "nan"))
                    pub_hz = float(pub.get("sonar_hz", "nan"))
                    pub_n = int(float(pub.get("sonar_n", "nan")))
                dj = p["mean"] - pub_j
                out_lines.append(
                    f"- **{tag}** n={a['n']} (pub {pub_n}) Hz={a['hz']:.6f} (pub {pub_hz:.6f}) "
                    f"Jmean={p['mean']:.6f} med={p['med']:.1f} P95={p['p95']:.3f} max={p['mx']:.0f} nJ={p['n']} "
                    f"| Δpub_J={dj:.6f} signed_mean={a['signed_mean']:.2f} "
                    f"pos/neg/0={a['signed_pos']}/{a['signed_neg']}/{a['signed_zero']} "
                    f"signed_sum={a['signed_sum']} mult={a['mult']} "
                    f"mono={a['mono']} dups={a['dups']} wraps={a['wraps']} "
                    f"wake_ok={a['wake_ok']} steps={a['wake_step_set']} "
                    f"imm={a['n_imm']}@{a['imm_jobs']} dly-4/0={a['dly_neg4']}/{a['dly_0']} "
                    f"mean_iv={a['mean_iv_all']:.2f} span={a['span']} "
                    f"jobseq={a['job_ids_ok']} iterseq={a['iter_ok']} "
                    f"wall-tick_us_mean={a['wall_minus_tick_ms_mean']:.1f} "
                    f"stretch_runs_max={max(a['stretch_runs']) if a['stretch_runs'] else 0}"
                )

    # Representative tables (r1)
    out_lines.append("\n## Raw 15-row tables (r1)\n")
    for B in ("B1024", "B2048", "B4096"):
        tag = f"phase3r_{B}_r1"
        wa = analyze(tag, load_wheel(LOG / f"{tag}_wheel_timing.csv"))
        sa = analyze(tag, load_sonar(LOG / f"{tag}_sonar_timing.csv"))
        out_lines.append(table15(wa, f"Wheel {tag}"))
        out_lines.append("")
        out_lines.append(table15(sa, f"Sonar {tag}"))
        out_lines.append("")

        # Interval histogram detail for wheel
        out_lines.append(f"### Wheel {tag} interval histogram (primary pairs)")
        out_lines.append("")
        hist: dict[str, int] = {}
        for iv in wa["intervals"]:
            bucket = f"{round(iv/1000)}ms"
            hist[bucket] = hist.get(bucket, 0) + 1
        for k in sorted(hist, key=lambda x: int(x.replace("ms", ""))):
            out_lines.append(f"- {k}: {hist[k]}")
        out_lines.append("")

        # First 30 signed deviations wheel
        out_lines.append(f"### Wheel {tag} first 30 primary signed (iv−P)")
        out_lines.append("")
        out_lines.append("```")
        out_lines.append(", ".join(str(x) for x in wa["signed"][:30]))
        out_lines.append("```")
        out_lines.append("")

    # Expected mean interval from Hz: if n jobs over ~30s with irregular gaps
    out_lines.append("\n## Consistency: mean_interval vs 1/Hz\n")
    for B in ("B1024", "B2048", "B4096"):
        tag = f"phase3r_{B}_r1"
        wa = analyze(tag, load_wheel(LOG / f"{tag}_wheel_timing.csv"))
        # mean of ALL consecutive intervals including first
        expected_if_uniform = 1e6 / wa["hz"]  # us between starts if uniform
        out_lines.append(
            f"- {tag}: n={wa['n']} Hz={wa['hz']:.4f} ⇒ uniform_iv={expected_if_uniform:.1f}µs; "
            f"measured_mean_iv={wa['mean_iv_all']:.1f}µs; "
            f"primary_signed_mean={wa['signed_mean']:.1f}µs; "
            f"tick_delta_pub={published[tag]['tick_delta']}; "
            f"tick_delta/1000≈Hz proxy={float(published[tag]['tick_delta'])/1000:.3f}"
        )

    # Compare wheel vs sonar start-by-start for B2048 r1
    out_lines.append("\n## Wheel vs Sonar start alignment (B2048 r1)\n")
    tag = "phase3r_B2048_r1"
    w = analyze(tag, load_wheel(LOG / f"{tag}_wheel_timing.csv"))
    s = analyze(tag, load_sonar(LOG / f"{tag}_sonar_timing.csv"))
    out_lines.append(f"- wheel_n={w['n']} sonar_n={s['n']}")
    diffs = [w["starts"][i] - s["starts"][i] for i in range(min(len(w["starts"]), len(s["starts"])))]
    out_lines.append(
        f"- s_wheel−s_sonar: mean={statistics.fmean(diffs):.1f} med={statistics.median(diffs):.1f} "
        f"min={min(diffs)} max={max(diffs)}"
    )
    # Are interval sequences nearly identical?
    wi, si = w["intervals"], s["intervals"]
    ncmp = min(len(wi), len(si))
    idiff = [wi[i] - si[i] for i in range(ncmp)]
    out_lines.append(
        f"- Δinterval wheel−sonar: mean={statistics.fmean(idiff):.2f} "
        f"absmean={statistics.fmean([abs(x) for x in idiff]):.2f} "
        f"maxabs={max(abs(x) for x in idiff)} identical={sum(1 for x in idiff if x==0)}/{ncmp}"
    )

    text = "\n".join(out_lines) + "\n"
    (OUT / "INDEPENDENT_RECOMPUTE.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
