#!/usr/bin/env python3
"""Build the advisor evidence report from existing NS256 measurements.

Runs the FAR/sliding reproduction and the formal DMR reproduction with
their write targets pointed at this directory. Does not modify
post_advisor_bc_v1, dmr/final, raw logs, firmware, or the manuscript.
"""
from __future__ import annotations

import csv
import hashlib
import html
import importlib.util
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
NS256 = OUT.parents[1]
RT = NS256 / "freertos_rt"
BC_PY = NS256 / "additional_eval" / "post_advisor_bc_v1" / "reproduce_post_advisor_bc.py"
BC_DIR = BC_PY.parent
DMR_PY = NS256 / "additional_eval" / "dmr" / "final" / "reproduce_final_dmr.py"
FINAL_DMR = DMR_PY.parent / "FINAL_DMR_RESULTS.csv"
METRICS_PY = NS256.parents[1] / "scripts" / "rt_metrics_wall30.py"

FREQS = (10, 100, 1000)
MODES = ("original", "prng", "fpe")
BS = (64, 128, 256, 512, 1024, 2048, 4096)
COLORS = {"original": "#0f4c81", "prng": "#b45309", "fpe": "#6d28d9"}
LABEL = {"original": "Original SMARM", "prng": "SMARM+PRNG", "fpe": "SMARM+FPE"}

# Coverage-matrix Gate 3 samples. The matrix does not name Sonar vs Wheel
# at 10 Hz, and several jitter cells are words rather than exact means.
SAMPLES = [
    ("10Hz10Hz", "noattest", "-", 300, 10.0, "~0"),
    ("10Hz10Hz", "original", "1024", 300, 10.0, "~255"),
    ("10Hz10Hz", "original", "4096", 91, 3.03, "large"),
    ("Wheel100Hz", "noattest", "-", 3000, 100.0, "0"),
    ("Wheel100Hz", "original", "1024", 3000, 100.0, "~429"),
    ("Wheel100Hz", "original", "4096", 828, 27.6, "large"),
    ("Wheel1000Hz", "noattest", "-", 3000, 1000.0, "0"),
    ("Wheel1000Hz", "original", "1024", 2999, 999.7, "~85"),
    ("Wheel1000Hz", "original", "4096", 836, 279, "~2.6k"),
]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise SystemExit(f"no rows for {path.name}")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def index_rows(rows: list[dict], keys: tuple[str, ...]) -> dict:
    out = {}
    for row in rows:
        key = tuple(row[k] for k in keys)
        if key in out:
            raise SystemExit(f"duplicate key {key}")
        out[key] = row
    return out


def near(a: str, b: str, tol: float = 5e-7) -> bool:
    if a == b:
        return True
    try:
        return abs(float(a) - float(b)) <= tol
    except ValueError:
        return False


def diff_tables(left: list[dict], right: list[dict], keys: tuple[str, ...], fields: list[str]) -> list[str]:
    a = index_rows(left, keys)
    b = index_rows(right, keys)
    issues = []
    if set(a) != set(b):
        issues.append(f"key mismatch missing={sorted(set(a) - set(b))[:3]} extra={sorted(set(b) - set(a))[:3]}")
    for key in sorted(set(a) & set(b)):
        for field in fields:
            if not near(str(a[key].get(field, "")), str(b[key].get(field, ""))):
                issues.append(f"{key} {field}: {a[key].get(field)} != {b[key].get(field)}")
                if len(issues) > 12:
                    return issues
    return issues


def independent_far() -> tuple[list[dict], list[dict], list[str]]:
    """Recompute FAR from the archived round CSVs. Does not clip at 1."""
    per_round = []
    summary = []
    issues = []
    for freq in FREQS:
        camp = RT / f"FarSweep_NS256_{freq}Hz_A"
        cells = [(mode, b, 0) for mode in MODES for b in BS]
        cells += [("original", b, 1) for b in (1024, 4096)]
        aggregate = index_rows(read_csv(camp / "analysis" / "far_matrix_aggregate.csv"), ("mode", "B", "farref"))
        if len(aggregate) != 23:
            issues.append(f"{freq} Hz aggregate has {len(aggregate)} rows, expected 23")
        for mode, block, farref in cells:
            tag = f"ns256_far_{freq}_{mode}_B{block}" + ("_farref" if farref else "")
            path = camp / "analysis" / f"{tag}_rounds.csv"
            rows = read_csv(path)
            if len(rows) != 10:
                issues.append(f"{tag}: {len(rows)} rounds")
                continue
            originals = []
            continuous = []
            for row in rows:
                runtime_ms = int(row["runtime_ms"])
                actual = int(row["actual"])
                expected = int(row["expected"])
                if expected != (runtime_ms * freq) // 1000:
                    issues.append(f"{tag} round {row['round']}: floor expected mismatch")
                elapsed_s = runtime_ms / 1000.0
                far_original = actual / expected
                far_continuous = actual / (freq * elapsed_s)
                stored = float(row["far"])
                rounded = ((actual * 10000 + expected // 2) // expected) / 10000.0
                if abs(stored - rounded) > 1e-4:
                    issues.append(f"{tag} round {row['round']}: firmware FAR {stored} != {rounded}")
                if abs(far_original - actual / expected) > 0:
                    issues.append("original ratio was altered")
                originals.append(far_original)
                continuous.append(far_continuous)
                per_round.append(
                    {
                        "freq_hz": freq,
                        "mode": mode,
                        "B": block,
                        "farref": farref,
                        "round": int(row["round"]),
                        "runtime_ms_floor": runtime_ms,
                        "actual_count": actual,
                        "expected_floor": expected,
                        "far_original": f"{far_original:.6f}",
                        "far_continuous": f"{far_continuous:.6f}",
                    }
                )
            mean_original = statistics.fmean(originals)
            mean_continuous = statistics.fmean(continuous)
            ratio_of_sums = sum(int(r["actual"]) for r in rows) / sum(
                freq * int(r["runtime_ms"]) / 1000.0 for r in rows
            )
            agg = aggregate[(mode, str(block), str(farref))]
            if abs(mean_original - float(agg["far_indep_mean"])) > 5e-7:
                issues.append(
                    f"{tag}: mean of ratios {mean_original:.6f} != aggregate {agg['far_indep_mean']}"
                )
            summary.append(
                {
                    "freq_hz": str(freq),
                    "mode": mode,
                    "B": str(block),
                    "farref": str(farref),
                    "n_rounds": "10",
                    "far_original_mean": f"{mean_original:.6f}",
                    "far_continuous_mean": f"{mean_continuous:.6f}",
                    "ratio_of_sums": f"{ratio_of_sums:.6f}",
                    "continuous_max": f"{max(continuous):.6f}",
                    "rounds_continuous_gt_1": str(sum(v > 1 for v in continuous)),
                    "aggregate_far_indep_mean": agg["far_indep_mean"],
                    "aggregate_far_fw_mean": agg["far_fw_mean"],
                }
            )
    above = [r for r in per_round if float(r["far_continuous"]) > 1]
    if not above:
        issues.append("no continuous FAR above 1 was preserved; check for clipping")
    if any(float(r["far_continuous"]) > 1 and float(r["far_continuous"]) == 1 for r in per_round):
        issues.append("a value above 1 was stored as 1")
    return per_round, summary, issues


def synthetic_sliding(bc) -> list[str]:
    """Missing-job rules on a constructed lattice. Not a measured result."""
    period = 100000
    tend = 650000
    rows = []

    def add(tick: int, start: int, finish: int) -> None:
        rows.append(
            {
                "sched_wake_tick": str(tick),
                "period_us": str(period),
                "s_us": str(start),
                "f_us": str(finish),
            }
        )

    add(0, 1000, 2000)
    add(100, 101000, 102000)
    # tick 200 is an unexecuted obligation: no row.
    add(300, 301000, 302000)  # predecessor missing -> gap excluded
    add(400, 401000, 520000)  # consecutive with 300; finish after s_prev+2P
    add(500, 501000, 502000)  # eligible again
    stats = bc.sliding_one(rows, 0, 0, tend, 0, 0)
    issues = []
    expect = {
        "lattice": 6,
        "init_excluded": 1,
        "unexecuted": 1,
        "gap_excluded": 1,
        "sliding_eligible": 3,
        "sliding_miss": 1,
        "invalid_excluded": 0,
    }
    for key, value in expect.items():
        if stats[key] != value:
            issues.append(f"synthetic {key}={stats[key]} expected {value}")
    if stats["sliding_ratio"] != 1 / 3:
        issues.append(f"synthetic ratio {stats['sliding_ratio']}")
    # The unexecuted job must not become a success or a fabricated finish.
    if stats["sliding_eligible"] + stats["excluded_jobs"] + stats["unexecuted"] != stats["lattice"]:
        issues.append("synthetic partition failed")
    return issues


def published_jitter_ok(label: str, value: float) -> bool:
    if label == "0":
        return abs(value) < 1e-9
    if label == "~0":
        return value < 5
    if label == "large":
        return value >= 1000
    if label == "~2.6k":
        return abs(value - 2600) <= max(400, 0.15 * 2600)
    if label.startswith("~"):
        target = float(label[1:])
        return abs(value - target) <= max(30.0, 0.15 * target)
    return False


def stamp_column(fieldnames: list[str]) -> str:
    for name in ("s_exec_us", "actual_start_us"):
        if name in fieldnames:
            return name
    raise SystemExit(f"no start column in {fieldnames}")


def jitter_tables(metrics) -> tuple[list[dict], list[dict], list[str], list[str]]:
    campaigns = [
        ("10Hz10Hz", RT / "WheelSonar10Hz_NS256_A" / "raw", "ns256_10Hz10Hz", 100000, ("sonar", "wheel")),
        ("Wheel100Hz", RT / "Wheel100Hz_NS256_A" / "raw", "ns256_Wheel100Hz", 10000, ("wheel",)),
        ("Wheel1000Hz", RT / "Wheel1000Hz_NS256_A" / "raw", "ns256_Wheel1000Hz", 1000, ("wheel",)),
    ]
    per_run = []
    hard = []
    for profile, raw, prefix, period, tasks in campaigns:
        cells = [("noattest", "-")] + [(mode, str(b)) for mode in MODES for b in BS]
        for mode, block in cells:
            for run in (1, 2, 3):
                tag = f"{prefix}_noattest_r{run}" if mode == "noattest" else f"{prefix}_{mode}_B{block}_r{run}"
                cell = raw / tag
                uart_path = cell / f"{tag}_uart_capture.txt"
                if not uart_path.exists():
                    hard.append(f"missing jitter capture {tag}")
                    continue
                text = uart_path.read_text(encoding="utf-8", errors="replace")
                obs = re.search(r"T0=(\d+)\s+Tend=(\d+)", text)
                p_match = re.search(r"P_us=(\d+)", text)
                if not obs or not p_match:
                    hard.append(f"missing T0/P_us {tag}")
                    continue
                t0, tend = int(obs.group(1)), int(obs.group(2))
                if int(p_match.group(1)) != period:
                    hard.append(f"{tag}: P_us {p_match.group(1)} != {period}")
                for task in tasks:
                    path = cell / f"{tag}_{task}_timing.csv"
                    with path.open(newline="", encoding="utf-8") as handle:
                        reader = csv.DictReader(handle)
                        column = stamp_column(reader.fieldnames or [])
                        stamps = [int(row[column]) for row in reader]
                    in_window = [s for s in stamps if t0 <= s < tend]
                    manual = []
                    if len(in_window) >= 2:
                        primary = in_window[1:]
                        manual = [abs((b - a) - period) for a, b in zip(primary, primary[1:])]
                    jobs = [metrics.JobStamp(s, s) for s in stamps]
                    scored = metrics.score_regularity(jobs, t0, tend, p_us=period, n_nominal=1)
                    lib = scored.primary.jitters_us
                    if lib != manual:
                        hard.append(f"{tag} {task}: library jitter != |Δs-P| after dropping the first start")
                        continue
                    intervals = [(b - a) for a, b in zip(in_window, in_window[1:])]
                    uart = re.search(
                        rf"{task}_inter_start_us n/min/avg/max=(\d+)/(-?\d+)/(-?\d+)/(-?\d+)",
                        text,
                    )
                    uart_avg = int(uart.group(3)) if uart else ""
                    mean_interval = statistics.fmean(intervals) if intervals else ""
                    if uart and intervals:
                        if int(uart.group(1)) != len(intervals):
                            hard.append(
                                f"{tag} {task}: UART interval count {uart.group(1)} != {len(intervals)}"
                            )
                        if abs(float(uart_avg) - mean_interval) > 1.0:
                            hard.append(
                                f"{tag} {task}: UART interval avg {uart_avg} != mean Δs {mean_interval:.3f}"
                            )
                    per_run.append(
                        {
                            "profile": profile,
                            "task": task,
                            "mode": mode,
                            "B": block,
                            "run": str(run),
                            "period_us": str(period),
                            "stamp_column": column,
                            "T0": str(t0),
                            "Tend": str(tend),
                            "n_starts": str(len(in_window)),
                            "n_pairs": str(scored.primary.n_pairs),
                            "startup_excluded": str(scored.n_startup_excluded),
                            "mean_us": f"{scored.primary.mean_us:.6f}",
                            "sd_us": f"{scored.primary.sd_us:.6f}",
                            "p95_us": f"{scored.primary.p95_us:.6f}",
                            "max_us": f"{scored.primary.max_us:.6f}",
                            "obs_hz": f"{scored.obs_hz:.6f}",
                            "mean_interval_us": "" if mean_interval == "" else f"{mean_interval:.6f}",
                            "uart_interval_avg_us": uart_avg,
                            "stamp_hash": hashlib.sha256(",".join(map(str, in_window)).encode()).hexdigest(),
                        }
                    )
    grouped = defaultdict(list)
    for row in per_run:
        grouped[(row["profile"], row["task"], row["mode"], row["B"])].append(row)
    summary = []
    soft = []
    for key, rows in sorted(grouped.items()):
        hashes = {row["stamp_hash"] for row in rows}
        distinct = []
        seen = set()
        for row in rows:
            if row["stamp_hash"] not in seen:
                seen.add(row["stamp_hash"])
                distinct.append(row)
        means = [float(row["mean_us"]) for row in distinct]
        if any(int(row["n_pairs"]) == 0 for row in distinct):
            hard.append(f"{key}: undefined jitter mean with zero pairs")
            continue
        plotted = statistics.fmean(means)
        if len(hashes) == 1 and abs(plotted - float(rows[0]["mean_us"])) > 1e-9:
            hard.append(f"{key}: identical traces were not kept as one measurement")
        summary.append(
            {
                "profile": key[0],
                "task": key[1],
                "mode": key[2],
                "B": key[3],
                "period_us": rows[0]["period_us"],
                "n_files": str(len(rows)),
                "n_distinct_timing": str(len(hashes)),
                "n_starts_run1": rows[0]["n_starts"],
                "n_pairs_run1": rows[0]["n_pairs"],
                "mean_us": f"{plotted:.6f}",
                "sd_us_run1": rows[0]["sd_us"],
                "p95_us_run1": rows[0]["p95_us"],
                "max_us_run1": rows[0]["max_us"],
                "mean_min_us": f"{min(means):.6f}",
                "mean_max_us": f"{max(means):.6f}",
                "obs_hz_run1": rows[0]["obs_hz"],
                "aggregation": (
                    "single timing trace; three files not averaged as independent samples"
                    if len(hashes) == 1
                    else "unweighted mean of distinct-capture means; whisker is the min-max range"
                ),
                "statistic": "mean of |start[k]-start[k-1]-P| after dropping the first in-window start",
                "unit": "microseconds",
                "uart_interval_avg_us_run1": rows[0]["uart_interval_avg_us"],
            }
        )
    by_key = {(r["profile"], r["task"], r["mode"], r["B"]): r for r in summary}
    for profile, mode, block, n_starts, freq, label in SAMPLES:
        candidates = [r for (p, _t, m, b), r in by_key.items() if (p, m, b) == (profile, mode, block)]
        if not candidates:
            soft.append(f"MISSING sample {profile} {mode} B{block}")
            continue
        start_hits = [r for r in candidates if int(r["n_starts_run1"]) == n_starts]
        if not start_hits:
            soft.append(
                f"MISMATCH {profile} {mode} B{block}: starts "
                + ", ".join(f"{r['task']}={r['n_starts_run1']}" for r in candidates)
                + f" published {n_starts}"
            )
            continue
        hz_hits = [r for r in start_hits if abs(float(r["obs_hz_run1"]) - freq) <= max(0.2, 0.01 * freq)]
        if not hz_hits:
            soft.append(
                f"MISMATCH {profile} {mode} B{block}: obs Hz "
                + ", ".join(f"{r['task']}={r['obs_hz_run1']}" for r in pool)
                + f" published {freq}"
            )
            continue
        jitter_hits = [r for r in hz_hits if published_jitter_ok(label, float(r["mean_us"]))]
        detail = ", ".join(f"{r['task']}={float(r['mean_us']):.1f} us" for r in hz_hits)
        if not jitter_hits:
            soft.append(f"MISMATCH {profile} {mode} B{block}: published jitter {label}; recomputed {detail}")
        else:
            soft.append(f"MATCH {profile} {mode} B{block}: published {label}; {detail}")
    return per_run, summary, hard, soft


def style_axes() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.labelsize": 9,
            "axes.titlesize": 10,
            "legend.fontsize": 7.5,
            "axes.linewidth": 0.6,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        }
    )


def plot_far(summary: list[dict]) -> list[dict]:
    style_axes()
    series = []
    for freq in FREQS:
        fig, ax = plt.subplots(figsize=(3.55, 3.45))
        for mode in MODES:
            xs, old, new = [], [], []
            for block in BS:
                row = next(
                    r
                    for r in summary
                    if int(r["freq_hz"]) == freq
                    and r["mode"] == mode
                    and int(r["B"]) == block
                    and int(r["farref"]) == 0
                )
                xs.append(block)
                old.append(float(row["far_original_mean"]))
                new.append(float(row["far_continuous_mean"]))
                series.append(
                    {
                        "figure": f"fig_far_{freq}hz",
                        "series": f"{mode} original formula",
                        "x": str(block),
                        "y": row["far_original_mean"],
                    }
                )
                series.append(
                    {
                        "figure": f"fig_far_{freq}hz",
                        "series": f"{mode} continuous",
                        "x": str(block),
                        "y": row["far_continuous_mean"],
                    }
                )
            ax.plot(xs, new, color=COLORS[mode], marker="o", linewidth=1.3, label=LABEL[mode])
            ax.plot(
                xs,
                old,
                color=COLORS[mode],
                marker="o",
                linewidth=1.0,
                linestyle="none",
                markerfacecolor="white",
                markeredgewidth=1.1,
                label="_nolegend_",
            )
        refs = [r for r in summary if int(r["freq_hz"]) == freq and int(r["farref"]) == 1]
        if refs:
            ax.scatter(
                [int(r["B"]) for r in refs],
                [float(r["far_continuous_mean"]) for r in refs],
                marker="D",
                s=26,
                color="#111111",
                zorder=4,
                label="FAR-REF",
            )
            for row in refs:
                series.append(
                    {
                        "figure": f"fig_far_{freq}hz",
                        "series": "FAR-REF continuous",
                        "x": row["B"],
                        "y": row["far_continuous_mean"],
                    }
                )
        ax.axhline(1.0, color="#888888", linewidth=0.7)
        ax.set_xscale("log", base=2)
        ax.set_xticks(list(BS))
        ax.set_xticklabels([str(b) for b in BS], rotation=45, ha="right")
        ax.set_xlabel("Block size B (bytes)")
        ax.set_ylabel("FAR")
        ax.set_title(f"{freq} Hz")
        ymax = max(float(r["far_original_mean"]) for r in summary if int(r["freq_hz"]) == freq)
        ymax = max(ymax, max(float(r["far_continuous_mean"]) for r in summary if int(r["freq_hz"]) == freq))
        ax.set_ylim(0, max(1.15, ymax * 1.18))
        ax.grid(True, axis="y", linewidth=0.4, color="#dddddd")
        handles, labels = ax.get_legend_handles_labels()
        handles.append(
            plt.Line2D(
                [0],
                [0],
                color="#333333",
                marker="o",
                linestyle="none",
                markerfacecolor="white",
                markeredgewidth=1.1,
            )
        )
        labels.append("Original formula")
        fig.legend(
            handles,
            labels,
            frameon=False,
            fontsize=6.5,
            loc="lower center",
            bbox_to_anchor=(0.5, 0.01),
            ncol=2,
        )
        fig.subplots_adjust(left=0.18, right=0.98, top=0.88, bottom=0.40)
        fig.savefig(OUT / f"fig_far_{freq}hz.png", dpi=160)
        plt.close(fig)
    return series


def plot_deadline(rows: list[dict]) -> list[dict]:
    style_axes()
    panels = [
        ("10Hz10Hz", "sonar", "Sonar, concurrent 10 Hz"),
        ("10Hz10Hz", "wheel", "Wheel, concurrent 10 Hz"),
        ("Wheel100Hz", "wheel", "Wheel only, 100 Hz"),
        ("Wheel1000Hz", "wheel", "Wheel only, 1000 Hz"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.8), sharey=True)
    series = []
    for ax, (profile, task, title) in zip(axes.ravel(), panels):
        for mode in MODES:
            xs, fixed, slide = [], [], []
            for block in BS:
                row = next(
                    r
                    for r in rows
                    if r["profile"] == profile
                    and r["task"] == task
                    and r["mode"] == mode
                    and r["B"] == str(block)
                    and r["run"] == "1"
                )
                xs.append(block)
                fixed.append(float(row["fixed_dmr"]))
                slide.append(float(row["sliding_miss_ratio"]))
                series.append({"figure": "fig_fixed_vs_sliding", "series": f"{title} {mode} fixed", "x": str(block), "y": row["fixed_dmr"]})
                series.append(
                    {
                        "figure": "fig_fixed_vs_sliding",
                        "series": f"{title} {mode} sliding",
                        "x": str(block),
                        "y": row["sliding_miss_ratio"],
                    }
                )
            ax.plot(
                xs,
                fixed,
                color=COLORS[mode],
                marker="o",
                linewidth=1.3,
                label=LABEL[mode] if ax is axes[0, 0] else "_nolegend_",
            )
            ax.plot(
                xs,
                slide,
                color=COLORS[mode],
                marker="x",
                linestyle="--",
                linewidth=1.1,
                label="_nolegend_",
            )
        ax.set_xscale("log", base=2)
        ax.set_xticks(list(BS))
        ax.set_xticklabels([str(b) for b in BS], rotation=45, ha="right")
        ax.set_title(title)
        ax.set_ylim(-0.05, 1.08)
        ax.grid(True, axis="y", linewidth=0.4, color="#dddddd")
        ax.set_xlabel("B (bytes)")
    axes[0, 0].set_ylabel("Ratio")
    axes[1, 0].set_ylabel("Ratio")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    handles.extend(
        [
            plt.Line2D([0], [0], color="#333333", marker="o", linewidth=1.3),
            plt.Line2D([0], [0], color="#333333", marker="x", linestyle="--", linewidth=1.1),
        ]
    )
    labels.extend(["Fixed formal DMR", "Sliding-reference ratio"])
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, fontsize=8, bbox_to_anchor=(0.5, 0.995))
    fig.subplots_adjust(left=0.08, right=0.98, top=0.82, bottom=0.08, hspace=0.55, wspace=0.28)
    fig.savefig(OUT / "fig_fixed_vs_sliding.png", dpi=160)
    plt.close(fig)
    return series


def plot_jitter(summary: list[dict]) -> list[dict]:
    style_axes()
    panels = [
        ("10Hz10Hz", "sonar", "Sonar, concurrent 10 Hz"),
        ("10Hz10Hz", "wheel", "Wheel, concurrent 10 Hz"),
        ("Wheel100Hz", "wheel", "Wheel only, 100 Hz"),
        ("Wheel1000Hz", "wheel", "Wheel only, 1000 Hz"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.8), sharey=False)
    series = []
    for ax, (profile, task, title) in zip(axes.ravel(), panels):
        period = None
        for mode in MODES:
            xs, ys, lo, hi = [], [], [], []
            for block in BS:
                row = next(
                    r
                    for r in summary
                    if r["profile"] == profile and r["task"] == task and r["mode"] == mode and r["B"] == str(block)
                )
                period = int(row["period_us"])
                y = float(row["mean_us"])
                xs.append(block)
                ys.append(y)
                lo.append(y - float(row["mean_min_us"]))
                hi.append(float(row["mean_max_us"]) - y)
                series.append({"figure": "fig_jitter_log", "series": f"{title} {mode}", "x": str(block), "y": row["mean_us"]})
            ax.errorbar(
                xs,
                ys,
                yerr=[lo, hi],
                color=COLORS[mode],
                marker="o",
                linewidth=1.2,
                capsize=2,
                label=LABEL[mode],
            )
        if period:
            ax.axhline(period, color="#888888", linewidth=0.7, linestyle="--")
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xticks(list(BS))
        ax.set_xticklabels([str(b) for b in BS], rotation=45, ha="right")
        ax.set_title(title)
        ax.set_xlabel("B (bytes)")
        ax.grid(True, which="both", axis="y", linewidth=0.35, color="#e4e4e4")
    axes[0, 0].set_ylabel("Mean |Δs − P| (µs)")
    axes[1, 0].set_ylabel("Mean |Δs − P| (µs)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 0.995))
    fig.subplots_adjust(left=0.08, right=0.98, top=0.84, bottom=0.08, hspace=0.50, wspace=0.28)
    fig.savefig(OUT / "fig_jitter_log.png", dpi=160)
    plt.close(fig)
    return series


def series_matches(series: list[dict], sources: dict[tuple, str]) -> list[str]:
    issues = []
    for point in series:
        key = (point["figure"], point["series"], point["x"])
        if key not in sources:
            issues.append(f"plotted point has no source {key}")
            continue
        if not near(point["y"], sources[key]):
            issues.append(f"chart {key}: plotted {point['y']} != csv {sources[key]}")
    return issues


def lookup(rows: list[dict], **wanted) -> dict:
    for row in rows:
        if all(str(row[key]) == str(value) for key, value in wanted.items()):
            return row
    raise KeyError(wanted)


def workload(profile: str, task: str) -> str:
    names = {
        ("10Hz10Hz", "sonar"): "Sonar, concurrent 10 Hz",
        ("10Hz10Hz", "wheel"): "Wheel, concurrent 10 Hz",
        ("Wheel100Hz", "wheel"): "Wheel only, 100 Hz",
        ("Wheel1000Hz", "wheel"): "Wheel only, 1000 Hz",
    }
    return names[(profile, task)]


def img_data(name: str) -> str:
    import base64

    payload = base64.b64encode((OUT / name).read_bytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def timeline_svg(example: dict) -> str:
    s_prev = int(example["s_prev_us"])
    points = {
        "s_prev": 0,
        "r_fixed": int(example["r_fixed_us"]) - s_prev,
        "s": int(example["s_us"]) - s_prev,
        "f": int(example["f_us"]) - s_prev,
        "d_fixed": int(example["d_fixed_us"]) - s_prev,
        "d_slide": int(example["d_slide_us"]) - s_prev,
    }
    span = max(points.values()) * 1.08
    width = 680

    def x_of(value: int) -> float:
        return 48 + (value / span) * (width - 70)

    colors = {
        "s_prev": "#333333",
        "r_fixed": "#0f4c81",
        "s": "#333333",
        "f": "#9b1c1c" if example["fixed_miss"] == "1" or example["sliding_miss"] == "1" else "#0d5c3d",
        "d_fixed": "#0f4c81",
        "d_slide": "#6d28d9",
    }
    labels = {
        "s_prev": "s[i-1]",
        "r_fixed": "r fixed",
        "s": "s[i]",
        "f": "finish",
        "d_fixed": "d fixed",
        "d_slide": "d slide",
    }
    parts = [
        f'<svg viewBox="0 0 {width} 160" role="img">',
        f'<line x1="48" y1="64" x2="{width - 16}" y2="64" stroke="#9aa7b2" stroke-width="1"/>',
    ]
    placed: list[tuple[float, int]] = []
    for key in ("s_prev", "r_fixed", "s", "f", "d_fixed", "d_slide"):
        x = x_of(points[key])
        lane = 0
        while any(abs(x - px) < 78 and lane == py for px, py in placed):
            lane += 1
        placed.append((x, lane))
        above = lane % 2 == 0
        level = lane // 2
        y = 64 - 16 - level * 14 if above else 64 + 16 + level * 14
        parts.append(f'<line x1="{x:.1f}" y1="64" x2="{x:.1f}" y2="{y}" stroke="{colors[key]}" stroke-width="1"/>')
        parts.append(f'<circle cx="{x:.1f}" cy="64" r="3.2" fill="{colors[key]}"/>')
        text_y = y - 3 if above else y + 11
        parts.append(
            f'<text x="{x:.1f}" y="{text_y}" text-anchor="middle" font-size="11" fill="{colors[key]}">'
            f'{labels[key]} {points[key]}</text>'
        )
    parts.append("</svg>")
    return "".join(parts)


def write_html(ctx: dict) -> None:
    far = ctx["far_summary"]
    slide = [r for r in ctx["sliding"] if r["run"] == "1" and r["mode"] in MODES]
    examples = ctx["examples"]
    highlight = lookup(ctx["sliding"], profile="Wheel100Hz", task="wheel", mode="original", B="2048", run="1")
    dmr_h = lookup(ctx["dmr"], profile="Wheel100Hz", task="wheel", mode="original", B="2048", run="1")
    b128 = lookup(far, freq_hz="10", mode="original", B="128", farref="0")
    fpe = lookup(far, freq_hz="10", mode="fpe", B="2048", farref="0")
    phase = max(
        (
            r
            for r in ctx["far_rounds"]
            if r["freq_hz"] == "10" and r["mode"] == "original" and r["B"] == "128" and r["farref"] == "0"
        ),
        key=lambda r: float(r["far_continuous"]),
    )

    def far_table(freq: int) -> str:
        body = []
        for block in BS:
            cells = []
            for mode in MODES:
                row = lookup(far, freq_hz=str(freq), mode=mode, B=str(block), farref="0")
                cells.append(f"<td>{row['far_original_mean']}</td><td>{row['far_continuous_mean']}</td>")
            body.append(f"<tr><td class=\"left\">{block}</td>{''.join(cells)}</tr>")
        refs = []
        for block in (1024, 4096):
            row = lookup(far, freq_hz=str(freq), mode="original", B=str(block), farref="1")
            refs.append(
                f"<tr><td class=\"left\">FAR-REF B={block}</td><td>{row['far_original_mean']}</td>"
                f"<td>{row['far_continuous_mean']}</td><td colspan=\"4\"></td></tr>"
            )
        return (
            "<table><caption>Mean of the ten per-round ratios. Open markers on the figure are the original FAR; "
            "filled markers are the recalculated FAR. Neither series is clipped at 1.</caption>"
            "<thead><tr><th class=\"left\">B</th><th>Original old</th><th>Original new</th>"
            "<th>PRNG old</th><th>PRNG new</th><th>FPE old</th><th>FPE new</th></tr></thead><tbody>"
            + "".join(body)
            + "".join(refs)
            + "</tbody></table>"
        )

    deadline_rows = []
    for profile, task in (
        ("10Hz10Hz", "sonar"),
        ("10Hz10Hz", "wheel"),
        ("Wheel100Hz", "wheel"),
        ("Wheel1000Hz", "wheel"),
    ):
        for mode in MODES:
            for block in BS:
                row = lookup(slide, profile=profile, task=task, mode=mode, B=str(block), run="1")
                deadline_rows.append(
                    "<tr>"
                    f"<td class=\"left\">{html.escape(workload(profile, task))}</td>"
                    f"<td class=\"left\">{html.escape(LABEL[mode])}</td>"
                    f"<td>{block}</td><td>{row['fixed_dmr']}</td><td>{row['sliding_miss_ratio']}</td>"
                    f"<td>{row['sliding_eligible']}</td><td>{row['excluded_jobs']}</td>"
                    f"<td>{row['unexecuted']}</td><td>{row['n_distinct_timing']}</td>"
                    "</tr>"
                )
    example_blocks = []
    for example in examples:
        if example["kind"] == "sliding_miss" and example["B"] == "4096":
            continue
        example_blocks.append(
            "<h3>"
            + html.escape(f"B={example['B']}, {example['kind'].replace('_', ' ')}")
            + "</h3>"
            + timeline_svg(example)
            + "<p class=\"src\">Measured Wheel-only 100 Hz, Original SMARM, run 1. Times are microseconds after the previous actual start. "
            f"Fixed miss={example['fixed_miss']}, sliding miss={example['sliding_miss']}. "
            f"Job ticks {example['prev_job_tick']} then {example['job_tick']}. P={example['P_us']} µs.</p>"
        )
    n_distinct_high = sorted(
        {(r["profile"], r["n_distinct_timing"]) for r in slide if r["profile"] != "10Hz10Hz"}
    )
    checks = "".join(f"<li><strong>{html.escape(name)}</strong> — {html.escape(detail)}</li>" for name, detail in ctx["check_lines"])
    soft = "".join(f"<li>{html.escape(line)}</li>" for line in ctx["jitter_notes"])
    status = ctx["status"]
    page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SMARM+ NS256 advisor evidence</title>
<style>
:root {{ --ink:#1c2430; --muted:#5c6b7a; --line:#d5dde6; --bg:#f4f7fb; --card:#fff; --accent:#0f4c81; --warn:#8a4b08; --warnbg:#fff6e8; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; color:var(--ink); background:var(--bg); font:16px/1.6 "Segoe UI", sans-serif; }}
header {{ background:var(--accent); color:#fff; padding:28px 32px 20px; }}
header h1 {{ margin:0 0 8px; font-size:1.5rem; }}
header p {{ margin:0; max-width:85ch; color:#e7f0f8; }}
.badge {{ display:inline-block; margin-top:12px; padding:3px 8px; border-radius:4px; background:#fff; color:var(--accent); font-weight:700; }}
main {{ max-width:1080px; margin:0 auto; padding:22px 18px 64px; }}
section {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:20px 22px; margin:0 0 16px; }}
h2 {{ margin:0 0 10px; color:var(--accent); font-size:1.25rem; }}
h3 {{ margin:16px 0 6px; font-size:1.02rem; }}
p, ul {{ margin:0 0 10px; }}
.th {{ background:#f3f7fb; border-left:4px solid var(--accent); padding:8px 12px; margin:8px 0 12px; }}
.note {{ background:var(--warnbg); border-left:4px solid var(--warn); padding:8px 12px; margin:8px 0 12px; }}
table {{ width:100%; border-collapse:collapse; font-size:.82rem; margin:8px 0 12px; }}
th, td {{ border-bottom:1px solid var(--line); padding:5px 7px; text-align:right; white-space:nowrap; }}
th.left, td.left {{ text-align:left; }}
th {{ background:#eef4fa; }}
caption {{ caption-side:bottom; text-align:left; color:var(--muted); font-size:.78rem; padding-top:6px; white-space:normal; }}
img {{ width:100%; height:auto; }}
.grid {{ display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }}
.scroll {{ overflow:auto; max-height:520px; border:1px solid var(--line); }}
code {{ font-family:Consolas, monospace; font-size:.9em; }}
.src {{ color:var(--muted); font-size:.8rem; }}
@media (max-width:800px) {{ .grid {{ grid-template-columns:1fr; }} }}
@media print {{ body {{ background:#fff; }} section {{ break-inside:avoid; }} }}
</style>
</head>
<body>
<header>
<h1>SMARM+ FreeRTOS NS256 — advisor evidence</h1>
<p>Recalculated FAR, fixed formal deadline-miss ratio, exploratory sliding-reference miss ratio, and inter-start jitter. All values are derived from existing measurements. This page does not select a deadline definition.</p>
<div class="badge">Verification {html.escape(status)}</div>
</header>
<main>
<section id="a">
<h2>A. Experimental setup</h2>
<p>The board is the NUCLEO-L552ZE-Q. The non-secure application region is 256 KiB at <code>0x08040000</code>. Attestation uses one pass and the post-KDF keys. These figures do not extend the result to Zephyr.</p>
<div class="th"><strong>คำอธิบาย.</strong> มีสามชุดการวัด คนละงานกัน ห้ามเอามาปนเป็นงานเดียวกัน</div>
<table>
<thead><tr><th class="left">Dataset</th><th class="left">What runs</th><th class="left">What is counted</th></tr></thead>
<tbody>
<tr><td class="left">FAR at 10, 100, and 1000 Hz</td><td class="left">Synthetic NormalTask during attestation</td><td class="left">Activations inside the attestation call. This is not Wheel and not Sonar.</td></tr>
<tr><td class="left">Concurrent Sonar + Wheel, 10 Hz, 30 s</td><td class="left">Both real tasks, attestation at low priority</td><td class="left">Formal DMR, sliding ratio, and jitter for each task.</td></tr>
<tr><td class="left">Wheel only, 100 Hz, 30 s</td><td class="left">Wheel, no Sonar</td><td class="left">Same three Wheel metrics. Sonar is absent.</td></tr>
<tr><td class="left">Wheel only, 1000 Hz, 3 s</td><td class="left">Wheel, wall shortened because of log size</td><td class="left">Same nominal job count as 100 Hz, not the same wall time.</td></tr>
</tbody>
</table>
<p>Fixed formal DMR keeps the nominal job identity <code>sched_wake_tick</code>. Release and deadline stay on that identity. A job is late when its finish is after <code>d = r + P</code>, unexecuted when the obligation has no row, and right-censored when <code>d &gt; Tend</code>. Right-censored jobs stay out of both the numerator and the denominator.</p>
</section>
<section id="b">
<h2>B. FAR comparison</h2>
<p>The archived logs store <code>Runtime</code> as an integer millisecond. That value is <code>floor(TIM2 difference × 1000 / 137500)</code>. The raw TIM2 counter is not in the log, so the unused part of the last millisecond cannot be recovered. One TIM2 tick is <code>1/137500</code> s, about 7.27 µs. The remaining uncertainty is strictly less than 1 ms.</p>
<p><code>FAR_original = actual / floor(f × T_ms / 1000)</code>. <code>FAR_continuous = actual / (f × T_ms / 1000)</code>. Each reported mean is the mean of ten per-round ratios, which is the campaign aggregate. It is not the ratio of the summed counts. Values above 1 are kept. This is not a nominal-job fulfillment ratio.</p>
<div class="th"><strong>คำอธิบาย.</strong> ที่ 10 Hz ตัวหารเดิมเหลือแค่ 2 หรือ 3 เลยทำให้ FAR เดิมสูงเกินจริง สูตรใหม่ใช้เวลาเป็นวินาทีตรง ๆ แต่เวลายังถูกปัดลงเป็นมิลลิวินาทีอยู่</div>
<div class="note">Original SMARM, B = 128, 10 Hz: {b128['far_original_mean']} → {b128['far_continuous_mean']}. SMARM+FPE, B = 2048, 10 Hz: {fpe['far_original_mean']} → {fpe['far_continuous_mean']}. The second change is a real drop that the integer denominator had hidden, because both the count and the floored expectation were 2.</div>
<p>One 10 Hz Original B = 128 round has runtime {phase['runtime_ms_floor']} ms, {phase['actual_count']} activations, and floored expectation {phase['expected_floor']}. Original FAR is {phase['far_original']}; continuous FAR is {phase['far_continuous']}. A window can contain one more release than <code>f×T</code>, so a single round may sit slightly above 1. FAR-REF leaves interrupts enabled and shows the same kind of excess at 10 Hz, so the excess is not evidence that the task outran its period.</p>
<div class="grid">
<figure><img alt="FAR at 10 Hz" src="{img_data('fig_far_10hz.png')}"><figcaption class="src">10 Hz. Derived from existing rounds.</figcaption></figure>
<figure><img alt="FAR at 100 Hz" src="{img_data('fig_far_100hz.png')}"><figcaption class="src">100 Hz. Derived from existing rounds.</figcaption></figure>
<figure><img alt="FAR at 1000 Hz" src="{img_data('fig_far_1000hz.png')}"><figcaption class="src">1000 Hz. The two formulas match because the floored expectation equals the millisecond count.</figcaption></figure>
</div>
<h3>10 Hz</h3>{far_table(10)}
<h3>100 Hz</h3>{far_table(100)}
<h3>1000 Hz</h3>{far_table(1000)}
</section>
<section id="c">
<h2>C. Deadline comparison</h2>
<p>Fixed formal DMR is the audited result. The other column is the exploratory sliding-reference miss ratio. For a nominally consecutive pair, <code>d_slide = s[i−1] + 2P</code>, and the pair misses when the finish is after that instant. The first lattice job is excluded. A job with no valid immediate predecessor is excluded. An unexecuted obligation is counted separately and is not treated as on time. This is not a standard soft-deadline DMR, and it does not replace the fixed result.</p>
<div class="th"><strong>คำอธิบาย.</strong> ตารางเดิมตรึงเวลากับรหัสงาน Sliding ขยับเส้นตายตามเวลาเริ่มจริงของงานก่อนหน้า งานที่ไม่ได้รันไม่ถูกนับว่าผ่าน และไม่เข้าตัวส่วนของ sliding</div>
<div class="note">Wheel only, 100 Hz, Original SMARM, B = 2048, run 1: fixed DMR = {highlight['fixed_dmr']} ({dmr_h['late']} late, {dmr_h['unexec']} unexecuted, {dmr_h['incomplete']} incomplete, {dmr_h['censored']} right-censored, eligible {dmr_h['eligible']}). Sliding miss ratio = {highlight['sliding_miss_ratio']} on {highlight['sliding_eligible']} eligible pairs. Excluded jobs = {highlight['excluded_jobs']} ({highlight['init_excluded']} initialization, {highlight['gap_excluded']} gap, {highlight['invalid_excluded']} invalid timestamp). Unexecuted obligations = {highlight['unexecuted']}. Those unexecuted obligations stay outside the sliding denominator and are not counted as on time. The executed jobs still finish within two periods of the previous actual start, so the sliding ratio is 0 while the fixed ratio is near 1.</div>
<img alt="Fixed DMR and sliding ratio" src="{img_data('fig_fixed_vs_sliding.png')}">
<p class="src">Solid lines are fixed formal DMR. Dashed lines are the exploratory sliding-reference miss ratio. Run 1 is shown. At 100 Hz and 1000 Hz the three capture files are one timing trace, so the line is that trace. Distinct high-rate counts in this run-1 table: {html.escape(str(n_distinct_high))}.</p>
<h3>Measured timelines</h3>
<p>These three traces are measured. They are not hypothetical. The absolute timestamps are in <code>sliding_worked_examples.csv</code>. The drawing uses microseconds relative to the previous actual start so the deadlines can be compared.</p>
{''.join(example_blocks)}
<div class="th"><strong>ทำไมตัวเลขถึงคนละเรื่อง.</strong> ที่ B = 2048 งานที่รันยังห่างกันประมาณหนึ่งคาบ แต่ทั้งกลุ่มเลื่อนออกจากตารางเดิม และงานท้ายแถวไม่ได้รัน Sliding มองเฉพาะคู่ที่รันติดกัน จึงดูปกติ Fixed DMR นับทั้งงานที่สายและงานที่ไม่ได้รัน</div>
<h3>Run 1, all attested configurations</h3>
<div class="scroll"><table>
<thead><tr><th class="left">Workload</th><th class="left">Algorithm</th><th>B</th><th>Fixed DMR</th><th>Sliding miss ratio</th><th>Sliding eligible</th><th>Excluded</th><th>Unexecuted</th><th>Distinct traces</th></tr></thead>
<tbody>{''.join(deadline_rows)}</tbody>
</table></div>
<p class="src">Ten-hertz configurations have three distinct traces. Their other runs are in <code>sliding_reference_results.csv</code>. The graph uses run 1 so that one panel does not mix three traces into an average.</p>
</section>
<section id="d">
<h2>D. Inter-start jitter</h2>
<p>The plotted statistic is the arithmetic mean of <code>|s[k] − s[k−1] − P|</code>, in microseconds. The first in-window start, and the interval that touches it, is dropped. The standard deviation in the CSV is the population standard deviation, divided by the number of pairs. The 95th percentile is the linear nearest-rank percentile from <code>rt_metrics_wall30.py</code>. Whiskers are the minimum and maximum of distinct-capture means. They are not a confidence interval.</p>
<p>Wheel 100 Hz and Wheel 1000 Hz have three files and one distinct start sequence per configuration. Those files are not averaged as three independent measurements. The UART field <code>inter_start avg</code> is the mean interval itself, not <code>|Δs − P|</code>. The dashed line on each panel is the period.</p>
<div class="th"><strong>คำอธิบาย.</strong> Jitter คือความห่างของเวลาเริ่มจริงเทียบกับคาบ ไม่ใช่ deadline miss ไฟล์ซ้ำที่ความถี่สูงนับเป็นเส้นเวลาเดียว</div>
<img alt="Inter-start jitter on a logarithmic axis" src="{img_data('fig_jitter_log.png')}">
<p class="src">Logarithmic vertical axis. Gate 3 timing files, which are a different capture from the formal DMR recapture. No-attestation means are in the CSV and are omitted here because several are zero and cannot sit on a logarithmic axis.</p>
</section>
<section id="e">
<h2>E. Advisor decision page</h2>
<p>Three options are open. This report does not rank them.</p>
<div class="th"><strong>คำอธิบาย.</strong> หน้านี้ให้เลือก ไม่ได้เลือกให้ ยังไม่แก้ต้นฉบับจนกว่าจะมีการยืนยัน</div>
<h3>Option 1 — Recalculated FAR, jitter, and fixed formal DMR</h3>
<p><strong>Strength.</strong> FAR uses the recorded duration without an integer job-count denominator. Jitter describes start spacing. Fixed DMR remains the audited deadline result, including late, unexecuted, and right-censored jobs.</p>
<p><strong>Limitation.</strong> FAR is still quantized to 1 ms. At 1000 Hz the two FAR formulas are identical for that reason. Jitter comes from the Gate 3 captures, not from the DMR recapture.</p>
<p><strong>Manuscript implication.</strong> A later revision would replace 10 Hz FAR values that used the floored expectation, and would keep fixed DMR as the deadline claim. Sliding would not appear.</p>
<h3>Option 2 — Option 1 plus the sliding-reference ratio as a supplement</h3>
<p><strong>Strength.</strong> The supplement shows that jobs which do run can remain about one period apart after the original schedule has already been missed.</p>
<p><strong>Limitation.</strong> The ratio looks healthy when the unexecuted tail is removed from its denominator. It must not be labeled formal DMR or a standard soft deadline.</p>
<p><strong>Manuscript implication.</strong> Any supplement would have to print eligible, excluded, and unexecuted counts beside the ratio. The fixed DMR table would stay the deadline result.</p>
<h3>Option 3 — Another deadline definition, to be specified</h3>
<p><strong>Strength.</strong> The advisor can name the reference instant and the miss rule before any further claim is written.</p>
<p><strong>Limitation.</strong> The present fixed and sliding numbers would remain records of those two definitions. They would not be renamed after the fact.</p>
<p><strong>Manuscript implication.</strong> No deadline paragraph should be revised until that definition is written down. This directory does not invent one.</p>
</section>
<section id="f">
<h2>F. Evidence and limitations</h2>
<ul>
<li>FAR rounds: <code>freertos_rt/FarSweep_NS256_{{10,100,1000}}Hz_A/analysis/</code>, 69 configurations, 10 rounds each.</li>
<li>Fixed DMR: <code>additional_eval/dmr/final/FINAL_DMR_RESULTS.csv</code>, recomputed in memory-equivalent form to <code>recomputed_formal_dmr.csv</code>.</li>
<li>Sliding inputs: <code>additional_eval/dmr/raw</code> and <code>additional_eval/dmr/recapture/raw</code>. Prior sliding tables in <code>post_advisor_bc_v1</code> were compared and not overwritten.</li>
<li>Jitter inputs: Gate 3 <code>WheelSonar10Hz_NS256_A</code>, <code>Wheel100Hz_NS256_A</code>, and <code>Wheel1000Hz_NS256_A</code>.</li>
<li>FAR logs do not contain the raw TIM2 difference.</li>
<li>High-rate DMR and jitter captures are repeated files, not three independent timing outcomes.</li>
<li>The nine Gate 3 coverage-matrix jitter cells were rounded or verbal. Exact recomputed means are below. A mismatch with a rounded cell is reported and does not change the exact mean.</li>
<li>No Zephyr measurement is included or inferred.</li>
</ul>
<h3>Reproduction checks</h3>
<ul>{checks}</ul>
<h3>Published jitter samples against the exact recomputation</h3>
<ul>{soft}</ul>
</section>
</main>
</body>
</html>
"""
    (OUT / "ADVISOR_EVIDENCE_REPORT.html").write_text(page, encoding="utf-8")


def write_markdown(ctx: dict) -> None:
    highlight = lookup(ctx["sliding"], profile="Wheel100Hz", task="wheel", mode="original", B="2048", run="1")
    b128 = lookup(ctx["far_summary"], freq_hz="10", mode="original", B="128", farref="0")
    fpe = lookup(ctx["far_summary"], freq_hz="10", mode="fpe", B="2048", farref="0")
    lines = [
        "# Advisor decision summary",
        "",
        f"Verification status: **{ctx['status']}**.",
        "",
        "This note does not choose an option and does not revise the manuscript.",
        "",
        "## Results that the decision has to accommodate",
        "",
        f"- Original SMARM, 10 Hz, B = 128: FAR {b128['far_original_mean']} → {b128['far_continuous_mean']}. Mean of ten per-round ratios. Not clipped.",
        f"- SMARM+FPE, 10 Hz, B = 2048: FAR {fpe['far_original_mean']} → {fpe['far_continuous_mean']}. The integer expectation hid this drop.",
        "- At 1000 Hz the original and continuous FAR means are the same, because the archived duration is an integer millisecond and the expected count is that millisecond count.",
        f"- Wheel only, 100 Hz, Original, B = 2048, run 1: fixed DMR {highlight['fixed_dmr']}; sliding miss ratio {highlight['sliding_miss_ratio']}; sliding eligible {highlight['sliding_eligible']}; excluded {highlight['excluded_jobs']}; unexecuted {highlight['unexecuted']}.",
        "- The sliding ratio excludes those unexecuted obligations. It is not formal DMR and not a standard soft deadline.",
        "- Jitter is the mean of `|Δs − P|` in microseconds after the first in-window start is dropped. Identical high-rate files are one trace.",
        "",
        "## Options left open",
        "",
        "1. Recalculated FAR, inter-start jitter, and fixed formal DMR.",
        "2. Option 1, plus the sliding-reference ratio only as a supplement that prints eligible, excluded, and unexecuted counts.",
        "3. A different deadline definition, specified by the advisor before any deadline text is rewritten.",
        "",
        "Option 1 changes the 10 Hz FAR narrative relative to the floored expectation and leaves fixed DMR as the deadline result. Option 2 adds a metric that can read as healthy while fixed DMR is near 1. Option 3 keeps both current calculations as records of their own definitions.",
        "",
        "## Approval still required",
        "",
        "- Which of the three options is the deadline and FAR reporting choice.",
        "- Whether a later manuscript revision may replace the 10 Hz FAR numbers.",
        "- Whether the sliding ratio may be shown outside this evidence directory.",
        "- The wording of any alternative deadline, if option 3 is taken.",
        "",
    ]
    (OUT / "ADVISOR_DECISION_SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    checks = "\n".join(f"- **{name}.** {detail}" for name, detail in ctx["check_lines"])
    soft = "\n".join(f"- {line}" for line in ctx["jitter_notes"])
    reproduction = f"""# Reproduction verification

Status: **{ctx['status']}**.

Command, from the repository root:

```
python thesis/03_rt_experiment/key_separation/ns256/additional_eval/advisor_evidence_v1/build_advisor_evidence.py
```

The script writes only in this directory. `reproduce_post_advisor_bc.py` is executed with its output directory redirected here, so `post_advisor_bc_v1` is not rewritten. `reproduce_final_dmr.py` is executed with `OUT` redirected to `recomputed_formal_dmr.csv`, so `FINAL_DMR_RESULTS.csv` is not rewritten.

## Checks

{checks}

## Gate 3 sample comparison

The coverage matrix gives nine rounded or verbal jitter samples. The exact statistic is the mean of `|Δs − P|` after dropping the first in-window start. A sample marked MISMATCH is a difference from that rounded cell. The exact mean is kept.

{soft}

## Formulas confirmed

- FAR original: `actual / floor(runtime_ms × f / 1000)`, then the mean of ten ratios.
- FAR continuous: `actual / (f × runtime_ms / 1000)`, same mean. No clip at 1.
- Duration source: integer millisecond floor. Raw TIM2 difference is absent. Tick length `1/137500` s is unused precision, and the missing piece is under 1 ms.
- Fixed DMR: nominal id `sched_wake_tick`; late if finish `>` `r+P`; unexecuted if no row; right-censored if `d > Tend`, omitted from numerator and denominator.
- Sliding: `d = s[i-1] + 2P` only for nominally consecutive jobs with valid timestamps. First job excluded. Gap after a missing predecessor excluded. Unexecuted jobs are not successes.
- Jitter: `J = |Δs − P|` from `rt_metrics_wall30.score_regularity`, population SD (`/n`), first in-window start dropped. UART `inter_start avg` matches the mean interval, which is a different number.

## What was not done

No hardware run, no firmware edit, no manuscript edit, no Zephyr run, and no change to the frozen FAR, DMR, or Gate 3 raw files.
"""
    (OUT / "REPRODUCTION_VERIFICATION.md").write_text(reproduction, encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    check_lines: list[tuple[str, str]] = []
    hard: list[str] = []

    far_rounds, far_summary, far_issues = independent_far()
    hard.extend(far_issues)
    check_lines.append(
        (
            "FAR independent recomputation",
            "PASS, 69 configurations × 10 rounds" if not far_issues else "FAIL: " + "; ".join(far_issues[:6]),
        )
    )

    bc = load_module("reproduce_post_advisor_bc", BC_PY)
    bc.HERE = OUT
    bc.main()
    reproduced_summary = read_csv(OUT / "far_comparison.csv")
    far_fields = ["far_original_mean", "far_continuous_mean"]
    far_diff = diff_tables(
        far_summary,
        reproduced_summary,
        ("freq_hz", "mode", "B", "farref"),
        far_fields,
    )
    # The reproduction summary uses the same means; compare those fields only.
    hard.extend(far_diff)
    v1_diff = diff_tables(
        reproduced_summary,
        read_csv(BC_DIR / "far_comparison.csv"),
        ("freq_hz", "mode", "B", "farref"),
        ["far_original_mean", "far_continuous_mean", "n_rounds", "continuous_max"],
    )
    hard.extend(v1_diff)
    check_lines.append(
        (
            "FAR versus post_advisor_bc_v1",
            "PASS, means match the existing directory" if not v1_diff and not far_diff else "FAIL: " + "; ".join((far_diff + v1_diff)[:6]),
        )
    )

    slide_issues = synthetic_sliding(bc)
    hard.extend(slide_issues)
    sliding = read_csv(OUT / "sliding_reference_results.csv")
    examples = read_csv(OUT / "sliding_worked_examples.csv")
    v1_slide = diff_tables(
        sliding,
        read_csv(BC_DIR / "sliding_reference_results.csv"),
        ("profile", "task", "mode", "B", "run"),
        [
            "fixed_dmr",
            "sliding_miss_ratio",
            "sliding_eligible",
            "sliding_miss",
            "excluded_jobs",
            "init_excluded",
            "gap_excluded",
            "invalid_excluded",
            "unexecuted",
            "n_distinct_timing",
        ],
    )
    v1_ex = diff_tables(
        examples,
        read_csv(BC_DIR / "sliding_worked_examples.csv"),
        ("kind", "profile", "B"),
        ["prev_job_tick", "job_tick", "s_prev_us", "s_us", "f_us", "d_fixed_us", "d_slide_us", "fixed_miss", "sliding_miss", "P_us"],
    )
    hard.extend(v1_slide)
    hard.extend(v1_ex)
    blank = [r for r in sliding if r["mode"] in MODES and r["sliding_miss_ratio"] == ""]
    if blank:
        hard.append(f"{len(blank)} attested sliding ratios are blank")
    gaps = sum(int(r["gap_excluded"]) for r in sliding)
    check_lines.append(
        (
            "Sliding rules and existing tables",
            "PASS, synthetic gap handling matched; measured gap_excluded total "
            f"{gaps}; tables match post_advisor_bc_v1"
            if not slide_issues and not v1_slide and not v1_ex and not blank
            else "FAIL: " + "; ".join((slide_issues + v1_slide + v1_ex)[:6]),
        )
    )

    dmr = load_module("reproduce_final_dmr", DMR_PY)
    dmr.OUT = OUT / "recomputed_formal_dmr.csv"
    dmr.main()
    fresh = read_csv(OUT / "recomputed_formal_dmr.csv")
    frozen = read_csv(FINAL_DMR)
    dmr_fields = [
        "eligible",
        "ontime",
        "late",
        "unexec",
        "incomplete",
        "censored",
        "miss",
        "dmr",
        "dup",
        "r_mis",
        "w_step_bad",
        "n_capture_files",
        "n_distinct_timing",
        "table_row",
        "blocked",
    ]
    dmr_diff = diff_tables(fresh, frozen, ("profile", "task", "mode", "B", "run"), dmr_fields)
    hard.extend(dmr_diff)
    check_lines.append(
        (
            "Fixed formal DMR",
            f"PASS, {len(fresh)} rows match FINAL_DMR_RESULTS.csv including late, unexecuted, and right-censored counts"
            if not dmr_diff
            else "FAIL: " + "; ".join(dmr_diff[:6]),
        )
    )

    metrics = load_module("rt_metrics_wall30", METRICS_PY)
    jitter_runs, jitter_summary, jitter_hard, jitter_notes = jitter_tables(metrics)
    hard.extend(jitter_hard)
    write_csv(OUT / "jitter_per_capture.csv", jitter_runs)
    write_csv(OUT / "jitter_summary.csv", jitter_summary)
    identical = sum(1 for r in jitter_summary if r["n_distinct_timing"] == "1" and r["n_files"] == "3")
    check_lines.append(
        (
            "Inter-start jitter",
            "PASS, library matches |Δs−P|; "
            f"{identical} configurations have three identical files and were kept as one trace"
            if not jitter_hard
            else "FAIL: " + "; ".join(jitter_hard[:6]),
        )
    )

    far_series = plot_far(far_summary)
    deadline_series = plot_deadline(sliding)
    jitter_series = plot_jitter(jitter_summary)
    sources = {}
    for point in far_series + deadline_series + jitter_series:
        sources[(point["figure"], point["series"], point["x"])] = point["y"]
    # Rebuild the expected source from the CSVs and compare.
    csv_sources = {}
    for row in far_summary:
        if row["farref"] != "0":
            csv_sources[(f"fig_far_{row['freq_hz']}hz", "FAR-REF continuous", row["B"])] = row["far_continuous_mean"]
            continue
        csv_sources[(f"fig_far_{row['freq_hz']}hz", f"{row['mode']} original formula", row["B"])] = row["far_original_mean"]
        csv_sources[(f"fig_far_{row['freq_hz']}hz", f"{row['mode']} continuous", row["B"])] = row["far_continuous_mean"]
    for row in sliding:
        if row["run"] != "1" or row["mode"] not in MODES:
            continue
        title = workload(row["profile"], row["task"])
        csv_sources[("fig_fixed_vs_sliding", f"{title} {row['mode']} fixed", row["B"])] = row["fixed_dmr"]
        csv_sources[("fig_fixed_vs_sliding", f"{title} {row['mode']} sliding", row["B"])] = row["sliding_miss_ratio"]
    for row in jitter_summary:
        if row["mode"] not in MODES:
            continue
        title = workload(row["profile"], row["task"])
        csv_sources[("fig_jitter_log", f"{title} {row['mode']}", row["B"])] = row["mean_us"]
    chart_diff = []
    if set(sources) != set(csv_sources):
        chart_diff.append("plotted keys differ from CSV keys")
    for key, value in csv_sources.items():
        if key not in sources or not near(sources[key], value):
            chart_diff.append(f"{key}: plot {sources.get(key)} csv {value}")
            if len(chart_diff) > 8:
                break
    hard.extend(chart_diff)
    for name in ("fig_far_10hz.png", "fig_far_100hz.png", "fig_far_1000hz.png", "fig_fixed_vs_sliding.png", "fig_jitter_log.png"):
        if (OUT / name).stat().st_size < 5000:
            hard.append(f"{name} is too small to be a figure")
    check_lines.append(
        (
            "Charts versus CSVs",
            f"PASS, {len(csv_sources)} plotted points match the CSVs written here"
            if not chart_diff
            else "FAIL: " + "; ".join(chart_diff[:6]),
        )
    )

    matrix_bad = [line for line in jitter_notes if line.startswith("MISMATCH") or line.startswith("MISSING")]
    status = "FAIL" if hard else ("PARTIAL" if matrix_bad else "PASS")
    ctx = {
        "status": status,
        "far_rounds": [{**r, "freq_hz": str(r["freq_hz"]), "B": str(r["B"]), "farref": str(r["farref"])} for r in far_rounds],
        "far_summary": far_summary,
        "sliding": sliding,
        "examples": examples,
        "dmr": fresh,
        "check_lines": check_lines,
        "jitter_notes": jitter_notes,
    }
    write_html(ctx)
    write_markdown(ctx)
    page = (OUT / "ADVISOR_EVIDENCE_REPORT.html").read_text(encoding="utf-8")
    external = re.findall(r"""(?:src|href)\s*=\s*["'](?:https?:)?//""", page)
    if external or "<script src=" in page:
        raise SystemExit(f"HTML is not offline-standalone: {external[:3]}")
    required = (b128_text(far_summary, "128", "original"), b128_text(far_summary, "2048", "fpe"))
    for text in required:
        if text not in page:
            raise SystemExit(f"HTML missing {text}")
    if highlight_text(sliding) not in page:
        raise SystemExit("HTML missing the Wheel 100 Hz B=2048 sliding ratio")
    print(f"STATUS {status}")
    print(f"hard={len(hard)} matrix_notes={len(jitter_notes)} matrix_bad={len(matrix_bad)}")
    for line in jitter_notes:
        print(line)
    if hard:
        print("HARD")
        for line in hard[:20]:
            print(line)


def b128_text(summary: list[dict], block: str, mode: str) -> str:
    row = lookup(summary, freq_hz="10", mode=mode, B=block, farref="0")
    return f"{row['far_original_mean']} → {row['far_continuous_mean']}"


def highlight_text(sliding: list[dict]) -> str:
    row = lookup(sliding, profile="Wheel100Hz", task="wheel", mode="original", B="2048", run="1")
    return row["sliding_miss_ratio"]


if __name__ == "__main__":
    main()
