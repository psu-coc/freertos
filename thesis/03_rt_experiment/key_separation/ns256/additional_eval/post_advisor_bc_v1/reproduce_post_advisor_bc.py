#!/usr/bin/env python3
"""Recalculate FAR and the exploratory sliding-reference metric.

Reads existing FAR round CSVs and formal DMR captures.
Writes only into this directory. Does not flash, and does not edit raw data
or additional_eval/dmr/final/.
"""
from __future__ import annotations

import csv
import re
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
NS256 = HERE.parents[1]
RT = NS256 / "freertos_rt"
DMR = NS256 / "additional_eval" / "dmr"
FINAL = DMR / "final" / "FINAL_DMR_RESULTS.csv"

FREQS = (10, 100, 1000)
MODES = ("original", "prng", "fpe")
BS = (64, 128, 256, 512, 1024, 2048, 4096)
COLORS = {"original": "#0f4c81", "prng": "#b45309", "fpe": "#6d28d9"}

# Logged Runtime=N ms is floor(tim2_diff * 1000 / 137500). One discarded
# millisecond is the remaining uncertainty. TIM2 itself is 1/137500 s.
TIM2_TICKS_PER_SEC = 137500
MS_TRUNCATION_S = 0.001


def far_campaign(freq: int) -> Path:
    return RT / f"FarSweep_NS256_{freq}Hz_A"


def load_rounds(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 10:
        raise SystemExit(f"{path}: expected 10 rounds, got {len(rows)}")
    return rows


def far_tables() -> tuple[list[dict], list[dict]]:
    per_round: list[dict] = []
    summary: list[dict] = []
    for freq in FREQS:
        camp = far_campaign(freq)
        analysis = camp / "analysis"
        cells = [(mode, b, 0) for mode in MODES for b in BS]
        cells += [("original", b, 1) for b in (1024, 4096)]
        for mode, b, farref in cells:
            tag = f"ns256_far_{freq}_{mode}_B{b}" + ("_farref" if farref else "")
            path = analysis / f"{tag}_rounds.csv"
            if not path.exists():
                path = camp / "raw" / tag / f"{tag}_rounds.csv"
            if not path.exists():
                raise SystemExit(f"missing rounds: {tag}")
            rows = load_rounds(path)
            conts = []
            origs = []
            for r in rows:
                runtime_ms = int(r["runtime_ms"])
                actual = int(r["actual"])
                expected = int(r["expected"])
                recomputed_expected = (runtime_ms * freq) // 1000
                if recomputed_expected != expected:
                    raise SystemExit(
                        f"{tag} round {r['round']}: expected {expected} != floor {recomputed_expected}"
                    )
                if runtime_ms <= 0:
                    raise SystemExit(f"{tag}: nonpositive duration")
                elapsed_s = runtime_ms / 1000.0
                far_orig = actual / expected
                far_cont = actual / (freq * elapsed_s)
                stored = float(r["far"])
                fw = ((actual * 10000 + expected // 2) // expected) / 10000.0
                if abs(stored - fw) > 1e-4:
                    raise SystemExit(f"{tag} round {r['round']}: stored FAR {stored} != firmware rounding {fw}")
                conts.append(far_cont)
                origs.append(far_orig)
                per_round.append(
                    {
                        "freq_hz": freq,
                        "mode": mode,
                        "B": b,
                        "farref": farref,
                        "round": int(r["round"]),
                        "runtime_ms_floor": runtime_ms,
                        "elapsed_s_from_floor_ms": f"{elapsed_s:.6f}",
                        "actual_count": actual,
                        "expected_floor": expected,
                        "far_original": f"{far_orig:.6f}",
                        "far_continuous": f"{far_cont:.6f}",
                        "delta_continuous_minus_original": f"{far_cont - far_orig:.6f}",
                        "duration_source": "integer_ms_floor_of_tim2",
                        "tim2_tick_s": f"{1 / TIM2_TICKS_PER_SEC:.9f}",
                        "remaining_truncation_s": f"{MS_TRUNCATION_S:.3f}",
                    }
                )
            summary.append(
                {
                    "freq_hz": freq,
                    "mode": mode,
                    "B": b,
                    "farref": farref,
                    "n_rounds": 10,
                    "duration_ms_mean": f"{statistics.fmean(int(r['runtime_ms']) for r in rows):.4f}",
                    "actual_mean": f"{statistics.fmean(int(r['actual']) for r in rows):.4f}",
                    "expected_floor_mean": f"{statistics.fmean(int(r['expected']) for r in rows):.4f}",
                    "far_original_mean": f"{statistics.fmean(origs):.6f}",
                    "far_continuous_mean": f"{statistics.fmean(conts):.6f}",
                    "delta_mean": f"{statistics.fmean(conts) - statistics.fmean(origs):.6f}",
                    "rounds_original_gt_1": sum(v > 1 for v in origs),
                    "rounds_continuous_gt_1": sum(v > 1 for v in conts),
                    "continuous_min": f"{min(conts):.6f}",
                    "continuous_max": f"{max(conts):.6f}",
                    "clipped": "no",
                    "derived_from": "existing_round_csv_not_a_new_experiment",
                }
            )
    return per_round, summary


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plot_far(summary: list[dict]) -> None:
    for freq in FREQS:
        fig, ax = plt.subplots(figsize=(8.2, 4.6))
        width = 0.24
        xs = list(range(len(BS)))
        for i, mode in enumerate(MODES):
            vals = []
            orig = []
            for b in BS:
                row = next(
                    r
                    for r in summary
                    if int(r["freq_hz"]) == freq
                    and r["mode"] == mode
                    and int(r["B"]) == b
                    and int(r["farref"]) == 0
                )
                vals.append(float(row["far_continuous_mean"]))
                orig.append(float(row["far_original_mean"]))
            offset = (i - 1) * width
            ax.bar(
                [x + offset for x in xs],
                vals,
                width=width,
                color=COLORS[mode],
                label=f"{mode} continuous",
            )
            ax.plot(
                [x + offset for x in xs],
                orig,
                linestyle="none",
                marker="o",
                markersize=4,
                color="black",
                label="original FAR" if i == 0 else None,
            )
        ref = [
            r
            for r in summary
            if int(r["freq_hz"]) == freq and int(r["farref"]) == 1
        ]
        if ref:
            ax.scatter(
                [BS.index(int(r["B"])) for r in ref],
                [float(r["far_continuous_mean"]) for r in ref],
                marker="D",
                s=28,
                color="#111111",
                zorder=4,
                label="FAR-REF continuous",
            )
        ax.axhline(1.0, color="#888888", linewidth=0.8)
        ax.set_xticks(xs)
        ax.set_xticklabels([str(b) for b in BS])
        ax.set_xlabel("Block size B (bytes)")
        ax.set_ylabel("FAR")
        ax.set_title(f"NS256 FAR at {freq} Hz, derived from existing rounds")
        ymax = max(float(r["far_continuous_mean"]) for r in summary if int(r["freq_hz"]) == freq)
        ymax = max(ymax, max(float(r["far_original_mean"]) for r in summary if int(r["freq_hz"]) == freq))
        ax.set_ylim(0, max(1.45, ymax * 1.08))
        ax.legend(frameon=False, fontsize=8, ncol=2)
        fig.tight_layout()
        fig.savefig(HERE / f"far_continuous_{freq}hz.png", dpi=160)
        plt.close(fig)


def uart_text(cell: Path, tag: str) -> str:
    return (cell / f"{tag}_uart_capture.txt").read_text(encoding="utf-8", errors="replace")


def valid_row(row: dict, tend: int) -> bool:
    s = int(row["s_us"])
    f = int(row["f_us"])
    return s > 0 and f > 0 and s < tend


def sliding_one(rows: list[dict], origin: int, tick0: int, tend: int, t0: int, first_w: int) -> dict:
    period = int(rows[0]["period_us"])
    step = period // 1000
    by_w: dict[int, dict] = {}
    dups = 0
    for row in rows:
        w = int(row["sched_wake_tick"])
        if w in by_w:
            dups += 1
        by_w[w] = row
    lattice: list[int] = []
    w = first_w
    for _ in range(20000):
        release = origin + (w - tick0) * 1000
        deadline = release + period
        if deadline > tend:
            break
        if deadline <= t0:
            w += step
            continue
        lattice.append(w)
        w += step
    if not lattice:
        raise SystemExit("empty lattice")
    unexec = invalid = init_ex = gap_ex = eligible = miss = 0
    for i, job in enumerate(lattice):
        row = by_w.get(job)
        if row is None:
            unexec += 1
            continue
        if not valid_row(row, tend):
            invalid += 1
            continue
        if i == 0:
            init_ex += 1
            continue
        prev = lattice[i - 1]
        if job - prev != step:
            raise SystemExit("lattice step is not one nominal period")
        prow = by_w.get(prev)
        if prow is None or not valid_row(prow, tend):
            gap_ex += 1
            continue
        eligible += 1
        d_slide = int(prow["s_us"]) + 2 * period
        if int(row["f_us"]) > d_slide:
            miss += 1
    accounted = unexec + invalid + init_ex + gap_ex + eligible
    if accounted != len(lattice):
        raise SystemExit(f"partition {accounted} != lattice {len(lattice)}")
    ratio = (miss / eligible) if eligible else ""
    return {
        "lattice": len(lattice),
        "sliding_eligible": eligible,
        "sliding_miss": miss,
        "sliding_ratio": ratio,
        "init_excluded": init_ex,
        "gap_excluded": gap_ex,
        "invalid_excluded": invalid,
        "unexecuted": unexec,
        "excluded_jobs": init_ex + gap_ex + invalid,
        "dups": dups,
        "period_us": period,
    }


def load_fixed() -> dict[tuple, dict]:
    out = {}
    with FINAL.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (row["profile"], row["task"], row["mode"], row["B"], int(row["run"]))
            out[key] = row
    return out


def parse_tag(tag: str):
    m = re.match(
        r"dmr_(10Hz10Hz|Wheel100Hz|Wheel1000Hz)_(noattest|original|prng|fpe)"
        r"(?:_(B\d+))?_r(\d+)$",
        tag,
    )
    if not m:
        raise SystemExit(f"bad tag {tag}")
    block = m.group(3).lstrip("B") if m.group(3) else "-"
    return m.group(1), m.group(2), block, int(m.group(4))


def sliding_tables(fixed: dict[tuple, dict]) -> tuple[list[dict], list[dict]]:
    results = []
    examples = []
    cells = []
    for p in sorted((DMR / "raw").glob("dmr_10Hz10Hz_*")):
        if p.is_dir() and "_inj" not in p.name:
            cells.append((p, "10hz"))
    for p in sorted((DMR / "recapture" / "raw").glob("dmr_Wheel*")):
        if p.is_dir():
            cells.append((p, "wheel"))
    wanted = {("Wheel100Hz", "original", b) for b in ("1024", "2048", "4096")}
    for cell, kind in cells:
        tag = cell.name
        profile, mode, block, run = parse_tag(tag)
        text = uart_text(cell, tag)
        rows = list(csv.DictReader((cell / f"{tag}_formal_dmr.csv").open(encoding="utf-8")))
        if kind == "10hz":
            obs = re.search(r"T0=(\d+)\s+Tend=(\d+)", text)
            tick = re.search(r"tick0=(\d+)", text)
            if not obs or not tick:
                raise SystemExit(f"missing 10 Hz anchor {tag}")
            t0, tend, tick0 = int(obs.group(1)), int(obs.group(2)), int(tick.group(1))
            tasks = ("sonar", "wheel")
            anchors = {task: (t0, tick0, tend, t0, tick0) for task in tasks}
        else:
            anc = re.search(
                r"DMR_ANCHOR method=systick_edge tick0=(\d+) tick0_tim2_us=(\d+) .* T0_us=(\d+) Tend_us=(\d+)",
                text,
            )
            entry = re.search(r"wheel_tick_entry=(\d+)", text)
            if not anc or not entry:
                raise SystemExit(f"missing wheel anchor {tag}")
            tick0, origin, t0, tend = (int(anc.group(i)) for i in range(1, 5))
            tasks = ("wheel",)
            anchors = {"wheel": (origin, tick0, tend, t0, int(entry.group(1)))}
        for task in tasks:
            task_rows = [r for r in rows if r["task"] == task]
            if not task_rows:
                raise SystemExit(f"no {task} rows {tag}")
            origin, tick0, tend, t0, first_w = anchors[task]
            stats = sliding_one(task_rows, origin, tick0, tend, t0, first_w)
            key = (profile, task, mode, block, run)
            ref = fixed.get(key)
            if ref is None:
                raise SystemExit(f"no formal DMR row for {key}")
            if int(ref["unexec"]) != stats["unexecuted"]:
                raise SystemExit(
                    f"unexecuted mismatch {key}: formal {ref['unexec']} sliding-lattice {stats['unexecuted']}"
                )
            if int(ref["eligible"]) != stats["lattice"]:
                raise SystemExit(
                    f"lattice mismatch {key}: formal eligible {ref['eligible']} lattice {stats['lattice']}"
                )
            if stats["dups"]:
                raise SystemExit(f"duplicate job id {key}")
            ratio = stats["sliding_ratio"]
            results.append(
                {
                    "profile": profile,
                    "task": task,
                    "mode": mode,
                    "B": block,
                    "run": run,
                    "fixed_dmr": ref["dmr"],
                    "fixed_eligible": ref["eligible"],
                    "fixed_unexec": ref["unexec"],
                    "sliding_miss_ratio": "" if ratio == "" else f"{ratio:.6f}",
                    "sliding_eligible": stats["sliding_eligible"],
                    "sliding_miss": stats["sliding_miss"],
                    "excluded_jobs": stats["excluded_jobs"],
                    "init_excluded": stats["init_excluded"],
                    "gap_excluded": stats["gap_excluded"],
                    "invalid_excluded": stats["invalid_excluded"],
                    "unexecuted": stats["unexecuted"],
                    "n_capture_files": ref["n_capture_files"],
                    "n_distinct_timing": ref["n_distinct_timing"],
                    "table_row": ref["table_row"],
                    "metric": "exploratory_sliding_reference_miss_ratio",
                    "not_formal_dmr": "yes",
                }
            )
            if run == 1 and (profile, mode, block) in wanted and task == "wheel":
                examples.extend(example_rows(task_rows, origin, tick0, tend, t0, first_w, profile, block))
    return results, examples


def example_rows(rows, origin, tick0, tend, t0, first_w, profile, block) -> list[dict]:
    period = int(rows[0]["period_us"])
    step = period // 1000
    by_w = {int(r["sched_wake_tick"]): r for r in rows}
    lattice = []
    w = first_w
    for _ in range(20000):
        release = origin + (w - tick0) * 1000
        if release + period > tend:
            break
        if release + period <= t0:
            w += step
            continue
        lattice.append(w)
        w += step
    out = []
    # Second lattice job, if both it and the first have timestamps.
    if len(lattice) > 1 and lattice[1] in by_w and lattice[0] in by_w:
        cur, prev = by_w[lattice[1]], by_w[lattice[0]]
        if valid_row(cur, tend) and valid_row(prev, tend):
            out.append(describe("second_job", profile, block, prev, cur, period))
    picked_late = picked_slide = False
    for i, job in enumerate(lattice[1:], start=1):
        prev_w = lattice[i - 1]
        if job not in by_w or prev_w not in by_w:
            continue
        cur, prev = by_w[job], by_w[prev_w]
        if not (valid_row(cur, tend) and valid_row(prev, tend)):
            continue
        if job - prev_w != step:
            continue
        fixed_late = int(cur["f_us"]) > int(cur["d_us"])
        slide_miss = int(cur["f_us"]) > int(prev["s_us"]) + 2 * period
        if fixed_late and not slide_miss and not picked_late:
            out.append(describe("fixed_late_sliding_ok", profile, block, prev, cur, period))
            picked_late = True
        if slide_miss and not picked_slide:
            out.append(describe("sliding_miss", profile, block, prev, cur, period))
            picked_slide = True
        if picked_late and picked_slide:
            break
    return out


def describe(kind, profile, block, prev, cur, period) -> dict:
    s_prev = int(prev["s_us"])
    return {
        "kind": kind,
        "profile": profile,
        "B": block,
        "prev_job_tick": int(prev["sched_wake_tick"]),
        "job_tick": int(cur["sched_wake_tick"]),
        "s_prev_us": s_prev,
        "s_us": int(cur["s_us"]),
        "f_us": int(cur["f_us"]),
        "r_fixed_us": int(cur["r_us"]),
        "d_fixed_us": int(cur["d_us"]),
        "r_slide_us": s_prev + period,
        "d_slide_us": s_prev + 2 * period,
        "fixed_miss": int(int(cur["f_us"]) > int(cur["d_us"])),
        "sliding_miss": int(int(cur["f_us"]) > s_prev + 2 * period),
        "P_us": period,
    }


def plot_sliding(rows: list[dict]) -> None:
    panels = [
        ("10Hz10Hz", "sonar", "Sonar 10 Hz"),
        ("10Hz10Hz", "wheel", "Wheel 10 Hz"),
        ("Wheel100Hz", "wheel", "Wheel 100 Hz"),
        ("Wheel1000Hz", "wheel", "Wheel 1000 Hz"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.2), sharey=True)
    for ax, (profile, task, title) in zip(axes.ravel(), panels):
        for mode in MODES:
            xs, fixed, slide = [], [], []
            for b in BS:
                row = next(
                    r
                    for r in rows
                    if r["profile"] == profile
                    and r["task"] == task
                    and r["mode"] == mode
                    and r["B"] == str(b)
                    and int(r["run"]) == 1
                )
                xs.append(b)
                fixed.append(float(row["fixed_dmr"]))
                slide.append(float(row["sliding_miss_ratio"]) if row["sliding_miss_ratio"] else float("nan"))
            ax.plot(xs, fixed, color=COLORS[mode], marker="o", linewidth=1.4, label=f"{mode} fixed")
            ax.plot(xs, slide, color=COLORS[mode], marker="x", linestyle="--", linewidth=1.1, label=f"{mode} sliding")
        ax.set_title(title)
        ax.set_xlabel("B (bytes)")
        ax.set_ylim(-0.05, 1.05)
        ax.grid(True, axis="y", linewidth=0.4, color="#dddddd")
    axes[0, 0].set_ylabel("Ratio")
    axes[1, 0].set_ylabel("Ratio")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=6, frameon=False, fontsize=8)
    fig.suptitle("Fixed DMR (solid) and exploratory sliding-reference ratio (dashed), run 1", y=1.02)
    fig.tight_layout()
    fig.savefig(HERE / "sliding_vs_fixed_run1.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    per_round, summary = far_tables()
    write_csv(HERE / "far_rounds_continuous.csv", per_round)
    write_csv(HERE / "far_comparison.csv", summary)
    plot_far(summary)
    fixed = load_fixed()
    sliding, examples = sliding_tables(fixed)
    write_csv(HERE / "sliding_reference_results.csv", sliding)
    headline = [r for r in sliding if r["table_row"] == "yes" and r["mode"] != "noattest"]
    write_csv(HERE / "sliding_comparison_run1.csv", headline)
    if examples:
        write_csv(HERE / "sliding_worked_examples.csv", examples)
    plot_sliding(sliding)
    n10 = sum(1 for r in summary if int(r["freq_hz"]) == 10 and int(r["farref"]) == 0)
    nslide = len(sliding)
    print(f"far_summary_rows={len(summary)} irq_off_10hz={n10} sliding_rows={nslide} examples={len(examples)}")
    over = [r for r in summary if int(r["farref"]) == 0 and float(r["far_continuous_mean"]) > 1.02]
    print("continuous_mean_above_1.02:", [(r["freq_hz"], r["mode"], r["B"], r["far_continuous_mean"]) for r in over])


if __name__ == "__main__":
    main()
