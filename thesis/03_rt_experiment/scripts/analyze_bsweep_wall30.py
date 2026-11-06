#!/usr/bin/env python3
"""Aggregate wall30 Original SMARM B-sweep (+ noattest) with startup exclusion.

Does not flash or open COM ports. Formal DMR remains QUARANTINED.

Outputs under --out-dir:
  per_run_metrics.csv
  aggregate_by_config.csv
  CAMPAIGN_SUMMARY.md
  fig_B_vs_freq.png / fig_B_vs_jitter.png / fig_B_vs_tick_delta.png
  fig_B_vs_tdis.png (if tdis present)
"""
from __future__ import annotations

import argparse
import csv
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from rt_metrics_wall30 import JobStamp, score_regularity, P_US_DEFAULT

N_NOM = 300
P_US = P_US_DEFAULT

# Original campaign (+ optional noattest). PRNG/FPE use --shuffle-mode.
CONFIGS_ORIGINAL = [
    ("noattest", "phase2r_noattest"),
    ("B64", "phase3r_B64"),
    ("B128", "phase3r_B128"),
    ("B256", "phase3r_B256"),
    ("B512", "phase3r_B512"),
    ("B1024", "phase3r_B1024"),
    ("B2048", "phase3r_B2048"),
    ("B4096", "phase3r_B4096"),
]

CONFIGS_SHUFFLE = [
    ("B64", "phase3r_{mode}_B64"),
    ("B128", "phase3r_{mode}_B128"),
    ("B256", "phase3r_{mode}_B256"),
    ("B512", "phase3r_{mode}_B512"),
    ("B1024", "phase3r_{mode}_B1024"),
    ("B2048", "phase3r_{mode}_B2048"),
    ("B4096", "phase3r_{mode}_B4096"),
]

# Back-compat alias
CONFIGS = CONFIGS_ORIGINAL


def parse_uart_meta(text: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for key in (
        "T0",
        "Tend",
        "warmup_us",
        "duration_us",
        "tick0",
        "tick_end",
        "tick_delta",
        "tick_expected_approx",
        "sonar_starts",
        "wheel_starts",
        "sonar_ovf",
        "wheel_ovf",
        "sonar_first_immediate",
        "wheel_first_immediate",
        "wheel_first_short_window",
        "attest_rounds",
        "attest_rounds_in_obs",
        "tdis_avg",
        "tdis_max",
        "incomplete",
    ):
        m = re.search(rf"{key}=(\d+)", text)
        if m:
            out[key] = int(m.group(1))
    m = re.search(r"B=(\d+)", text)
    if m:
        out["B"] = int(m.group(1))
    m = re.search(r"USE_SAU=(\d+)\s+USE_SNAP=(\d+)", text)
    if m:
        out["USE_SAU"] = int(m.group(1))
        out["USE_SNAP"] = int(m.group(2))
    return out


def load_sonar(path: Path) -> Tuple[List[JobStamp], List[Dict[str, str]]]:
    rows = list(csv.DictReader(path.open(encoding="utf-8", errors="replace")))
    jobs = [
        JobStamp(s_exec=int(r["actual_start_us"]), finish=int(r["finish_us"]))
        for r in rows
    ]
    return jobs, rows


def load_wheel(path: Path) -> Tuple[List[JobStamp], List[Dict[str, str]]]:
    rows = list(csv.DictReader(path.open(encoding="utf-8", errors="replace")))
    jobs = []
    for r in rows:
        s = r.get("s_exec_us") or r.get("window_end_us")
        wo = r.get("window_open_us")
        jobs.append(
            JobStamp(
                s_exec=int(s),
                finish=int(r["finish_us"]),
                window_open=int(wo) if wo not in (None, "") else None,
            )
        )
    return jobs, rows


def delay_status_counts(rows: List[Dict[str, str]]) -> Dict[str, int]:
    if not rows or "delay_until_status" not in rows[0]:
        return {}
    return dict(Counter(r["delay_until_status"] for r in rows))


def sensor_error_count(rows: List[Dict[str, str]]) -> int:
    if not rows or "status" not in rows[0]:
        return 0
    return sum(1 for r in rows if int(r["status"]) != 0)


def mean_std(xs: List[float]) -> Tuple[float, float]:
    if not xs:
        return float("nan"), float("nan")
    m = sum(xs) / len(xs)
    if len(xs) == 1:
        return m, 0.0
    v = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return m, math.sqrt(v)


def analyze_run(log_dir: Path, prefix: str, run: int) -> Optional[Dict[str, Any]]:
    tag = f"{prefix}_r{run}"
    uart_p = log_dir / f"{tag}_uart_capture.txt"
    sonar_p = log_dir / f"{tag}_sonar_timing.csv"
    wheel_p = log_dir / f"{tag}_wheel_timing.csv"
    if not (uart_p.is_file() and sonar_p.is_file() and wheel_p.is_file()):
        return None
    uart = uart_p.read_text(encoding="utf-8", errors="replace")
    meta = parse_uart_meta(uart)
    if "T0" not in meta or "Tend" not in meta:
        return None
    t0, tend = meta["T0"], meta["Tend"]
    sonar_jobs, sonar_rows = load_sonar(sonar_p)
    wheel_jobs, wheel_rows = load_wheel(wheel_p)
    s = score_regularity(sonar_jobs, t0, tend, p_us=P_US, n_nominal=N_NOM)
    w = score_regularity(wheel_jobs, t0, tend, p_us=P_US, n_nominal=N_NOM)
    return {
        "tag": tag,
        "run": run,
        "warmup_us": meta.get("warmup_us"),
        "duration_us": meta.get("duration_us"),
        "T0": t0,
        "Tend": tend,
        "obs_delta_us": tend - t0,
        "tick_delta": meta.get("tick_delta"),
        "B": meta.get("B"),
        "USE_SAU": meta.get("USE_SAU"),
        "USE_SNAP": meta.get("USE_SNAP"),
        "attest_rounds": meta.get("attest_rounds"),
        "attest_rounds_in_obs": meta.get("attest_rounds_in_obs"),
        "tdis_avg": meta.get("tdis_avg"),
        "tdis_max": meta.get("tdis_max"),
        "sonar_ovf": meta.get("sonar_ovf", 0),
        "wheel_ovf": meta.get("wheel_ovf", 0),
        "sonar_first_immediate": meta.get("sonar_first_immediate"),
        "wheel_first_immediate": meta.get("wheel_first_immediate"),
        "wheel_first_short_window": meta.get("wheel_first_short_window"),
        "sonar_n": s.n_starts,
        "sonar_hz": s.obs_hz,
        "sonar_freq_ratio": s.freq_ratio,
        "sonar_startup_excl": s.n_startup_excluded,
        "sonar_j_mean": s.primary.mean_us,
        "sonar_j_sd": s.primary.sd_us,
        "sonar_j_p95": s.primary.p95_us,
        "sonar_j_max": s.primary.max_us,
        "sonar_j_sens_mean": s.sensitivity_incl_startup.mean_us,
        "sonar_j_sens_max": s.sensitivity_incl_startup.max_us,
        "sonar_delay_status": delay_status_counts(sonar_rows),
        "sonar_status_err": sensor_error_count(sonar_rows),
        "wheel_n": w.n_starts,
        "wheel_hz": w.obs_hz,
        "wheel_freq_ratio": w.freq_ratio,
        "wheel_startup_excl": w.n_startup_excluded,
        "wheel_j_mean": w.primary.mean_us,
        "wheel_j_sd": w.primary.sd_us,
        "wheel_j_p95": w.primary.p95_us,
        "wheel_j_max": w.primary.max_us,
        "wheel_j_sens_mean": w.sensitivity_incl_startup.mean_us,
        "wheel_j_sens_max": w.sensitivity_incl_startup.max_us,
        "wheel_delay_status": delay_status_counts(wheel_rows),
        "dmr": "QUARANTINED",
    }


def write_csv(path: Path, rows: List[Dict[str, Any]], fieldnames: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            flat = dict(r)
            if isinstance(flat.get("sonar_delay_status"), dict):
                flat["sonar_delay_status"] = str(flat["sonar_delay_status"])
            if isinstance(flat.get("wheel_delay_status"), dict):
                flat["wheel_delay_status"] = str(flat["wheel_delay_status"])
            w.writerow(flat)


def try_plots(agg: List[Dict[str, Any]], out_dir: Path) -> List[str]:
    made: List[str] = []
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        return [f"plots_skipped: {e}"]

    # Numeric B only on x; noattest as separate marker at x=-1
    def x_of(cfg: str) -> float:
        if cfg == "noattest":
            return -1.0
        return float(int(cfg[1:]))

    xs, freqs, fj, fj_p95, ticks, tdis = [], [], [], [], [], []
    labels = []
    for row in agg:
        cfg = row["config"]
        labels.append(cfg)
        xs.append(x_of(cfg))
        freqs.append(row["sonar_hz_mean"])
        fj.append(row["sonar_j_mean_mean"])
        fj_p95.append(row["sonar_j_p95_mean"])
        ticks.append(row["tick_delta_mean"])
        tdis.append(row.get("tdis_avg_mean"))

    def scatter_line(path: Path, ys, ylabel: str, title: str) -> None:
        fig, ax = plt.subplots(figsize=(7, 4))
        for x, y, lab in zip(xs, ys, labels):
            if y is None or (isinstance(y, float) and math.isnan(y)):
                continue
            ax.scatter([x], [y], s=60, label=lab if lab == "noattest" else None)
            ax.annotate(lab, (x, y), textcoords="offset points", xytext=(4, 4), fontsize=8)
        # connect numeric B in order
        pts = [(x, y) for x, y, lab in zip(xs, ys, labels) if lab != "noattest" and y is not None]
        pts.sort()
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-o", color="C0", label="SMARM B-sweep")
        ax.set_xlabel("ATTEST_BLOCK_SIZE B (bytes); noattest at -1")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        plt.close(fig)
        made.append(str(path.name))

    scatter_line(out_dir / "fig_B_vs_freq.png", freqs, "Sonar observed Hz (mean)", "B vs observed frequency")
    scatter_line(out_dir / "fig_B_vs_jitter.png", fj, "Sonar primary jitter mean (us)", "B vs mean regularity jitter")
    # also overlay P95 as second file
    scatter_line(out_dir / "fig_B_vs_jitter_p95.png", fj_p95, "Sonar primary jitter P95 (us)", "B vs P95 regularity jitter")
    scatter_line(out_dir / "fig_B_vs_tick_delta.png", ticks, "FreeRTOS tick_delta (ms)", "B vs tick advancement")
    if any(t is not None and not (isinstance(t, float) and math.isnan(t)) for t in tdis):
        scatter_line(out_dir / "fig_B_vs_tdis.png", tdis, "tdis_avg (device units)", "B vs interrupt-disabled duration")
    return made


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument(
        "--shuffle-mode",
        choices=("original", "prng", "fpe"),
        default="original",
        help="original: phase3r_B*; prng/fpe: phase3r_{mode}_B* (no smoke tags)",
    )
    ap.add_argument(
        "--include-noattest",
        action="store_true",
        help="Include phase2r_noattest (original mode only)",
    )
    args = ap.parse_args()
    log_dir: Path = args.log_dir
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.shuffle_mode == "original":
        configs = list(CONFIGS_ORIGINAL)
        if not args.include_noattest:
            configs = [(c, p) for c, p in configs if c != "noattest"]
            # default original analysis historically included noattest when present;
            # keep prior behavior: include noattest if logs exist
            if (log_dir / "phase2r_noattest_r1_uart_capture.txt").is_file():
                configs = list(CONFIGS_ORIGINAL)
    else:
        configs = [
            (cfg, prefix.format(mode=args.shuffle_mode))
            for cfg, prefix in CONFIGS_SHUFFLE
        ]

    per_run: List[Dict[str, Any]] = []
    for cfg, prefix in configs:
        for run in (1, 2, 3):
            row = analyze_run(log_dir, prefix, run)
            if row is None:
                continue
            row["config"] = cfg
            row["shuffle_mode"] = args.shuffle_mode
            per_run.append(row)

    fields = [
        "config", "tag", "run", "obs_delta_us", "warmup_us", "duration_us",
        "tick_delta", "B", "USE_SAU", "USE_SNAP",
        "attest_rounds", "attest_rounds_in_obs", "tdis_avg", "tdis_max",
        "sonar_n", "sonar_hz", "sonar_freq_ratio", "sonar_startup_excl",
        "sonar_j_mean", "sonar_j_sd", "sonar_j_p95", "sonar_j_max",
        "sonar_j_sens_mean", "sonar_j_sens_max", "sonar_status_err", "sonar_delay_status",
        "wheel_n", "wheel_hz", "wheel_freq_ratio", "wheel_startup_excl",
        "wheel_j_mean", "wheel_j_sd", "wheel_j_p95", "wheel_j_max",
        "wheel_j_sens_mean", "wheel_j_sens_max", "wheel_delay_status",
        "sonar_ovf", "wheel_ovf",
        "sonar_first_immediate", "wheel_first_immediate", "wheel_first_short_window",
        "dmr",
    ]
    write_csv(out_dir / "per_run_metrics.csv", per_run, fields)

    agg: List[Dict[str, Any]] = []
    for cfg, _prefix in configs:
        rows = [r for r in per_run if r["config"] == cfg]
        if not rows:
            continue

        def col(name: str) -> List[float]:
            return [float(r[name]) for r in rows if r.get(name) is not None]

        def ms(name: str) -> Tuple[float, float]:
            return mean_std(col(name))

        sh, shs = ms("sonar_hz")
        wh, whs = ms("wheel_hz")
        sj, sjs = ms("sonar_j_mean")
        sp, sps = ms("sonar_j_p95")
        ss, sss = ms("sonar_j_sens_mean")
        wj, wjs = ms("wheel_j_mean")
        ws, wss = ms("wheel_j_sens_mean")
        td, tds = ms("tick_delta")
        ta, tas = ms("tdis_avg") if any(r.get("tdis_avg") is not None for r in rows) else (float("nan"), float("nan"))
        ar, ars = ms("attest_rounds") if any(r.get("attest_rounds") is not None for r in rows) else (float("nan"), float("nan"))
        agg.append({
            "config": cfg,
            "n_runs": len(rows),
            "sonar_hz_mean": sh, "sonar_hz_sd": shs,
            "wheel_hz_mean": wh, "wheel_hz_sd": whs,
            "sonar_j_mean_mean": sj, "sonar_j_mean_sd": sjs,
            "sonar_j_p95_mean": sp, "sonar_j_p95_sd": sps,
            "sonar_j_sens_mean_mean": ss, "sonar_j_sens_mean_sd": sss,
            "wheel_j_mean_mean": wj, "wheel_j_mean_sd": wjs,
            "wheel_j_sens_mean_mean": ws, "wheel_j_sens_mean_sd": wss,
            "tick_delta_mean": td, "tick_delta_sd": tds,
            "tdis_avg_mean": ta, "tdis_avg_sd": tas,
            "attest_rounds_mean": ar, "attest_rounds_sd": ars,
            "dmr": "QUARANTINED",
        })

    agg_fields = list(agg[0].keys()) if agg else ["config"]
    write_csv(out_dir / "aggregate_by_config.csv", agg, agg_fields)

    plots = try_plots(agg, out_dir)

    lines = [
        "# wall30 B-sweep aggregate",
        "",
        "Primary David jitter: **excludes first in-window sample** (and its interval).",
        "Frequency: **all** in-window starts. Formal DMR: **QUARANTINED**.",
        "",
        f"Per-run rows found: {len(per_run)} / 24 expected.",
        "",
        "| config | n | sonar Hz mean±SD | wheel Hz mean±SD | sonar J mean±SD (us) | tick_delta mean±SD |",
        "|--------|---|------------------|------------------|----------------------|--------------------|",
    ]
    for a in agg:
        lines.append(
            f"| {a['config']} | {a['n_runs']} | "
            f"{a['sonar_hz_mean']:.3f}±{a['sonar_hz_sd']:.3f} | "
            f"{a['wheel_hz_mean']:.3f}±{a['wheel_hz_sd']:.3f} | "
            f"{a['sonar_j_mean_mean']:.1f}±{a['sonar_j_mean_sd']:.1f} | "
            f"{a['tick_delta_mean']:.0f}±{a['tick_delta_sd']:.0f} |"
        )
    lines.append("")
    lines.append("Plots: " + (", ".join(plots) if plots else "none"))
    lines.append("")
    lines.append("## Per-run tags present")
    for r in per_run:
        lines.append(
            f"- `{r['tag']}`: sonar_n={r['sonar_n']} Hz={r['sonar_hz']:.3f} "
            f"Jmean={r['sonar_j_mean']:.1f} (sens={r['sonar_j_sens_mean']:.1f}) "
            f"tick_delta={r['tick_delta']} first_imm={r['sonar_first_immediate']}/{r['wheel_first_immediate']} "
            f"short_win={r['wheel_first_short_window']}"
        )
    (out_dir / "CAMPAIGN_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out_dir} per_run={len(per_run)} configs={len(agg)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
