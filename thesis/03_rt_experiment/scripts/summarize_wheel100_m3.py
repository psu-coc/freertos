#!/usr/bin/env python3
"""Aggregate all phase4r_w100_* attest + noattest runs; write Gate D tables."""
from __future__ import annotations

import csv
import statistics
from collections import defaultdict
from pathlib import Path

from analyze_wheel100_indep import analyze_tag, LOG

OUT = Path(__file__).resolve().parents[1] / "analysis" / "wheel100_m3"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tags = sorted(
        p.name.replace("_uart_capture.txt", "")
        for p in LOG.glob("phase4r_w100_*_uart_capture.txt")
    )
    rows = []
    for tag in tags:
        if not (LOG / f"{tag}_wheel_timing.csv").is_file():
            continue
        a = analyze_tag(tag)
        # parse mode/B/run
        parts = tag.split("_")
        # phase4r_w100_original_B1024_r1 | noattest_r1 | noattest_smoke
        if "noattest" in tag:
            mode, B = "noattest", 0
            run = 0 if tag.endswith("_smoke") else int(parts[-1][1:])
        else:
            mode = parts[2]
            B = int(parts[3][1:])
            run = int(parts[4][1:])
        a.update(mode=mode, B=B, run=run)
        rows.append(a)

    per = OUT / "per_run_metrics.csv"
    fields = [
        "tag", "mode", "B", "run", "n", "hz", "nJ", "j_mean", "j_med", "j_p95", "j_max",
        "signed_mean", "tick_delta", "ovf", "attest_rounds", "tdis_avg", "mono", "fault",
    ]
    with per.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for a in rows:
            w.writerow(a)

    # Aggregate by mode,B (attest only + noattest)
    groups: dict[tuple, list] = defaultdict(list)
    for a in rows:
        if a.get("fault") or a.get("ovf") not in (0, None):
            continue
        if a["mode"] == "noattest":
            groups[("noattest", 0)].append(a)
        else:
            groups[(a["mode"], a["B"])].append(a)

    agg_path = OUT / "aggregate_by_config.csv"
    with agg_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "mode", "B", "n_runs", "hz_mean", "hz_sd", "j_mean_mean", "j_mean_sd",
                "j_p95_mean", "tick_delta_mean", "rounds_mean", "tdis_mean",
            ]
        )
        for (mode, B), xs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
            hz = [x["hz"] for x in xs]
            jm = [x["j_mean"] for x in xs]
            jp = [x["j_p95"] for x in xs]
            td = [x["tick_delta"] for x in xs if x["tick_delta"] is not None]
            rd = [x["attest_rounds"] for x in xs if x["attest_rounds"] is not None]
            td2 = [x["tdis_avg"] for x in xs if x["tdis_avg"] is not None]

            def msd(v):
                if not v:
                    return "", ""
                if len(v) == 1:
                    return f"{v[0]:.6g}", "0"
                return f"{statistics.fmean(v):.6g}", f"{statistics.stdev(v):.6g}"

            hz_m, hz_s = msd(hz)
            j_m, j_s = msd(jm)
            w.writerow(
                [
                    mode,
                    B,
                    len(xs),
                    hz_m,
                    hz_s,
                    j_m,
                    j_s,
                    f"{statistics.fmean(jp):.6g}" if jp else "",
                    f"{statistics.fmean(td):.6g}" if td else "",
                    f"{statistics.fmean(rd):.6g}" if rd else "",
                    f"{statistics.fmean(td2):.6g}" if td2 else "",
                ]
            )

    md = [
        "# Wheel100Hz M3 aggregate",
        "",
        f"Runs analyzed: **{len(rows)}**",
        "",
        f"- Per-run: `{per.name}`",
        f"- Aggregate: `{agg_path.name}`",
        "",
    ]
    (OUT / "SUMMARY.md").write_text("\n".join(md), encoding="utf-8")
    print(f"wrote {per} ({len(rows)} runs)")
    print(f"wrote {agg_path} ({len(groups)} configs)")


if __name__ == "__main__":
    main()
