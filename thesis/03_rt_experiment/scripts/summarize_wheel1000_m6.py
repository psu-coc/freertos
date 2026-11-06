#!/usr/bin/env python3
"""Summarize Wheel1000Hz M6 matrix + NoAttest baselines into CSV + markdown."""
from __future__ import annotations

import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_wheel1000_indep import analyze_tag  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "logs"
OUT = ROOT / "analysis" / "wheel1000_m6"
MODES = ("original", "prng", "fpe")
BS = (64, 128, 256, 512, 1024, 2048, 4096)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for m in MODES:
        for B in BS:
            for r in (1, 2, 3):
                tag = f"phase4r_w1000_{m}_B{B}_r{r}"
                uart = LOG / f"{tag}_uart_capture.txt"
                if not uart.exists():
                    continue
                a = analyze_tag(tag)
                rows.append(a)

    na = []
    for tag in (
        "phase4r_w1000_noattest_smoke",
        "phase4r_w1000_noattest_r1",
        "phase4r_w1000_noattest_r2",
        "phase4r_w1000_noattest_r3",
    ):
        if (LOG / f"{tag}_uart_capture.txt").exists():
            na.append(analyze_tag(tag))

    fields = [
        "tag", "n", "hz", "hz_span", "nJ", "j_mean", "j_med", "j_p95", "j_max",
        "signed_mean", "tick_delta", "ovf", "attest_rounds", "tdis_avg",
        "mono", "fault", "profile_ok",
    ]
    with (OUT / "all_runs.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for a in na + rows:
            w.writerow(a)

    # Aggregate by mode,B
    groups: dict[tuple[str, int], list] = defaultdict(list)
    for a in rows:
        # phase4r_w1000_{mode}_B{B}_r{N}
        parts = a["tag"].split("_")
        mode = parts[2]
        B = int(parts[3][1:])
        groups[(mode, B)].append(a)

    agg_path = OUT / "matrix_summary.csv"
    with agg_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "mode", "B", "n_runs", "hz_mean", "hz_min", "hz_max",
            "j_mean_mean", "j_max_max", "tick_mean", "rounds_mean", "tdis_mean",
            "any_fault", "all_mono",
        ])
        for m in MODES:
            for B in BS:
                g = groups.get((m, B), [])
                if not g:
                    continue
                hz = [x["hz"] for x in g]
                jm = [x["j_mean"] for x in g]
                jx = [x["j_max"] for x in g]
                td = [x["tick_delta"] or 0 for x in g]
                rd = [x["attest_rounds"] or 0 for x in g]
                td2 = [x["tdis_avg"] or 0 for x in g]
                w.writerow([
                    m, B, len(g),
                    f"{statistics.fmean(hz):.2f}", f"{min(hz):.2f}", f"{max(hz):.2f}",
                    f"{statistics.fmean(jm):.2f}", f"{max(jx):.0f}",
                    f"{statistics.fmean(td):.1f}", f"{statistics.fmean(rd):.1f}",
                    f"{statistics.fmean(td2):.0f}",
                    any(x["fault"] for x in g),
                    all(x["mono"] for x in g),
                ])

    md = []
    md.append("# Wheel1000Hz_wall3 M6 matrix summary\n")
    md.append(f"Attest runs analyzed: **{len(rows)}**/63\n")
    md.append(f"NoAttest baselines: **{len(na)}**\n\n")
    if na:
        md.append("## NoAttest\n\n")
        md.append("| tag | n | Hz | J_mean | J_max | tick_delta |\n|-----|---|-----|--------|-------|------------|\n")
        for a in na:
            md.append(
                f"| {a['tag']} | {a['n']} | {a['hz']:.2f} | {a['j_mean']:.3f} | "
                f"{a['j_max']:.0f} | {a['tick_delta']} |\n"
            )
        md.append("\n")
    md.append("## Attest matrix (mean of available reps)\n\n")
    md.append(
        "| mode | B | n | Hz_mean | J_mean | J_max | tick | rounds | tdis |\n"
        "|------|---|---|---------|--------|-------|------|--------|------|\n"
    )
    for m in MODES:
        for B in BS:
            g = groups.get((m, B), [])
            if not g:
                continue
            md.append(
                f"| {m} | {B} | {len(g)} | {statistics.fmean(x['hz'] for x in g):.1f} | "
                f"{statistics.fmean(x['j_mean'] for x in g):.1f} | "
                f"{max(x['j_max'] for x in g):.0f} | "
                f"{statistics.fmean(x['tick_delta'] or 0 for x in g):.0f} | "
                f"{statistics.fmean(x['attest_rounds'] or 0 for x in g):.0f} | "
                f"{statistics.fmean(x['tdis_avg'] or 0 for x in g):.0f} |\n"
            )
    (OUT / "MATRIX_SUMMARY.md").write_text("".join(md), encoding="utf-8")
    print(f"wrote {OUT / 'all_runs.csv'} ({len(na)} NA + {len(rows)} attest)")
    print(f"wrote {agg_path}")
    print(f"wrote {OUT / 'MATRIX_SUMMARY.md'}")


if __name__ == "__main__":
    main()
