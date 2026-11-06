#!/usr/bin/env python3
"""Gate C: cross-mode comparison Original vs PRNG vs FPE wall30 B-sweep.

Reads aggregate CSVs; writes COMPARISON.md + overlay figures.
Does not flash. Formal DMR remains QUARANTINED.
tdis: device DWT cycles; time_ms = cycles / 110e3 (110 MHz).
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def load_agg(path: Path) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    with path.open(encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            cfg = row["config"]
            if cfg == "noattest":
                continue
            parsed: Dict[str, float] = {}
            for k, v in row.items():
                if k in ("config", "dmr"):
                    continue
                try:
                    parsed[k] = float(v) if v not in (None, "", "nan") else float("nan")
                except ValueError:
                    continue
            out[cfg] = parsed
    return out


def tdis_ms(cycles: float) -> float:
    if cycles != cycles:  # NaN
        return float("nan")
    return cycles / 110_000.0  # 110 MHz → ms


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--original", type=Path, required=True)
    ap.add_argument("--prng", type=Path, required=True)
    ap.add_argument("--fpe", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)

    o = load_agg(args.original)
    p = load_agg(args.prng)
    f = load_agg(args.fpe)
    bs = sorted({int(c[1:]) for c in set(o) | set(p) | set(f)}, key=int)

    lines = [
        "# Stage 3 cross-mode comparison (wall30 ×3)",
        "",
        "Primary David jitter excludes first in-window sample. Sensitivity includes startup.",
        "tdis_avg is DWT cycles; tdis_ms = cycles/110000 (110 MHz).",
        "Formal DMR = QUARANTINED.",
        "",
        "| B | mode | Hz | Jprim_us | Jsens_us | tick_delta | tdis_cy | tdis_ms | rounds |",
        "|---|------|----|----------|----------|------------|---------|---------|--------|",
    ]
    rows_csv: List[Dict] = []
    for B in bs:
        cfg = f"B{B}"
        for name, tab in (("original", o), ("prng", p), ("fpe", f)):
            if cfg not in tab:
                continue
            r = tab[cfg]
            hz = r.get("sonar_hz_mean", float("nan"))
            jp = r.get("sonar_j_mean_mean", float("nan"))
            # sensitivity may be absent in older aggregates — leave nan
            js = r.get("sonar_j_sens_mean_mean", float("nan"))
            tick = r.get("tick_delta_mean", float("nan"))
            tdis = r.get("tdis_avg_mean", float("nan"))
            rounds = r.get("attest_rounds_mean", float("nan"))
            if rounds != rounds:
                rounds = r.get("attest_rounds_in_obs_mean", float("nan"))
            lines.append(
                f"| {B} | {name} | {hz:.4f} | {jp:.1f} | {js:.1f} | {tick:.0f} | "
                f"{tdis:.0f} | {tdis_ms(tdis):.3f} | {rounds:.0f} |"
            )
            rows_csv.append({
                "B": B, "mode": name, "sonar_hz": hz, "sonar_j_primary": jp,
                "sonar_j_sensitivity": js, "tick_delta": tick,
                "tdis_avg_cy": tdis, "tdis_avg_ms": tdis_ms(tdis),
                "attest_rounds": rounds,
            })

    # Delta vs Original
    lines += ["", "## Δ vs Original (PRNG / FPE − Original)", "",
              "| B | dHz_prng | dHz_fpe | dJprim_prng | dJprim_fpe | dtdis_ms_prng | dtdis_ms_fpe |",
              "|---|----------|---------|-------------|------------|---------------|--------------|"]
    for B in bs:
        cfg = f"B{B}"
        if cfg not in o:
            continue
        def d(a, b, key):
            if cfg not in a or cfg not in b:
                return float("nan")
            return a[cfg].get(key, float("nan")) - b[cfg].get(key, float("nan"))
        lines.append(
            f"| {B} | {d(p,o,'sonar_hz_mean'):+.4f} | {d(f,o,'sonar_hz_mean'):+.4f} | "
            f"{d(p,o,'sonar_j_mean_mean'):+.1f} | {d(f,o,'sonar_j_mean_mean'):+.1f} | "
            f"{tdis_ms(d(p,o,'tdis_avg_mean')):+.3f} | {tdis_ms(d(f,o,'tdis_avg_mean')):+.3f} |"
        )

    (out / "COMPARISON.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with (out / "comparison_table.csv").open("w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=list(rows_csv[0].keys()) if rows_csv else [])
        if rows_csv:
            w.writeheader()
            w.writerows(rows_csv)

    # Overlay plots
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        def series(tab, key):
            xs, ys = [], []
            for B in bs:
                cfg = f"B{B}"
                if cfg in tab and tab[cfg].get(key) == tab[cfg].get(key):
                    xs.append(B)
                    ys.append(tab[cfg][key])
            return xs, ys

        for key, ylab, fname, transform in (
            ("sonar_hz_mean", "Observed Hz", "fig_overlay_freq.png", lambda v: v),
            ("sonar_j_mean_mean", "Primary jitter mean (us)", "fig_overlay_jitter.png", lambda v: v),
            ("tick_delta_mean", "tick_delta (ms)", "fig_overlay_tick.png", lambda v: v),
            ("tdis_avg_mean", "tdis_avg (ms @ 110 MHz)", "fig_overlay_tdis_ms.png", tdis_ms),
        ):
            fig, ax = plt.subplots(figsize=(7, 4))
            for name, tab, style in (("original", o, "-o"), ("prng", p, "-s"), ("fpe", f, "-^")):
                xs, ys = series(tab, key)
                ys = [transform(y) for y in ys]
                if xs:
                    ax.plot(xs, ys, style, label=name)
            ax.set_xscale("log", base=2)
            ax.set_xticks(bs)
            ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, _: f"{int(x)}"))
            ax.set_xlabel("B (bytes)")
            ax.set_ylabel(ylab)
            ax.set_title(ylab + " vs B")
            ax.grid(True, alpha=0.3)
            ax.legend()
            fig.tight_layout()
            fig.savefig(out / fname, dpi=140)
            plt.close(fig)
    except Exception as e:
        (out / "plots_error.txt").write_text(str(e), encoding="utf-8")

    print("wrote", out / "COMPARISON.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
