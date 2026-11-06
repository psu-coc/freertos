#!/usr/bin/env python3
"""Independent Table I stats from per-round CSVs (no manuscript numbers)."""
from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path

ACCESS_TABLE_I = {
    64: {"original": 644, "prng": 9310, "fpe": 4229},
    128: {"original": 562, "prng": 2730, "fpe": 2325},
    256: {"original": 521, "prng": 1066, "fpe": 1423},
    512: {"original": 500, "prng": 630, "fpe": 945},
    1024: {"original": 490, "prng": 526, "fpe": 726},
    2048: {"original": 484, "prng": 494, "fpe": 611},
    4096: {"original": 481, "prng": 484, "fpe": 553},
}


def stats(vals: list[float]) -> dict:
    return {
        "n": len(vals),
        "mean": statistics.fmean(vals),
        "median": statistics.median(vals),
        "min": min(vals),
        "max": max(vals),
        "pstdev": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--log-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "table_i_runtime" / "logs",
    )
    ap.add_argument("--out-csv", type=Path, default=None)
    args = ap.parse_args()
    rows_out = []
    for csv_path in sorted(args.log_dir.glob("table_i_*_rounds.csv")):
        # table_i_{mode}_B{B}_rounds.csv
        name = csv_path.name
        parts = name.replace("_rounds.csv", "").split("_")
        # table, i, mode, B1024  OR table, i, mode, B, 1024 — tag is table_i_{mode}_B{B}
        tag = name.replace("_rounds.csv", "")
        mode = tag.split("_")[2]
        b = int(tag.split("_B")[1])
        vals = []
        with csv_path.open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if int(row["ns_activations"]) != 0:
                    raise SystemExit(f"{csv_path}: ns_activations != 0")
                vals.append(float(row["runtime_ms"]))
        if len(vals) != 10:
            raise SystemExit(f"{csv_path}: expected 10 rounds, got {len(vals)}")
        st = stats(vals)
        paper = ACCESS_TABLE_I.get(b, {}).get(mode)
        delta = None if paper is None else st["mean"] - paper
        pct = None if paper in (None, 0) else 100.0 * delta / paper
        rows_out.append(
            {
                "tag": tag,
                "mode": mode,
                "B": b,
                **{k: f"{st[k]:.4f}" if isinstance(st[k], float) else st[k] for k in st},
                "access_ms": paper if paper is not None else "",
                "delta_ms": f"{delta:.1f}" if delta is not None else "",
                "delta_pct": f"{pct:.2f}" if pct is not None else "",
            }
        )
        print(
            f"{tag}: mean={st['mean']:.1f} median={st['median']:.1f} "
            f"min={st['min']:.0f} max={st['max']:.0f} pstdev={st['pstdev']:.3f} "
            f"access={paper} delta={delta}"
        )

    # Overhead at B=1024 if all three present
    by = {(r["mode"], int(r["B"])): float(r["mean"]) for r in rows_out}
    if ("original", 1024) in by and ("prng", 1024) in by and ("fpe", 1024) in by:
        o, p, f = by[("original", 1024)], by[("prng", 1024)], by[("fpe", 1024)]
        print(
            f"B1024 overheads: PRNG={(p/o-1)*100:.2f}% FPE={(f/o-1)*100:.2f}% "
            f"(Access claimed 7% / 48%)"
        )

    out = args.out_csv or (args.log_dir.parent / "analysis" / "table_i_summary.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    if rows_out:
        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
            w.writeheader()
            w.writerows(rows_out)
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
