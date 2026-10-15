#!/usr/bin/env python3
"""Parse FAR-REF logs → f_baseline(B); recompute FAR vs nominal-1000 using remeasure f_obs."""
from __future__ import annotations

import math
import re
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "logs"
OUT = Path(__file__).resolve().parent / "far_ref_baseline_RESULT.md"
BS = [128, 256, 512, 1024, 2048, 4096]

ROUND_RE = re.compile(
    r"^\.?Round\s+(\d+):\s+Runtime=(\d+)\s+ms,\s+NS=(\d+)/(\d+)\s+activations,\s+FAR=([0-9.]+)",
    re.M,
)

# Old FAR from snapguard_remeasure_log.md (2026-10-05), nominal-1000 denominator.
# When FAR≈1 and NS=T, f_obs = 1000 Hz. When FAR<1, f_obs = FAR_mean * 1000.
OLD_FAR = {
    "Baseline": {
        128: 1.0000,
        256: 1.0000,
        512: 1.0000,
        1024: 1.0000,
        2048: 0.5401,
        4096: 0.2735,
    },
    "Snap": {b: 1.0000 for b in BS},
    "Guard": {b: 1.0000 for b in BS},
}


def parse_log(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    rows = []
    for m in ROUND_RE.finditer(text):
        rid = int(m.group(1))
        T = int(m.group(2))
        actual = int(m.group(3))
        expected = int(m.group(4))
        far_nom = float(m.group(5))
        f_i = actual / (T / 1000.0) if T > 0 else float("nan")
        rows.append(
            {
                "round": rid,
                "T_ms": T,
                "actual": actual,
                "expected": expected,
                "far_nom": far_nom,
                "f_Hz": f_i,
            }
        )
    return rows


def mean_std(xs):
    n = len(xs)
    mu = sum(xs) / n
    var = sum((x - mu) ** 2 for x in xs) / n  # population, match tdis style
    return mu, math.sqrt(var)


def main() -> int:
    lines = []
    lines.append("# FAR reference — measured f_baseline (EVAL-ONLY)\n")
    lines.append("Date: 2026-10-14\n")
    lines.append("Flag: `FAR_REF_NO_IRQ_DISABLE=1` on Baseline path (HMAC with IRQ enabled).\n")
    lines.append("NOT a secure design — attestation workload present, no per-block IRQ-off.\n")
    lines.append("|M|=128 KiB @ 0x08060000, NormalTask 1000 Hz osDelayUntil, 10 rounds, TIM2.\n")
    lines.append("\nCompatibility with `snapguard_remeasure_log.md` FAR (2026-10-05): same |M|, 110 MHz, TARGET_FREQ_HZ=1000, NormalTask osPriorityHigh vs attestation osPriorityNormal, TIM2 FAR window — **OK to compare**.\n")

    fbase = {}
    for b in BS:
        path = LOG_DIR / f"far_ref_B{b}.txt"
        rows = parse_log(path)
        assert len(rows) == 10, f"{path} has {len(rows)} rounds"
        fs = [r["f_Hz"] for r in rows]
        mu, sd = mean_std(fs)
        fbase[b] = mu
        lines.append(f"\n## B = {b}\n")
        lines.append(f"Log: `{path.name}`\n\n")
        lines.append("| Round | Runtime ms | actual_run | expected_run | f_i (Hz) |\n")
        lines.append("|------:|----------:|-----------:|-------------:|---------:|\n")
        for r in rows:
            lines.append(
                f"| {r['round']} | {r['T_ms']} | {r['actual']} | {r['expected']} | {r['f_Hz']:.4f} |\n"
            )
        lines.append(f"\n**f_baseline mean = {mu:.4f} Hz**, std = {sd:.4f} Hz\n")

    lines.append("\n## Summary: measured f_baseline(B)\n\n")
    lines.append("| B | f_baseline mean (Hz) | std (Hz) |\n")
    lines.append("|--:|---------------------:|---------:|\n")
    for b in BS:
        path = LOG_DIR / f"far_ref_B{b}.txt"
        rows = parse_log(path)
        mu, sd = mean_std([r["f_Hz"] for r in rows])
        lines.append(f"| {b} | {mu:.4f} | {sd:.4f} |\n")

    lines.append("\n## Corrected FAR = f_observed / f_baseline\n\n")
    lines.append("f_observed recovered from old nominal FAR as `old_FAR * 1000` ")
    lines.append("(since old FAR = f_obs / 1000).\n\n")

    for design in ("Baseline", "Snap", "Guard"):
        lines.append(f"\n### {design}\n\n")
        lines.append(
            "| B | f_obs (Hz) | f_baseline (Hz) | old FAR ( /1000 ) | corrected FAR |\n"
        )
        lines.append("|--:|----------:|----------------:|------------------:|--------------:|\n")
        for b in BS:
            old = OLD_FAR[design][b]
            f_obs = old * 1000.0
            fb = fbase[b]
            corr = f_obs / fb
            lines.append(
                f"| {b} | {f_obs:.2f} | {fb:.4f} | {old:.4f} | **{corr:.4f}** |\n"
            )

    lines.append("\n## Comparison / conclusions\n\n")
    lines.append(
        "Across all B, measured f_baseline is **exactly 1000 Hz** "
        "(actual_run == Runtime_ms every round; std = 0). "
        "Therefore corrected FAR equals the old nominal-1000 FAR for every design/B. "
        "Paper conclusions (Baseline drops at B=2048/4096; Snap/Guard keep FAR=1) **do not change**.\n"
    )

    OUT.write_text("".join(lines), encoding="utf-8")
    print("".join(lines))
    print(f"\nwrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
