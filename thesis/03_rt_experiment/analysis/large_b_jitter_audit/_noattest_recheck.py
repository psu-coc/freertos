#!/usr/bin/env python3
"""Independent NoAttest primary-jitter recompute (campaign exclusion rule)."""
from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

LOG = Path(__file__).resolve().parents[2] / "logs"
OUT = Path(__file__).resolve().parents[1] / "noattest_baseline_recheck"
P = 100_000
OUT.mkdir(parents=True, exist_ok=True)


def pct(xs: list[float], p: float) -> float:
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return ys[f] * (c - k) + ys[c] * (k - f)


def load_wheel(path: Path):
    rows = []
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            s = r.get("s_exec_us") or r.get("window_end_us")
            rows.append(
                {
                    "s": int(s),
                    "wake": int(r["sched_wake_tick"]),
                    "dly": int(r["delay_until_status"]),
                    "imm": int(r["immediate_wake"]),
                    "job": int(r["job_id"]),
                }
            )
    return rows


def load_sonar(path: Path):
    rows = []
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            rows.append(
                {
                    "s": int(r["actual_start_us"]),
                    "wake": int(r["sched_wake_tick"]),
                    "dly": int(r["delay_until_status"]),
                    "imm": int(r["immediate_wake"]),
                    "job": int(r["job_id"]),
                }
            )
    return rows


def metrics(rows):
    starts = [r["s"] for r in rows]
    n = len(starts)
    rem = starts[1:]  # drop first in-window stamp (campaign primary)
    iv = [rem[i] - rem[i - 1] for i in range(1, len(rem))]
    J = [abs(x - P) for x in iv]
    all_iv = [starts[i] - starts[i - 1] for i in range(1, n)]
    Js = [abs(x - P) for x in all_iv]
    dly: dict[int, int] = {}
    for r in rows:
        dly[r["dly"]] = dly.get(r["dly"], 0) + 1
    return {
        "n": n,
        "hz": n / 30.0,
        "nJ": len(J),
        "mean": statistics.fmean(J) if J else float("nan"),
        "med": statistics.median(J) if J else float("nan"),
        "p95": pct(J, 95) if J else float("nan"),
        "mx": max(J) if J else float("nan"),
        "mn": min(J) if J else float("nan"),
        "sens_mean": statistics.fmean(Js) if Js else float("nan"),
        "sens_max": max(Js) if Js else float("nan"),
        "dly": dly,
        "n_imm": sum(1 for r in rows if r["imm"]),
        "mono": all(starts[i] < starts[i + 1] for i in range(n - 1)),
        "exact_P": sum(1 for x in iv if x == P),
        "starts": starts,
        "iv": iv,
        "J": J,
        "wake_steps": [rows[i]["wake"] - rows[i - 1]["wake"] for i in range(1, n)],
    }


def main() -> None:
    lines = ["# NoAttest independent recompute\n"]
    for tag in (
        "phase2r_noattest_r1",
        "phase2r_noattest_r2",
        "phase2r_noattest_r3",
    ):
        lines.append(f"\n## {tag}\n")
        for name, loader in (("wheel", load_wheel), ("sonar", load_sonar)):
            a = metrics(loader(LOG / f"{tag}_{name}_timing.csv"))
            lines.append(
                f"- **{name}**: n={a['n']} Hz={a['hz']:.6f} nJ={a['nJ']} "
                f"Jmean={a['mean']:.6f} med={a['med']:.1f} P95={a['p95']:.3f} "
                f"max={a['mx']:.0f} min={a['mn']:.0f} exact_P={a['exact_P']}/{a['nJ']} "
                f"sens_mean={a['sens_mean']:.6f} sens_max={a['sens_max']:.0f} "
                f"dly={a['dly']} imm={a['n_imm']} mono={a['mono']} "
                f"wake_steps_set={sorted(set(a['wake_steps']))}"
            )
        a = metrics(load_wheel(LOG / f"{tag}_wheel_timing.csv"))
        lines.append("\n### Wheel 10 consecutive starts (k=5..14)\n")
        lines.append("| k | s_exec_us | Δs_us | J=|Δ−P| |")
        lines.append("|---|-----------|-------|---------|")
        for i in range(5, 15):
            ds = a["starts"][i] - a["starts"][i - 1]
            lines.append(
                f"| {i} | {a['starts'][i]} | {ds} | {abs(ds - P)} |"
            )
        # Prove first primary pair after exclusion
        lines.append("\n### Startup exclusion proof (Wheel)\n")
        lines.append(
            f"- starts[0]={a['starts'][0]} (excluded from primary pairs)\n"
            f"- starts[1]={a['starts'][1]} starts[2]={a['starts'][2]} "
            f"→ first primary Δ={a['starts'][2]-a['starts'][1]} J={abs(a['starts'][2]-a['starts'][1]-P)}\n"
            f"- all-pairs interval starts[1]-starts[0]={a['starts'][1]-a['starts'][0]} "
            f"(sensitivity only; |J|={abs(a['starts'][1]-a['starts'][0]-P)})\n"
        )

    # Side-by-side with Original from published per_run
    pub = Path(__file__).resolve().parents[1] / "wall30_bsweep_original_recompute" / "per_run_metrics.csv"
    lines.append("\n## Published Original means (for table)\n")
    with pub.open(newline="") as f:
        for r in csv.DictReader(f):
            if r["config"] in ("noattest", "B1024", "B2048", "B4096"):
                lines.append(
                    f"- {r['tag']}: wheel_n={r['wheel_n']} wheel_hz={r['wheel_hz']} "
                    f"wheel_j_mean={r['wheel_j_mean']} tick_delta={r['tick_delta']}"
                )

    text = "\n".join(lines) + "\n"
    (OUT / "INDEPENDENT_RECOMPUTE.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
