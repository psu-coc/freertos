"""Grouped bar chart of mean t_disabled with std error bars (log scale).

Data from tdis_mean_std_log.md (2026-10-14 remeasure).
Guard means are per SAU lock/unlock window (µs → ms for common axis).

Writes:
  tdisabled_three_log.pdf
  tdisabled_three_log_1.pdf
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUTS = [ROOT / "tdisabled_three_log.pdf", ROOT / "tdisabled_three_log_1.pdf"]

B = np.array([128, 256, 512, 1024, 2048, 4096])
baseline_ms = np.array([0.119, 0.236, 0.469, 0.936, 1.868, 3.732])
baseline_std = np.array([0.0015, 0.0023, 0.0031, 0.0035, 0.0055, 0.0085])
snap_ms = np.array([0.008, 0.015, 0.031, 0.061, 0.122, 0.243])
snap_std = np.array([0.0009, 0.0012, 0.0017, 0.0024, 0.0031, 0.0038])
guard_us = np.array([0.40, 0.41, 0.43, 0.45, 0.52, 0.65])
guard_std_us = np.array([0.07, 0.08, 0.10, 0.15, 0.25, 0.29])
guard_ms = guard_us / 1000.0
guard_std = guard_std_us / 1000.0

plt.rcParams.update(
    {
        "font.size": 10,
        "axes.labelsize": 11,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "font.family": "serif",
        "mathtext.fontset": "dejavuserif",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    }
)

x = np.arange(len(B))
width = 0.26

fig, ax = plt.subplots(figsize=(3.45, 2.55))
ax.bar(
    x - width,
    baseline_ms,
    width,
    yerr=baseline_std,
    label="Baseline SMARM",
    color="#4d4d4d",
    edgecolor="black",
    linewidth=0.5,
    error_kw={"elinewidth": 0.7, "capsize": 1.5, "capthick": 0.7},
)
ax.bar(
    x,
    snap_ms,
    width,
    yerr=snap_std,
    label="SMARM-Snap",
    color="#9dc3e6",
    edgecolor="black",
    linewidth=0.5,
    hatch="//",
    error_kw={"elinewidth": 0.7, "capsize": 1.5, "capthick": 0.7},
)
ax.bar(
    x + width,
    guard_ms,
    width,
    yerr=guard_std,
    label="SMARM-Guard",
    color="#e8a49f",
    edgecolor="black",
    linewidth=0.5,
    hatch="..",
    error_kw={"elinewidth": 0.7, "capsize": 1.5, "capthick": 0.7},
)

ax.set_yscale("log")
ax.set_ylim(2e-4, 20)
ax.set_ylabel(r"$t_{\mathrm{disabled}}$ (ms, log scale)")
ax.set_xlabel(r"Block size $B$ (bytes)")
ax.set_xticks(x)
ax.set_xticklabels([str(b) for b in B])
ax.grid(axis="y", which="both", linestyle=":", linewidth=0.5, alpha=0.65)
ax.set_axisbelow(True)
ax.legend(frameon=False, loc="upper left", ncol=1, handlelength=1.4)

fig.tight_layout(pad=0.35)
for out in OUTS:
    fig.savefig(out, bbox_inches="tight")
    print("wrote", out)
