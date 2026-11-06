#!/usr/bin/env python3
"""Regenerate Fig. 8 (far_three_1000hz.pdf) — theme matches far_1000.pdf.

Run from thesis/:
    python gen_far_fig8.py
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# --- data (1000 Hz, FreeRTOS) -----------------------------------------------
B = np.array([64, 128, 256, 512, 1024, 2048, 4096], dtype=float)
far_baseline = np.array([1.00, 1.00, 1.00, 1.00, 0.98, 0.55, 0.27])
far_atomic = np.ones_like(B)
far_sau = np.ones_like(B)

# --- theme (same family as far_1000.pdf / Atomiccopy reference) -------------
C_BASE = "#E87722"       # orange — degrading baseline
C_ATOMIC = "#0072B2"     # blue — Atomic Copy
C_SAU = "#009E73"        # teal-green — SAU remapping (distinct, print-safe)

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 10,
    "axes.labelsize": 10,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.linewidth": 0.9,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, ax = plt.subplots(figsize=(4.6, 3.0))

# Baseline: dashed orange squares (like Zephyr line in reference fig)
ax.plot(
    B, far_baseline,
    linestyle="--", linewidth=1.5, color=C_BASE,
    marker="s", markersize=6, markerfacecolor=C_BASE,
    markeredgewidth=0, label="SMARM baseline",
)

# Atomic Copy: solid blue circles (like FreeRTOS line in reference fig)
ax.plot(
    B, far_atomic,
    linestyle="-", linewidth=1.5, color=C_ATOMIC,
    marker="o", markersize=6, markerfacecolor=C_ATOMIC,
    markeredgewidth=0, label="Atomic Copy",
)

# SAU remapping: solid green triangles
ax.plot(
    B, far_sau,
    linestyle="-", linewidth=1.5, color=C_SAU,
    marker="^", markersize=7, markerfacecolor=C_SAU,
    markeredgewidth=0, label="Memory locking",
)

# Linear x: all block sizes as tick positions; label only selected sizes
LABEL_TICKS = {64, 512, 1024, 2048, 4096}
ax.set_xticks(B)
ax.set_xticklabels([str(int(b)) if int(b) in LABEL_TICKS else "" for b in B])
ax.set_xlabel("Block Size (Bytes)")
ax.set_ylabel("Frequency Accuracy Ratio (FAR)")
ax.set_ylim(0.0, 1.2)
ax.set_yticks(np.arange(0.0, 1.21, 0.2))

ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.55, color="#888888")
ax.set_axisbelow(True)
for spine in ax.spines.values():
    spine.set_visible(True)
    spine.set_color("#222222")

ax.legend(loc="lower left", frameon=True, framealpha=0.95, edgecolor="#cccccc")

fig.tight_layout()
out = Path(__file__).resolve().parent / "far_three_1000hz.pdf"
fig.savefig(out, bbox_inches="tight")
plt.close()
print(f"Wrote {out}")
