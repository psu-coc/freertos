"""Regenerate fig_storage.pdf (Fig. 9): analytical permutation-state storage.

|M| = 512 KiB, n = |M| / B.
  SMARM       : bit-packed sigma, n * ceil(log2 n) bits
  SMARM+PRNG  : bitmap `used`, n bits
  SMARM+FPE   : 128-bit FFX key, 16 bytes (independent of n)
"""
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter

MEM_BYTES = 512 * 1024
BLOCK_SIZES = [64, 128, 256, 512, 1024, 2048, 4096]
FAR_LIMIT_B = 1024


def smarm_bytes(n):
    return n * math.ceil(math.log2(n)) / 8


def prng_bytes(n):
    return n / 8


FPE_BYTES = 16

ns = [MEM_BYTES // b for b in BLOCK_SIZES]
series = [
    ("SMARM", [smarm_bytes(n) for n in ns], "#1f77b4"),
    ("SMARM+PRNG", [prng_bytes(n) for n in ns], "#ff7f0e"),
    ("SMARM+FPE", [FPE_BYTES] * len(ns), "#2ca02c"),
]

plt.rcParams.update({
    "font.size": 10,
    "axes.labelsize": 10,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

fig, ax = plt.subplots(figsize=(3.5, 2.5))
x = np.arange(len(BLOCK_SIZES))
width = 0.27
for k, (label, values, color) in enumerate(series):
    ax.bar(x + (k - 1) * width, values, width, label=label, color=color, zorder=3)

ax.set_yscale("log")
ax.set_ylim(8, 30000)
ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v):,}"))
ax.set_xticks(x)
ax.set_xticklabels([str(b) for b in BLOCK_SIZES])
ax.set_xlabel("Block size $B$ (bytes)")
ax.set_ylabel("Permutation-state\nstorage (bytes, log)")
ax.grid(axis="y", alpha=0.3, zorder=0)

far_x = BLOCK_SIZES.index(FAR_LIMIT_B)
ax.axvline(far_x, color="red", linestyle="--", linewidth=1.2, zorder=4)
ax.text(far_x + 0.12, 12000, "FAR < 1.0\n($B$ > 1024)", color="red",
        fontsize=8, va="center", ha="left")

ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False,
          fontsize=8, handlelength=1.0, columnspacing=0.8, borderaxespad=0.2)
fig.tight_layout(pad=0.3)
fig.savefig("thesis/fig_storage.pdf", bbox_inches="tight", pad_inches=0.02)
fig.savefig("thesis/fig_storage_preview.png", dpi=200, bbox_inches="tight", pad_inches=0.02)

for label, values, _ in series:
    print(label, [round(v) for v in values])
