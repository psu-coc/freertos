import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from pathlib import Path

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.linewidth": 0.8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

out = Path(".")

B = np.array([64, 128, 256, 512, 1024, 2048, 4096])

# -------------------------------------------------------------------------
# FAR figures (single-column)
# -------------------------------------------------------------------------
far_base = np.array([1.00, 1.00, 1.00, 1.00, 0.98, 0.55, 0.27])
far_atomic = np.ones_like(B, dtype=float)
far_sau = np.ones_like(B, dtype=float)
sau_far_all = np.ones((3, len(B)), dtype=float)

fig, ax = plt.subplots(figsize=(4.6, 3.0))
C_BASE, C_ATOMIC, C_SAU = "#E87722", "#0072B2", "#009E73"
ax.plot(B, far_base, linestyle="--", linewidth=1.5, color=C_BASE,
        marker="s", markersize=6, markerfacecolor=C_BASE, markeredgewidth=0,
        label="SMARM baseline")
ax.plot(B, far_atomic, linestyle="-", linewidth=1.5, color=C_ATOMIC,
        marker="o", markersize=6, markerfacecolor=C_ATOMIC, markeredgewidth=0,
        label="Atomic Copy")
ax.plot(B, far_sau, linestyle="-", linewidth=1.5, color=C_SAU,
        marker="^", markersize=7, markerfacecolor=C_SAU, markeredgewidth=0,
        label="Memory locking")
ax.set_xticks(B)
LABEL_TICKS = {64, 512, 1024, 2048, 4096}
ax.set_xticklabels([str(b) if b in LABEL_TICKS else "" for b in B])
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
fig.savefig(out / "far_three_1000hz.pdf", bbox_inches="tight")
plt.close()

# Motivation Fig.: baseline FAR only (FreeRTOS — no second RTOS line)
fig, ax = plt.subplots(figsize=(4.6, 3.0))
ax.plot(
    B, far_base,
    linestyle="--", linewidth=1.5, color=C_BASE,
    marker="s", markersize=6, markerfacecolor=C_BASE,
    markeredgewidth=0, label="SMARM baseline (FreeRTOS)",
)
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
fig.savefig(out / "far_1000.pdf", bbox_inches="tight")
plt.close()

fig, hm = plt.subplots(figsize=(3.4, 1.85))
hm.imshow(sau_far_all, vmin=0.0, vmax=1.0, cmap="Blues", aspect="auto")
hm.set_xticks(np.arange(len(B)))
hm.set_xticklabels([str(b) for b in B], fontsize=7)
hm.set_yticks(np.arange(3))
hm.set_yticklabels(["1000 Hz", "100 Hz", "10 Hz"])
hm.set_xlabel("Block size B (bytes)")
hm.set_ylabel("Workload")
for row in range(3):
    for col in range(len(B)):
        hm.text(col, row, "1.0", ha="center", va="center",
                color="white", fontsize=7, fontweight="bold")
hm.tick_params(length=0)
fig.tight_layout()
fig.savefig(out / "far_sau_heatmap.pdf", bbox_inches="tight")
plt.close()

# -------------------------------------------------------------------------
# SAU design figures (single-column)
# -------------------------------------------------------------------------
fig, memory = plt.subplots(figsize=(3.4, 2.7))
memory.axis("off")
memory.set_xlim(0, 10)
memory.set_ylim(0, 10)

memory.add_patch(mpatches.FancyBboxPatch(
    (0.8, 1.0), 2.4, 8.0, boxstyle="round,pad=0.02,rounding_size=0.05",
    facecolor="#f7f7f7", edgecolor="#333333", linewidth=0.9))

memory.add_patch(plt.Rectangle(
    (0.9, 7.2), 2.2, 1.6, facecolor="#e0e0e0", edgecolor="#333333"))
memory.text(2.0, 8.0, "Secure flash\n(code/NSC)",
            ha="center", va="center", fontsize=7)

memory.add_patch(plt.Rectangle(
    (0.9, 5.8), 2.2, 1.3, facecolor="#cfe2f3", edgecolor="#333333"))
memory.text(2.0, 6.45, "NS code\n64 KiB",
            ha="center", va="center", fontsize=7)

memory.add_patch(plt.Rectangle(
    (0.9, 1.2), 2.2, 4.5, facecolor="#d9ead3", edgecolor="#333333"))
memory.add_patch(plt.Rectangle(
    (0.9, 3.0), 2.2, 1.1, facecolor="#f4cccc",
    edgecolor="#990000", linewidth=1.2))
memory.text(2.0, 4.7, "NS data\n(left)",
            ha="center", va="center", fontsize=7)
memory.text(2.0, 3.55, "Selected block\n$M_{\\sigma(i)}$ (Secure-by-default)",
            ha="center", va="center", fontsize=7, color="#660000")
memory.text(2.0, 1.9, "NS data\n(right)",
            ha="center", va="center", fontsize=7)

memory.annotate("SAU R1\n(static NS)", xy=(3.2, 6.45), xytext=(4.4, 7.5),
                fontsize=7, arrowprops=dict(arrowstyle="->", lw=0.8))
memory.annotate("SAU R6/R7\n(NS left/right)", xy=(3.2, 4.7), xytext=(4.4, 5.2),
                fontsize=7, arrowprops=dict(arrowstyle="->", lw=0.8))
memory.annotate("No enabled NS region\n$\\rightarrow$ Secure by default",
                xy=(3.2, 3.55), xytext=(4.4, 2.6), fontsize=7, color="#660000",
                arrowprops=dict(arrowstyle="->", color="#990000", lw=0.8))
fig.tight_layout()
fig.savefig(out / "sau_memory_map.pdf", bbox_inches="tight")
plt.close()

def _fc_oval(ax, cx, cy, w, h, text, fontsize=8):
    ax.add_patch(mpatches.FancyBboxPatch(
        (cx - w / 2, cy - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.45",
        facecolor="#ffffff", edgecolor="#222222", linewidth=1.0))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize)


def _fc_box(ax, cx, cy, w, h, text, fontsize=7.5, fc="#ffffff", ec="#222222", lw=1.0):
    ax.add_patch(mpatches.FancyBboxPatch(
        (cx - w / 2, cy - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor=fc, edgecolor=ec, linewidth=lw))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize)


def _fc_diamond(ax, cx, cy, w, h, text, fontsize=7.5):
    verts = [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)]
    ax.add_patch(plt.Polygon(verts, closed=True, facecolor="#ffffff",
                             edgecolor="#222222", linewidth=1.0))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize)


def _fc_arrow(ax, x0, y0, x1, y1):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="->", color="#222222", lw=1.0))


def _draw_measurement_flowchart(path, *, title_atomic, atomic_steps, hash_text,
                                atomic_note):
    """Atomic-Copy-style flowchart: Start → σ → loop → protected capture → hash → MAC."""
    fig, ax = plt.subplots(figsize=(5.6, 7.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 14.2)
    ax.axis("off")

    cx = 3.2
    _fc_oval(ax, cx, 13.6, 1.6, 0.55, "Start")
    _fc_arrow(ax, cx, 13.32, cx, 12.85)
    _fc_box(ax, cx, 12.35, 5.4, 0.95,
            "1) Permutation Generation\n"
            r"(Compute $\sigma$ via Fisher–Yates;" "\n"
            r"seeded with $k$ and chal. Init $i=0$)")
    _fc_arrow(ax, cx, 11.87, cx, 11.35)
    _fc_diamond(ax, cx, 10.55, 3.2, 1.45,
                "3) Loop Check\n" + r"(Is $i < n$?)")

    # Yes → right into atomic region
    _fc_arrow(ax, 4.8, 10.55, 6.35, 10.55)
    ax.text(5.45, 10.72, "Yes", fontsize=7, color="#222222")

    # Dashed protected region on the right
    ax.add_patch(mpatches.FancyBboxPatch(
        (5.85, 7.15), 3.85, 4.0,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor="#f3f3f3", edgecolor="#cc0000", linewidth=1.35,
        linestyle=(0, (4, 2.5))))
    ax.text(7.75, 10.85, title_atomic, ha="center", va="center",
            fontsize=7.5, fontweight="bold", color="#990000")
    ys = [10.15, 9.15, 8.15]
    for y, step in zip(ys, atomic_steps):
        _fc_box(ax, 7.75, y, 3.35, 0.72, step, fontsize=6.8)
    for y0, y1 in zip(ys[:-1], ys[1:]):
        _fc_arrow(ax, 7.75, y0 - 0.36, 7.75, y1 + 0.36)
    if atomic_note:
        ax.text(7.75, 7.4, atomic_note, ha="center", va="center",
                fontsize=6.2, color="#555555", style="italic")

    # Down from atomic box → hash
    _fc_arrow(ax, 7.75, 7.15, 7.75, 6.55)
    _fc_box(ax, 7.75, 6.05, 3.5, 0.85, hash_text, fontsize=7)
    _fc_arrow(ax, 7.75, 5.62, 7.75, 5.15)
    _fc_box(ax, 7.75, 4.7, 2.6, 0.7, r"Increment index $i$", fontsize=7.5)

    # Loop back to diamond
    ax.annotate("", xy=(4.8, 10.2), xytext=(6.45, 4.7),
                arrowprops=dict(arrowstyle="->", color="#222222", lw=1.0,
                                connectionstyle="arc3,rad=0.25"))

    # No → finalization
    _fc_arrow(ax, cx, 9.82, cx, 3.55)
    ax.text(cx + 0.25, 6.6, "No", fontsize=7, color="#222222")
    _fc_box(ax, cx, 2.95, 5.0, 0.95,
            "4) Finalization\n"
            r"(Compute MAC using key $k$; send response)")
    _fc_arrow(ax, cx, 2.47, cx, 1.85)
    _fc_oval(ax, cx, 1.45, 1.6, 0.55, "End")

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close()


_draw_measurement_flowchart(
    out / "atomic_workflow.pdf",
    title_atomic="2) Atomic Block Measurement",
    atomic_steps=[
        "Disable system-wide\ninterrupts",
        r"Memcpy block $M_{\sigma(i)}$" "\ninto Secure buffer",
        "Re-enable interrupts",
    ],
    hash_text=r"Measure (hash) Secure" "\n" + r"snapshot of $M_{\sigma(i)}$" "\n(interrupts enabled)",
    atomic_note="IRQ-off window = copy only",
)


def _draw_sau_flowchart(path):
    fig, ax = plt.subplots(figsize=(5.6, 7.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 14.2)
    ax.axis("off")

    cx = 3.2
    _fc_oval(ax, cx, 13.6, 1.6, 0.55, "Start")
    _fc_arrow(ax, cx, 13.32, cx, 12.85)
    _fc_box(ax, cx, 12.35, 5.4, 0.95,
            "1) Permutation Generation\n"
            r"(Compute $\sigma$ via Fisher–Yates;" "\n"
            r"seeded with $k$ and chal. Init $i=0$)")
    _fc_arrow(ax, cx, 11.87, cx, 11.35)
    _fc_diamond(ax, cx, 10.55, 3.2, 1.45,
                "3) Loop Check\n" + r"(Is $i < n$?)")
    _fc_arrow(ax, 4.8, 10.55, 6.35, 10.55)
    ax.text(5.45, 10.72, "Yes", fontsize=7, color="#222222")

    # Protected region: open hole → hash in place → restore
    ax.add_patch(mpatches.FancyBboxPatch(
        (5.85, 5.55), 3.85, 5.55,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor="#f3f3f3", edgecolor="#cc0000", linewidth=1.35,
        linestyle=(0, (4, 2.5))))
    ax.text(7.75, 10.8, "2) SAU-Protected Measurement", ha="center",
            va="center", fontsize=7.5, fontweight="bold", color="#990000")
    steps = [
        (10.05, r"Lock block $M_{\sigma(i)}$" "\n(SAU R6/R7 + DSB/ISB)"),
        (8.75, r"Measure (hash) $M_{\sigma(i)}$ in place" "\n(interrupts remain enabled)"),
        (7.45, "Restore full NS coverage\n+ DSB / ISB"),
    ]
    for y, text in steps:
        _fc_box(ax, 7.75, y, 3.45, 0.95, text, fontsize=6.6)
    _fc_arrow(ax, 7.75, 10.05 - 0.48, 7.75, 8.75 + 0.48)
    _fc_arrow(ax, 7.75, 8.75 - 0.48, 7.75, 7.45 + 0.48)
    ax.text(7.75, 6.05, "Protection = memory locking (not IRQ-off hash)",
            ha="center", va="center", fontsize=6.2, color="#555555",
            style="italic")

    _fc_arrow(ax, 7.75, 5.55, 7.75, 4.85)
    _fc_box(ax, 7.75, 4.4, 2.6, 0.7, r"Increment index $i$", fontsize=7.5)
    ax.annotate("", xy=(4.8, 10.2), xytext=(6.45, 4.4),
                arrowprops=dict(arrowstyle="->", color="#222222", lw=1.0,
                                connectionstyle="arc3,rad=0.25"))

    _fc_arrow(ax, cx, 9.82, cx, 3.55)
    ax.text(cx + 0.25, 6.6, "No", fontsize=7, color="#222222")
    _fc_box(ax, cx, 2.95, 5.0, 0.95,
            "4) Finalization\n"
            r"(Compute MAC using key $k$; send response)")
    _fc_arrow(ax, cx, 2.47, cx, 1.85)
    _fc_oval(ax, cx, 1.45, 1.6, 0.55, "End")

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close()


_draw_sau_flowchart(out / "sau_workflow.pdf")

# -------------------------------------------------------------------------
# Generic TrustZone-M isolation (Background — not board-specific)
# -------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(3.5, 3.2))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.axis("off")

ax.add_patch(mpatches.FancyBboxPatch(
    (0.4, 5.2), 9.2, 4.2,
    boxstyle="round,pad=0.02,rounding_size=0.08",
    facecolor="#f4f8fb", edgecolor="#1a5f7a", linewidth=1.2))
ax.text(5.0, 9.05, "Secure state", ha="center", fontsize=9, fontweight="bold",
        color="#1a5f7a")
ax.text(5.0, 7.6, "Secure flash / RAM\n(trusted code, keys, $k$, $\\sigma$)",
        ha="center", va="center", fontsize=7.5)
ax.add_patch(mpatches.FancyBboxPatch(
    (3.2, 5.55), 3.6, 0.85,
    boxstyle="round,pad=0.02,rounding_size=0.06",
    facecolor="#ffffff", edgecolor="#1a5f7a", linewidth=1.0))
ax.text(5.0, 5.97, "NSC entry gates", ha="center", va="center", fontsize=7.5,
        fontweight="bold")

ax.add_patch(mpatches.FancyBboxPatch(
    (0.4, 0.5), 9.2, 4.2,
    boxstyle="round,pad=0.02,rounding_size=0.08",
    facecolor="#fafafa", edgecolor="#b00020", linewidth=1.2, linestyle="--"))
ax.text(5.0, 4.35, "Non-Secure state", ha="center", fontsize=9,
        fontweight="bold", color="#b00020")
ax.text(5.0, 2.8, "NS application, RTOS,\nattested data (typical layout)",
        ha="center", va="center", fontsize=7.5)

ax.annotate("", xy=(5.0, 5.2), xytext=(5.0, 4.7),
            arrowprops=dict(arrowstyle="-|>", color="#222222", lw=1.2))
ax.text(5.55, 4.95, "NSC call only", fontsize=7, color="#222222")

ax.add_patch(mpatches.FancyBboxPatch(
    (0.4, 0.15), 9.2, 0.95,
    boxstyle="round,pad=0.02,rounding_size=0.06",
    facecolor="#fff8e7", edgecolor="#8a6d1d", linewidth=1.0))
ax.text(5.0, 0.62,
        "SAU + IDAU: enabled region $\\rightarrow$ NS or NSC;\n"
        "uncovered addresses $\\rightarrow$ Secure (default)",
        ha="center", va="center", fontsize=7.2, color="#5c4a12")

fig.tight_layout()
fig.savefig(out / "tz_isolation.pdf", bbox_inches="tight")
plt.close()

# -------------------------------------------------------------------------
# System / threat model (redesigned layout; same semantics as original)
# -------------------------------------------------------------------------
def _box(ax, x, y, w, h, text, fc="#ffffff", ec="#222222", lw=1.0,
         fontsize=8, weight="normal", ls="-"):
    ax.add_patch(mpatches.FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.015,rounding_size=0.08",
        facecolor=fc, edgecolor=ec, linewidth=lw, linestyle=ls))
    ax.text(x + w / 2.0, y + h / 2.0, text, ha="center", va="center",
            fontsize=fontsize, fontweight=weight, color="#111111")


fig, ax = plt.subplots(figsize=(7.2, 3.55))
ax.set_xlim(0, 14.4)
ax.set_ylim(0, 7.1)
ax.axis("off")

# Layer labels on the left
ax.text(0.15, 5.55, "Software", rotation=90, ha="center", va="center",
        fontsize=8, color="#555555")
ax.text(0.15, 2.55, "Isolation", rotation=90, ha="center", va="center",
        fontsize=8, color="#555555")
ax.text(0.15, 0.85, "Hardware", rotation=90, ha="center", va="center",
        fontsize=8, color="#555555")

# --- Software layer: NS stack (left) and Secure stack (right) ---
# NS outer frame (dashed = adversarial)
ax.add_patch(mpatches.FancyBboxPatch(
    (0.55, 3.55), 6.0, 3.25,
    boxstyle="round,pad=0.02,rounding_size=0.1",
    facecolor="#fafafa", edgecolor="#b00020", linewidth=1.4, linestyle="--"))
ax.text(0.75, 6.55, "Non-Secure World  (Untrusted / Compromised)",
        ha="left", va="center", fontsize=8, fontweight="bold", color="#b00020")
ax.text(5.95, 6.55, "Adversary control",
        ha="right", va="center", fontsize=7, color="#b00020",
        bbox=dict(boxstyle="round,pad=0.15", fc="#fdecea", ec="#b00020", lw=0.7))

_box(ax, 0.85, 5.15, 2.55, 0.95, "Real-time\nApplication Tasks",
     fc="#ffffff", ec="#555555")
_box(ax, 3.55, 5.15, 2.55, 0.95, "SMARM\nWrapper Task",
     fc="#ffffff", ec="#555555")
_box(ax, 0.85, 3.85, 5.25, 1.05, "RTOS Kernel (FreeRTOS)",
     fc="#eeeeee", ec="#555555")

# Secure outer frame
ax.add_patch(mpatches.FancyBboxPatch(
    (7.85, 3.55), 6.0, 3.25,
    boxstyle="round,pad=0.02,rounding_size=0.1",
    facecolor="#f4f8fb", edgecolor="#1a5f7a", linewidth=1.4))
ax.text(8.05, 6.55, "Secure World  (Trusted TEE)",
        ha="left", va="center", fontsize=8, fontweight="bold", color="#1a5f7a")
ax.text(13.45, 6.55, "Trusted",
        ha="right", va="center", fontsize=7, color="#1a5f7a",
        bbox=dict(boxstyle="round,pad=0.15", fc="#e8f4f8", ec="#1a5f7a", lw=0.7))

_box(ax, 8.15, 4.85, 2.45, 1.25, "SMARM Service\n(Secure entry)",
     fc="#ffffff", ec="#1a5f7a", lw=1.1)
_box(ax, 10.85, 3.85, 2.7, 2.25, "",
     fc="#e9eef2", ec="#1a5f7a", lw=1.1)
ax.text(12.2, 5.75, "Secure Storage", ha="center", va="center",
        fontsize=8, fontweight="bold", color="#1a5f7a")
_box(ax, 11.05, 4.85, 2.3, 0.55, r"Attestation key $k$",
     fc="#ffffff", ec="#555555", fontsize=7)
_box(ax, 11.05, 4.1, 2.3, 0.55, r"Permutation $\sigma$",
     fc="#ffffff", ec="#555555", fontsize=7)

# NSC call arrow between worlds
ax.annotate(
    "", xy=(7.85, 5.35), xytext=(6.55, 5.55),
    arrowprops=dict(arrowstyle="-|>", color="#222222", lw=1.4,
                    connectionstyle="arc3,rad=0.0"))
ax.text(7.15, 5.95, "NSC call", ha="center", va="bottom",
        fontsize=7.5, fontweight="bold")

# --- Isolation band ---
ax.add_patch(mpatches.FancyBboxPatch(
    (0.55, 2.05), 13.3, 1.2,
    boxstyle="round,pad=0.02,rounding_size=0.08",
    facecolor="#fff8e7", edgecolor="#8a6d1d", linewidth=1.1))
ax.text(7.2, 2.65, "TrustZone-M hardware isolation boundary",
        ha="center", va="center", fontsize=9, fontweight="bold",
        color="#5c4a12")

# --- Hardware layer ---
ax.add_patch(mpatches.FancyBboxPatch(
    (0.55, 0.25), 13.3, 1.55,
    boxstyle="round,pad=0.02,rounding_size=0.08",
    facecolor="#f0f0f0", edgecolor="#333333", linewidth=1.1))
ax.text(0.75, 1.5, "ARM Cortex-M hardware layer",
        ha="left", va="center", fontsize=8, fontweight="bold")

_box(ax, 1.0, 0.45, 3.5, 0.85, "CPU Core", fc="#ffffff", ec="#333333")
_box(ax, 5.0, 0.45, 4.4, 0.85, "TrustZone-M  (SAU / IDAU)",
     fc="#ffffff", ec="#1a5f7a", lw=1.2, weight="bold")
_box(ax, 9.9, 0.45, 3.5, 0.85, "NVIC\n(Interrupt Controller)",
     fc="#ffffff", ec="#333333", fontsize=7.5)

# NS software uses the CPU; isolation is realized by SAU/IDAU
ax.annotate(
    "", xy=(2.75, 1.8), xytext=(2.75, 3.55),
    arrowprops=dict(arrowstyle="-|>", color="#666666", lw=0.9))
ax.annotate(
    "", xy=(7.2, 1.3), xytext=(7.2, 2.05),
    arrowprops=dict(arrowstyle="<->", color="#1a5f7a", lw=1.1))
ax.text(7.45, 1.65, "enforced by", ha="left", va="center",
        fontsize=6.5, color="#1a5f7a")

fig.tight_layout()
fig.savefig(out / "system-threat-model.pdf", bbox_inches="tight")
plt.close()

# -------------------------------------------------------------------------
# SMARM workflow (redesigned; same semantics as original sequence figure)
# -------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 3.7))
ax.set_xlim(0, 14.4)
ax.set_ylim(0, 7.4)
ax.axis("off")

# Three actor columns
cols = [
    (1.1, 3.6, "Verifier", "#f7f7f7", "#444444"),
    (5.1, 3.6, "Non-Secure World\n(SMARM Wrapper Task)", "#fafafa", "#b00020"),
    (9.1, 3.6, "Secure World\n(SMARM Service)", "#f4f8fb", "#1a5f7a"),
]
for x, w, title, fc, ec in cols:
    ax.add_patch(mpatches.FancyBboxPatch(
        (x, 6.35), w, 0.85,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        facecolor=fc, edgecolor=ec, linewidth=1.2))
    ax.text(x + w / 2.0, 6.78, title, ha="center", va="center",
            fontsize=8, fontweight="bold", color=ec)
    # lifeline
    ax.plot([x + w / 2.0, x + w / 2.0], [0.45, 6.35],
            color="#888888", lw=1.0, ls=":")

# World boundary between NS and Secure
ax.plot([8.85, 8.85], [0.35, 7.2], color="#8a6d1d", lw=1.3, ls="-.")
ax.text(8.85, 7.28, "World boundary", ha="center", va="bottom",
        fontsize=7, color="#8a6d1d", fontweight="bold")

cx = [c[0] + c[1] / 2.0 for c in cols]  # centers


def msg(y, x0, x1, label, color="#222222"):
    ax.annotate(
        "", xy=(x1, y), xytext=(x0, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.15))
    mid = (x0 + x1) / 2.0
    ax.text(mid, y + 0.12, label, ha="center", va="bottom",
            fontsize=7.5, color=color)


# (1) and (2)
msg(5.95, cx[0], cx[1], "(1) Send challenge")
msg(5.45, cx[1], cx[2], "(2) Invoke service (NSC call)", color="#1a5f7a")

# Measurement loop box spanning NS + Secure
ax.add_patch(mpatches.FancyBboxPatch(
    (4.85, 1.85), 8.0, 3.2,
    boxstyle="round,pad=0.02,rounding_size=0.08",
    facecolor="#fffdf5", edgecolor="#8a6d1d", linewidth=1.15))
ax.add_patch(mpatches.FancyBboxPatch(
    (4.95, 4.55), 1.35, 0.38,
    boxstyle="round,pad=0.01,rounding_size=0.05",
    facecolor="#8a6d1d", edgecolor="#8a6d1d", linewidth=0.5))
ax.text(5.62, 4.74, "Loop", ha="center", va="center",
        fontsize=7.5, color="white", fontweight="bold")
ax.text(8.85, 4.74,
        "Measurement loop (preemptible between blocks)",
        ha="center", va="center", fontsize=7.5, color="#5c4a12")

# Measure-block activity on Secure lifeline
ax.add_patch(mpatches.FancyBboxPatch(
    (cx[2] - 1.15, 3.55), 2.3, 0.7,
    boxstyle="round,pad=0.02,rounding_size=0.06",
    facecolor="#ffffff", edgecolor="#1a5f7a", linewidth=1.1))
ax.text(cx[2], 3.9, "Measure block", ha="center", va="center",
        fontsize=8, color="#1a5f7a", fontweight="bold")

msg(3.15, cx[2], cx[1], "Yield CPU (interruptible)", color="#666666")
msg(2.45, cx[1], cx[2], "Resume service", color="#666666")

# small loop hint under resume
ax.annotate(
    "", xy=(cx[2] + 0.15, 3.55), xytext=(cx[2] + 0.15, 2.55),
    arrowprops=dict(arrowstyle="-|>", color="#8a6d1d", lw=0.9,
                    connectionstyle="arc3,rad=-0.35"))
ax.text(cx[2] + 0.75, 3.05, "next\nblock", ha="left", va="center",
        fontsize=6.5, color="#8a6d1d")

# (4) and (5)
msg(1.35, cx[2], cx[1], "(4) Return evidence", color="#1a5f7a")
msg(0.75, cx[1], cx[0], "(5) Forward evidence")

fig.tight_layout()
fig.savefig(out / "smarm_workflow.pdf", bbox_inches="tight")
plt.close()

print("generated:",
      "far_three_1000hz.pdf",
      "far_sau_heatmap.pdf",
      "sau_memory_map.pdf",
      "atomic_workflow.pdf",
      "sau_workflow.pdf",
      "tz_isolation.pdf",
      "system-threat-model.pdf",
      "smarm_workflow.pdf")
