# Snap/Guard — Reviewer response tracker

**Source manuscript:** `snapguard_overleaf_10062026.tex` (Overleaf export 2026-10-06)  
**Working revision:** `snapguard_revision.tex` (from Overleaf; **NEW** text in `\ben{...}` blue; **OLD** kept as `%` comments)  
**Data log:** `snapguard_remeasure_log.md`  
**Harness:** branch `snapguard-paper-reviewer`, `|M|=128 KiB @ 0x08060000`, `ATTEST_PASSES=1`

Figures: author will update separately (listed below). Do not treat figure PDF edits as done here.

---

## Major issues

| # | Reviewer ask | Status | Where addressed | Notes |
|---|---|---|---|---|
| M1 | Table I: Guard should not be faster than Baseline (SAU cost) | **DONE (remeasure + wording polish)** | `tab:attest_runtime`, Sec. IV (runtime) | New same-harness table: Guard ≈ Baseline (± few ms). Do **not** claim Guard is faster. Aggregate SAU cost is ~µs×2n ≪ multi-ms. Residual gap attributed to Secure-world scheduling noise / measurement jitter (not “platform variation” alone). |
| M2 | Guard may leak measurement order (fault/timing while hashing) | **DONE (honest S3 claim)** | Sec. III.E (security) | S1/S2/S3 split: S1+S2 hold for both; Guard **does not** claim identical S3 under concurrent NS accesses (fault-timing side channel). Mitigations: data-only window or Snap. |
| M3 | Attested region includes RTOS? How FAR≈1 for Guard? | **DONE (remeasure + coverage sentence)** | Sec. IV.A setup; fault-demo note | Paper window is **128 KiB data** @ `0x08060000` (no live FreeRTOS XIP). Full NS 256 KiB Guard → HardFault (demo). Production: Snap/baseline can cover full NS (incl. XIP); Guard uses dedicated non-XIP window. |
| M4 | Add measured FAR / deadline evidence for all 3 designs | **DONE (remeasure + table)** | Sec. IV.D + `tab:far` | FAR@1000 Hz measured: Baseline drops at B=2048/4096 (0.54/0.27); Snap & Guard = 1.000 all B. |
| M5 | How was t_disabled measured? avg vs worst-case | **DONE (text + table)** | Sec. IV.B + `tab:tdisabled` | DWT CYCCNT @ 110 MHz bracketing each IRQ-off window; report **mean** and note **max** for Guard/Snap/Baseline. |
| M6 | Table II: Guard “RT safe” vs fault text; baseline missing | **DONE (table + caption)** | `tab:ra_comparison` | Baseline TrustZone-M SMARM row added; Guard marked conditional / “data-window” not blanket RT-safe. |

---

## Minor issues / suggestions

| # | Reviewer ask | Status | Notes |
|---|---|---|---|
| m1 | Guard t_disabled should not grow with B | **DONE (clarify)** | Text: grows only modestly (SAU hole programming); still µs-scale. New numbers: ~0.39→0.65 µs mean window. |
| m2 | Unify “1–3 / two to three / roughly three” orders | **DONE** | Unified wording: Snap ~**15× (~1 order)**; Guard ~**three to four orders** at B=4096 (mean window). |
| m3 | B=64 in III.A but not in Sec. IV | **DONE** | Eval now includes B=64. |
| m4 | NS flash cannot be 512 KiB on L552 | **DONE** | Corrected: device 512 KiB total; NS bank 256 KiB; attested \|M\|=128 KiB. |
| m5 | SHA-256 SW or HW? | **DONE** | Stated **software** HMAC-SHA-256 in Secure world (affects Snap’s ~15×). |
| m6 | 0.80–1.30 µs = one SAU region or full guard step? | **DONE** | Clarified: each **lock** or **unlock** IRQ-off window (mean ~0.4–0.65 µs; max ~1.08 µs). Old 0.80–1.30 µs matched **lock+unlock per block** sum. |
| m7 | Fig. 1 “Taks” → “Tasks” | **NOT DONE (figure)** | Author to fix in figure asset. |

---

## Figures (author TODO — not done in this pass)

| Figure | File (expected) | Action |
|---|---|---|
| Fig. system model | `system-threat-model.pdf` | Keep; fix “Taks”→“Tasks” if that typo is here or in timing fig |
| Fig. timing | `timing_diagram.pdf` | Author refresh if needed |
| Fig. workflow | `combined_workflow.pdf` | Author |
| Fig. t_disabled | `tdisabled_three_log.pdf` | Regenerated from new data (`gen_tdisabled_fig.py`) — **author should re-upload to Overleaf** |
| (optional) FAR plot | — | Not required if `tab:far` is enough; author may add later |

---

## Strengths (acknowledge in response letter; no paper change required)

- Clever idea; Guard = SAU, no extra RAM  
- Honest about costs  
- Numbers consistent once Table I remeasured (Snap +7–8 ms on 128 KiB harness, not old +30 ms on 512 KiB)

---

## Suggested author-response letter bullets (Thai/EN draft)

1. **M1:** Remeasured all three modes on identical 128 KiB harness. Guard ≈ Baseline; we do not claim Guard is faster. SAU overhead is microseconds per block.  
2. **M2:** Added Sec. III.E discussion of order leakage via faults/timing; recommend data-only window or Snap for live XIP.  
3. **M3:** Attested region is 128 KiB non-XIP window; Guard on full NS flash faults (debug HardFault).  
4. **M4:** Added measured FAR@1000 Hz table for all three designs.  
5. **M5:** DWT method; mean (and worst-case noted).  
6. **M6:** Revised comparison table.  
7. **Minors:** flash map, B=64, SW SHA-256, SAU timing definition, unified orders; Fig typo pending.

---

## Checklist before resubmit

- [ ] Author updates Overleaf figures (esp. “Taks”, upload new `tdisabled_three_log.pdf`)
- [ ] Compile `snapguard_revision.tex` on Overleaf with `citation.bib`
- [ ] Optional: draft formal response PDF from this tracker
- [ ] Confirm `ATTEST_FAULT_DEMO=0` before any further board runs
