# SMARM+ Manuscript Revision Log (`smarmplus_main.tex`)

## Abstract — REVIEWED / APPROVED (unchanged)

**Date:** 2026-10-24  
**Decision:** **OPTION A — KEEP ABSTRACT UNCHANGED**

| Rule | Status |
|------|--------|
| Further Abstract edits | **None** |
| Restore “retaining comparable real-time behavior” | **No** |
| Existing `\ben{}` elsewhere in manuscript | **Preserve exactly** (Abstract currently has none) |
| Author terminology normalization | **Not done** |
| Introduction edits | **Not started** |

**Approved Abstract state:** as present in `smarmplus_main.tex` after prior author text (systematic study; interference with real-time tasks; security-requirements wording without constant-time or “comparable real-time behavior”).

**Evidence notes carried forward (not Abstract changes):**
- Evaluated PRNG: full-length bitmap scan, no early exit (`PRNG_TIMING_LEAKAGE_SOURCE_REVIEW.md`).
- Constant-time selection: **not** established.

### Flag for later section (do not edit Abstract)

| Flag ID | Topic | When |
|---------|-------|------|
| **FLAG-S3-TIMING** | Body S3 / “Timing behavior of order generation”: reconcile full-scan claim vs data-dependent branches / non-CT prototype; ensure wording does not overclaim constant-time relative to tip firmware | **Security Analysis** (or PRNG design timing paragraph) consistency check under author supervision |

---

## Introduction — THREE APPROVED EDITS APPLIED (2026-10-25)

**Status:** Minimal edits applied per author approval. See `INTRO_THREE_EDITS_CHANGE_RECORD.md`.

| Edit | Result |
|------|--------|
| 1 Delete Mirai/botnet opening `\ben`+`\oak` | Done — no replacement |
| 2 Drop `(S1)--(S3)` from Contrib. 1 only | Done — `\ben{satisfy SMARM's security requirements}` |
| 3 Delete “Both variants build…” sentence | Done — no replacement |

**Next:** Background only after explicit author approval.

## Background — TrustZone-M deletions D1+D2+D3 APPLIED (2026-10-25)

| Edit | Result |
|------|--------|
| D1 Redundant Armv8-M/`\oak` overlap sentence | Deleted |
| D2 Misconfiguration `\ben` paragraph + `\oak` | Deleted |
| D3 Unmarked “These findings motivate…” | Deleted |

SMARM Background subsection, Abstract, Introduction: untouched. Later sections: untouched.

## Section III — Realization — APPROVED MINIMAL REVISION APPLIED (2026-10-31)

**File:** `smarmplus_main.tex`, `\label{sec:porting}` only. `smarm_workflow.pdf` unchanged. No appendix added.

| Edit | Result |
|------|--------|
| Threat-model assumptions (secure boot, Secure-only \(k\), Non-Secure DMA isolation) | One sentence after the NSC sentence. Framed as assumptions. |
| Long threat-model `\ben` marked `delete this.` | Deleted |
| (S1) | `CPSID` sentence kept. Second sentence now limits (S1) to each block measurement. |
| Key hierarchy | Heading removed. Equation kept inside the Secure Service paragraph. Labels `0x00` / `0x01`. |
| Authenticated header | Paragraph deleted. Algorithm 1 caption, notation, and `HMAC.Update(hdr)` removed. Seed is `HMAC-SHA256(k_prp, chal)`. |
| Multiple traversals paragraph | Deleted. No replacement. |
| Between-block continuation paragraph | Deleted. Two sentences added to the existing Security subsection. |
| Platform Security Configuration | Subsection removed. No replacement appendix. |
| Workflow step 3 | Approved interrupt-reenable sentence. Figure untouched. |

**Left for later sections (not edited):** undefined `sec:platform-config` at the Evaluation TF-M sentence and the Related Work TCB sentence; `hdr` still in the PRNG and FPE algorithms; “continuation invariant” still named in those security paragraphs and in the formal proof; `|M| = 512` KiB and \(T > 1\) result claims.
