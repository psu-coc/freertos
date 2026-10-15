# Snap/Guard — Camera-ready update notes (paste / email)

**Paper:** Snap/Guard (ICSEC 2026, #1571346857)  
**File to upload:** `snapguard_overleaf_10062026.tex` (same content as `snapguard_revision.tex`)  
**Blue text:** new/changed prose is wrapped in `\ben{...}` for easy visual diff in the PDF.

## What we revised in this update

1. **[M2] Security claims (S1 / S2 / S3)**  
   Rewrote Sec. Security Guarantees so Snap and Guard are no longer claimed to retain identical **[S3]** under all conditions.  
   - **[S1]** and **[S2]** hold for both.  
   - **[S3]** storage secrecy of $\sigma$ still holds; for **Guard**, concurrent Non-Secure accesses to a locked block can create a **fault/timing side channel** on the remaining order.  
   - Mitigations stated: data-only attested window, or prefer Snap/baseline when live XIP must stay executable.

2. **[R3 / structure] Related Work moved earlier**  
   Related Work now follows the Introduction (before Background), as requested for camera-ready readability.

3. **[M1] Runtime wording**  
   Kept the remeasured Table I (Guard ≉ faster than baseline). Replaced vague “platform variation” with an explicit statement that residual ms-scale gaps are consistent with Secure-world scheduling noise / measurement jitter under FreeRTOS, not a Guard speedup.

4. **[M3] Coverage guidance**  
   Setup now states clearly: same 128 KiB non-XIP harness for fair comparison; in production Snap/baseline can cover full Non-Secure (incl. XIP), while Guard should use a dedicated non-XIP data window.

5. **Table readability**  
   Data tables use `\small` instead of `\footnotesize` (slightly larger body text).

6. **Motivation / Guard-vs-B / SAU aggregate (final polish)**  
   - Motivation no longer says “without compromising … security guarantees”; it says we preserve temporal consistency of each block measurement.  
   - Guard \tdis{} described as nearly independent of $B$; no SAU-hole causal claim.  
   - Aggregate SAU cost at $B{=}64$ stated as approximately $1.6$--$2.7$~ms (not $\ll 2$~ms).

## Still for the author (assets / Overleaf)

- Fix figure typo “Taks” → “Tasks”.  
- Re-upload regenerated `tdisabled_three_log.pdf` if not already on Overleaf.  
- Compile on Overleaf → PDF eXpress → camera-ready submission (due 5 Nov 2026).

## Short cover note (EN)

> Dear chairs / editors,  
> Please find an updated camera-ready draft. Relative to the previous revision we (i) separated Snap/Guard security claims so Guard no longer asserts identical order privacy under concurrent Non-Secure accesses, (ii) moved Related Work earlier, (iii) clarified runtime and coverage wording for the remeasured 128 KiB harness, and (iv) slightly enlarged table fonts. New text is highlighted in blue via `\ben{}` for review convenience.
