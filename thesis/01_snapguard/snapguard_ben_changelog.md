# Snap/Guard — `\ben{}` change log (track by hand)

**Pristine Overleaf (no edits):** `snapguard_overleaf_10062026_pristine.tex`  
**Working file (blue edits):** `snapguard_overleaf_10062026.tex` (= same as `snapguard_revision.tex`)  
**How to find edits in .tex:** search `\ben{` or `% OLD:`

> Cursor “Keep change” only appears on files the agent just edited in that turn.  
> For tracking content, use `\ben{...}` (blue in PDF) + this list.

| # | Reviewer | Location (approx) | What changed |
|---:|---|---|---|
| 1 | m2 | Abstract | Orders unified; FAR measured claim |
| 2 | m2, M4 | Intro contributions | 15× / 3–4 orders + FAR |
| 3 | m5, M5 | Sec. III.A Motivation | Software SHA-256; new t_dis numbers |
| 4 | m6 | Sec. III.D Guard | One SAU window vs lock+unlock sum |
| 5 | M2 | Sec. III.E Security | **Rewritten:** S1/S2/S3 split; Guard does **not** claim identical S3 (fault-timing side channel); Snap keeps strong S3 |
| 6 | M3, m3, m4 | Sec. IV.A Setup | 128 KiB window; flash 256+256; B=64; HardFault demo; **+ production coverage guidance** (Snap/baseline can cover full NS XIP) |
| 7 | M5, m1 | Sec. IV.B | DWT method; mean/worst; new prose + **NEW** `tab:tdisabled` |
| 8 | M4 | Sec. IV.D | Measured FAR prose + **NEW** `tab:far` |
| 9 | M1 | Sec. IV.E + Table I | 128 KiB runtimes; Guard ≉ faster; Snap +7–8 ms; **no “platform variation” alone** → Secure-world jitter wording |
| 10 | M6 | Sec. IV.F + Table II | TZ-M baseline row; Guard not blanket RT-safe; S3 caveat in footnote |
| 11 | m2, M3 | Conclusion | New numbers / guidance + Guard S3 side-channel pointer |
| 12 | R3.2 | Structure | **Related Work moved** to after Introduction (before Background) |
| 13 | R3 fonts | All data tables | `\footnotesize` → `\small`; slightly wider `\tabcolsep` |
| 14 | M2 consistency | Sec. III.A Motivation | Drop “without compromising … security guarantees” → preserve temporal consistency of each block |
| 15 | m1 | Sec. IV.B t_dis prose | Guard ≈ independent of B; small variation = measurement noise (cut SAU-hole causal claim) |
| 16 | M1 | Sec. IV.E runtime | Aggregate SAU cost ≈ 1.6–2.7 ms at B=64 (not ≪ 2 ms) |
| 17 | M2 | Sec. III.E S3 | Cut “strong (storage) sense”; add adversarial probing does not stop malware; no baseline-equivalent order privacy under Guard |
| 18 | M2/S1 | Background S1 | Goal = temporal consistency of capture (not “IRQ only between blocks”) |
| 19 | M1 | Sec. IV.E | Conservative residual-gap wording; “copy overhead” |
| 20 | CR format | Preamble | Removed `geometry` + float-spacing hacks; unblinded `\author`; “Anonymous” → “Prior work” |

**Figures (not `\ben` — you edit assets):** “Taks” typo; re-upload `tdisabled_three_log.pdf`

**Count:** search `\ben{` in working `.tex` after latest polish
