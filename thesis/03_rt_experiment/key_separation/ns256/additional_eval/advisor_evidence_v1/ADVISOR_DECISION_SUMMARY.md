# Advisor decision summary

Verification status: **PASS**.

This note does not choose an option and does not revise the manuscript.

## Results that the decision has to accommodate

- Original SMARM, 10 Hz, B = 128: FAR 1.350000 → 0.964798. Mean of ten per-round ratios. Not clipped.
- SMARM+FPE, 10 Hz, B = 2048: FAR 1.000000 → 0.673401. The integer expectation hid this drop.
- At 1000 Hz the original and continuous FAR means are the same, because the archived duration is an integer millisecond and the expected count is that millisecond count.
- Wheel only, 100 Hz, Original, B = 2048, run 1: fixed DMR 0.999333; sliding miss ratio 0.000000; sliding eligible 1629; excluded 2; unexecuted 1368.
- The sliding ratio excludes those unexecuted obligations. It is not formal DMR and not a standard soft deadline.
- Jitter is the mean of `|Δs − P|` in microseconds after the first in-window start is dropped. Identical high-rate files are one trace.

## Options left open

1. Recalculated FAR, inter-start jitter, and fixed formal DMR.
2. Option 1, plus the sliding-reference ratio only as a supplement that prints eligible, excluded, and unexecuted counts.
3. A different deadline definition, specified by the advisor before any deadline text is rewritten.

Option 1 changes the 10 Hz FAR narrative relative to the floored expectation and leaves fixed DMR as the deadline result. Option 2 adds a metric that can read as healthy while fixed DMR is near 1. Option 3 keeps both current calculations as records of their own definitions.

## Approval still required

- Which of the three options is the deadline and FAR reporting choice.
- Whether a later manuscript revision may replace the 10 Hz FAR numbers.
- Whether the sliding ratio may be shown outside this evidence directory.
- The wording of any alternative deadline, if option 3 is taken.
