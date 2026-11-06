# Methodology and limitations

All numbers below were recomputed from existing measurements. No board was run. Values above 1 were not clipped.

## B. Continuous FAR

Firmware (`NonSecure/Src/main.c`) samples `g_normal_counter` and TIM2 around `SECURE_ShuffledHMAC_secure`. It logs only

```
runtime_ms = floor(tim2_diff * 1000 / 137500)
expected = floor(runtime_ms * target_Hz / 1000)
FAR_original = actual / expected
```

The UART line `Round N: Runtime=... ms, NS=actual/expected` and the rounds CSV do not store `tim2_diff`. The finest elapsed time that can be recovered is the integer millisecond. TIM2 itself ticks every `1/137500` s, about 7.27 µs, but that count is not in the log. The unused fraction of the last millisecond is therefore still missing, and it is strictly less than 1 ms.

```
FAR_continuous = actual_count / (target_Hz * runtime_ms / 1000)
```

The mean is the mean of the ten per-round ratios, matching `analyze_ns256_far.py`. The stored four-decimal firmware FAR was checked against the same rounding, `(actual * 10000 + expected // 2) // expected`.

At 1000 Hz, `floor(runtime_ms * 1000 / 1000) = runtime_ms`, so the original and continuous means are identical. The integer-millisecond log cannot improve that frequency further. At 10 Hz the original denominator collapses to 2 or 3, which is why means such as 1.350 become 0.965. Individual rounds can still sit slightly above 1 because a window of length T can contain one more release than `f*T`. FAR-REF at 10 Hz, with interrupts left enabled, is 1.020 at B = 1024 and 0.994 at B = 4096 after the continuous formula. That excess is phase, not a claim that the task ran faster than requested.

FPE at 10 Hz and B = 2048 stays at original FAR 1.000 because both the count and the floored expectation are 2, while the continuous value is 0.673. The original formula hides that drop.

## C. Exploratory sliding-reference miss ratio

Formal DMR is copied from `FINAL_DMR_RESULTS.csv`. It is not recomputed as a new definition. The script checks that the nominal lattice length equals that file's `eligible` and that jobs with no stored row equal `unexec`.

Nominal identity is `sched_wake_tick`. A job is never identified from its timestamp.

For a lattice job `i` whose immediate predecessor `i-1` also has a valid start and finish:

```
r_slide = s[i-1] + P
d_slide = s[i-1] + 2P
miss if f[i] > d_slide
```

The denominator includes only those pairs.

Initialization: the first eligible nominal job has no predecessor, so it is excluded. If it has a valid start, that start is used only as `s[i-1]` for the next job. It is not counted as a success or a miss.

A nominal job with no row is unexecuted. It is excluded from the sliding denominator and reported in `unexecuted`. A stored row with `s == 0`, `f == 0`, or `s >= Tend` is `invalid_excluded`. If the current job has valid timestamps but its immediate predecessor does not, the current job is `gap_excluded`. In these captures `gap_excluded` is 0: the missing jobs form a suffix after a contiguous executed prefix, not a hole in the middle. The rule is still applied.

The ratio is blank when the denominator is 0. That did not occur. Excluded jobs are not treated as on time. Runs are not averaged. Wheel 100 Hz and 1000 Hz have three capture files and one distinct timing trace, so the run-1 row is the trace, not an independent sample of three. Sonar and Wheel at 10 Hz have three distinct traces; the comparison file keeps every run that the formal audit marked `table_row=yes`.

## Why sliding can look healthy while fixed DMR is near 1

Wheel 100 Hz, Original, B = 2048, run 1: fixed DMR is 0.999333, with 1368 unexecuted jobs. The sliding ratio on the 1629 consecutive executed pairs is 0. Those jobs are late against the original release grid, but each still finishes within `2P` of the previous actual start. The sliding ratio does not see the unexecuted tail.

Wheel 100 Hz, Original, B = 4096: the executed pairs themselves miss the `2P` test, so the sliding ratio is 1 as well.

## Worked timestamps (Wheel 100 Hz, run 1, P = 10000 µs)

B = 1024, second job, tick 2022. Finish 2034507 is before the fixed deadline 2043576 and before the sliding deadline 2043588. Neither test misses.

B = 2048, tick 2032. Finish 2056676 is after the fixed deadline 2053576, so the fixed test misses. The sliding deadline is 2059817, so the sliding test does not miss.

B = 4096, tick 2022. Finish 2055546 is after both the fixed deadline 2043576 and the sliding deadline 2043588. Both tests miss.

## Relation to inter-start jitter

Wheel's finish is a few microseconds after its start, so a sliding miss on Wheel is almost the same event as an inter-start gap larger than `2P`. The frozen Gate 3 jitter figures are a different capture and a different summary (a mean, not a miss ratio). At B = 1024 those means are far below one period, which agrees with a sliding ratio of 0. At B = 4096 the reported jitter is large, which agrees with a high sliding ratio. The two numbers should not be pooled.

## Checks performed

- Every FAR cell has 10 rounds.
- `expected` in the CSV equals `floor(runtime_ms * Hz / 1000)`.
- Stored firmware FAR matches the documented rounding.
- Sliding classes partition the formal eligible lattice.
- Unexecuted counts match `FINAL_DMR_RESULTS.csv`.
- Duplicate `sched_wake_tick` values abort the script. None were found.
