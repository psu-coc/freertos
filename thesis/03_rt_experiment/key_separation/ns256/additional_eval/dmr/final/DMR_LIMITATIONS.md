# Limitations attached to the final DMR

## One Wheel timing trace

Wheel 100 Hz and Wheel 1000 Hz each have three capture files per configuration. The job timing hash is the same in all three. The tables report that one trace. They do not report n = 3, and they do not support a confidence interval.

The files are separate acquisitions. The capture script opens COM3, clears the serial input buffer, hard-resets the board, and refuses to overwrite an existing UART file. Host timestamps differ. The firmware then emits the same job timestamps. A fresh capture and an independent timing outcome are different facts. Only the first is established for these Wheel runs.

## Boot counter on Wheel 100 Hz no-attestation

The three no-attestation files print boot numbers 2, 2, and 4. The counter is a `.noinit` SRAM word. A hard reset increments it. A flash can destroy that word or leave it in place, so two fresh flashes of the same image can both capture boot 2. Runs 1 and 2 are byte-identical, including that boot digit, and were written about five minutes apart. Run 3 differs by that one digit and has the same formal CSV hash. The repeated counter is a metadata limitation. It is not evidence that a capture was reused, and a changed counter would not by itself be evidence of a new timing outcome.

## 10 Hz release anchor

The 10 Hz captures predate the systick-edge line. Their nominal release is the TIM2 value stored with each row, `T0 + (sched_wake_tick - tick0) * 1000`. The first wake sits about 75 µs after T0. Shifting the lattice by that amount does not change any 10 Hz DMR. These tables keep the logged release.

## Eligible count at 100 Hz and 1000 Hz

The Wheel lattice starts at `tick0_tim2_us`, which is the measured time of the latched tick and is later than T0. The release whose deadline would fall after Tend is right-censored. The denominator is 2999 on every Wheel configuration. The censored column counts logged rows past that deadline, so it can be 1 or 2 while the excluded obligation is already outside the denominator.

## One-job movement at 10 Hz

Three 10 Hz configurations move a single job between late and unexecuted across otherwise distinct traces. The miss total is the same, so the reported DMR does not change.

- Original B2048, run 2: late 162 and unexecuted 135, against 163 and 134 on runs 1 and 3.
- PRNG B2048, run 3: late 164 and unexecuted 133, against 163 and 134 on runs 1 and 2.
- PRNG B4096, run 2: late 84 and unexecuted 214, against 83 and 215 on runs 1 and 3.

Sonar and Wheel show the same shift in each case. Incomplete stays 1.

## Wheel no-attestation traces at 10 Hz

Runs 2 and 3 of the 10 Hz no-attestation capture share one Wheel timing hash. Run 1 differs. The three UART hashes differ, and the three Sonar timing hashes differ. DMR is 0.000000 for both tasks on every run.

## Figures that are not these results

The stored-row campaign DMR and the slot-based recovery answer different questions. A slot score assigns a completion to whichever period contains its start time. The formal score keeps the completion on the release identity stored with that iteration. The final tables use the formal score only.
