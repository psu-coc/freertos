# SMARM+ — FreeRTOS vs Zephyr Wheel-1000Hz Scheduling Equivalence Audit

**Mode:** STRICT READ-ONLY scientific / source audit  
**Date:** 2026-11-06  
**Status:** STOP after audit — no code, build, flash, experiment, result, or manuscript changes.

## Research question

Observed Wheel-1000Hz inter-start jitter (Original):

| Platform | B64 | B1024 |
|---|---:|---:|
| FreeRTOS (audited primary) | 11.3 µs | 85.1 µs |
| Zephyr TIM2-corrected Gate 3 | ~992 µs | ~993 µs |

Zephyr no-attestation controls also show ~994 µs with alternating ~1990 µs / ~6 µs intervals.

**Question:** Are the two experimental implementations actually equivalent, or is the gap an implementation / harness mismatch rather than an inherent RTOS property?

---

## Evidence and source availability

### FreeRTOS (complete on Windows)

| Item | Path |
|---|---|
| Wheel task loop | `C:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\NonSecure\Src\main.c` (`WheelSmoke_Task`, wall-obs path) |
| Profile (1000 Hz / 3 s) | `NonSecure\Inc\rt_experiment_profile.h` |
| Tick / preemption | `NonSecure\Inc\FreeRTOSConfig.h` |
| `osDelayUntil` | `Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS_V2\cmsis_os2.c` |
| `vTaskDelayUntil` | `Middlewares\Third_Party\FreeRTOS\Source\tasks.c` |
| Audited jitter | `advisor_evidence_v1\jitter_summary.csv` |
| Raw starts | `dmr\raw\dmr_Wheel1000Hz_*\*_formal_dmr.csv` |

### Zephyr Gate 3 (partial source on Windows)

| Item | Path | Note |
|---|---|---|
| Campaign cell | `D:\smarm\SMARMPlus_Archive_20261105\Research_Evidence\Gate3_TIM2_Corrected_20261106\original-b64-wheel1000hz\` | UART + ELF + `.config` + `zephyr-workload.diff` |
| No-attest control | `...\Wheel1000_NoAttest_Control_20261106\` | Confirms pattern without attestation |
| Closest full `rt_workload.c` text | `...\smarmplus-evidence-20261103-rt-profiles\source\rt_workload.c` | **Pre-TIM2** `rt_now_us()` uses `k_cycle_get_64()` |
| Gate 3 HEAD | `306b85bbb6ca5f64e12e1b9a7ba13abbdc3a01e4` | `git-status.txt`: `src/rt_workload.c` is **untracked (`??`)** and **not** present as a full file in the Windows diff |

**Missing on Windows (exact Gate 3 TIM2 application tree):**

- `/home/laoamarin/Desktop/master/zephyr/samples/tfm_integration/smarm_experiment/src/rt_workload.c` (TIM2-era)
- Matching `rt_workload.h` / board overlays as built for Gate 3

**What can still be verified without that file:**

1. UART `RT_TIMEBASE source=TIM2 counter_hz=1000000` on every Gate 3 / control cell checked.
2. ELF symbols `rt_now_us`, `rt_sleep_until`, `wheel_thread` in `zephyr.elf` (`zephyr.map`).
3. Archived `rt_workload.c` sleep/wheel loop text matches the no-attest control report’s description of `rt_sleep_until` overdue-immediate behavior.
4. Raw interval structure is independent of attestation.

**Audit rule applied:** scheduling-semantics conclusions below are based on archived loop + Gate 3 binaries/UART/controls. Exact TIM2 `rt_now_us` line-by-line text is **INSUFFICIENT EVIDENCE** on Windows; timebase equivalence is still established at the measurement-output level.

---

## Phase A — Scheduling implementation comparison

### A.1 FreeRTOS Wheel-1000Hz loop (wall observation)

Profile (`rt_experiment_profile.h` 77–101):

```77:88:C:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\NonSecure\Inc\rt_experiment_profile.h
#elif RT_SENSOR_PROFILE == RT_PROFILE_WHEEL_1000HZ_WALL3
/* HR probe: 1 kHz × 3 s ≈ 3000 jobs (RAM-feasible). NOT comparable to wall30.
 * SysTick period == P (1 ms) — expect fragile DelayUntil; report honestly. */
...
#define RT_WHEEL_PERIOD_MS         1U
...
#define RT_OBS_DURATION_US         (3U * 1000000U)
#define RT_OBS_PERIOD_US           1000U
```

Warm-up / T0 (`main.c` 1030–1058): last ready task sets `g_obs_epoch = us_now()`, `T0 = epoch + 2 s`, busy-waits with `osDelay(1)` until `us_now() >= T0`.

Wheel loop core (`main.c` 2293–2348, 2482–2485):

```2293:2348:C:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\NonSecure\Src\main.c
    const uint32_t period_ms = PHASE2_WHEEL_PERIOD_MS;
    const uint32_t period_us = period_ms * 1000U;
    ...
    next = g_wheel_tick_entry;
    if (next >= period_ms) {
      next -= period_ms;
    }
    for (;;) {
      ...
      window_open_us = us_now();
      if (window_open_us >= g_obs_tend) {
        break;
      }
      ...
      next += period_ms;
      sched_wake_tick = next;
      tick_before = osKernelGetTickCount();
      dly_st = osDelayUntil(next);
      ...
      s_exec_us = us_now(); /* post-DelayUntil — compute eligibility */
```

Then edge count / optional inject / `finish_us = us_now()`; store record; `job++`.

**CMSIS `osDelayUntil`** (`cmsis_os2.c` 898–924):

```898:924:C:\Users\PC\STM32CubeIDE\workspace_2.0.0\freertos\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS_V2\cmsis_os2.c
osStatus_t osDelayUntil (uint32_t ticks) {
  ...
    tcnt = xTaskGetTickCount();
    delay = (TickType_t)ticks - tcnt;
    if((delay != 0U) && (0 == (delay >> (8 * sizeof(TickType_t) - 1)))) {
      vTaskDelayUntil (&tcnt, delay);
    }
    else
    {
      /* No delay or already expired */
      stat = osErrorParameter;
    }
```

If the absolute target is already overdue → **immediate return** (`osErrorParameter`), no block.  
If still in the future → `vTaskDelayUntil` waits until `now + remaining` (= absolute target when `tcnt` was latched as now).

Tick: `configTICK_RATE_HZ = 1000` (`FreeRTOSConfig.h` 71). Preemption on. Wheel priority: `osPriorityAboveNormal` (`main.c` 193–197). Attest: `osPriorityLow`.

### A.2 Zephyr Wheel-1000Hz loop (archived text; Gate 3 binary present)

Header period (`rt_workload.h` 31–34): `SMARM_RT_PERIOD_US 1000`, `SMARM_RT_OBS_US 3000000`, warmup 2 s.

`rt_sleep_until` (archived `rt_workload.c` 47–57):

```47:57:D:\smarm\SMARMPlus_Archive_20261105\Research_Evidence\smarmplus-evidence-20261103-rt-profiles\source\rt_workload.c
static void rt_sleep_until(uint32_t target_tick)
{
	for (;;) {
		uint32_t now = (uint32_t)k_uptime_ticks();
		int32_t delta = (int32_t)(target_tick - now);
		if (delta <= 0) {
			return;
		}
		k_sleep(K_TICKS((uint32_t)delta));
	}
}
```

Wheel thread (archived `rt_workload.c` 227–259):

```227:259:D:\smarm\SMARMPlus_Archive_20261105\Research_Evidence\smarmplus-evidence-20261103-rt-profiles\source\rt_workload.c
static void wheel_thread(void *a, void *b, void *c)
{
	uint32_t next, job = 0U;
	...
	next = observation_tick0;
	for (;;) {
		...
		tick = next + k_us_to_ticks_ceil32(SMARM_RT_PERIOD_US);
		open = rt_now_us();
		if ((int32_t)(open - observation_tend_us) >= 0) {
			break;
		}
		edges0 = wheel_edges;
		next = tick;
		rt_sleep_until(next);
		start = rt_now_us();
		...
		finish = rt_now_us();
		rt_record(..., next, start, finish, open, true, ...);
		job++;
	}
}
```

Gate 3 `.config`: `CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000`, `# CONFIG_TICKLESS_KERNEL is not set`, `CONFIG_TIMESLICING=y`.  
Priorities in archived announce: Wheel `K_PRIO_PREEMPT(4)`, Sonar `(5)`, dump `(6)` (Zephyr: lower number = higher priority).

ELF: `rt_sleep_until` size **0x18 bytes** — consistent with the tiny overdue/relative-sleep helper, not a distinct absolute-timeout API.

### A.3 Dimension-by-dimension classification

| # | Dimension | FreeRTOS | Zephyr (Gate 3 / archive) | Class |
|---|---|---|---|---|
| 1 | Initial release / epoch | Shared obs T0 after 2 s warmup; wheel `next` primed to entry−P | Shared `observation_tick0` after warmup sleep-until | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** (TIM2 vs tick sync details differ) |
| 2 | Period increment | `next += 1` tick (`period_ms=1`) | `next += k_us_to_ticks_ceil32(1000)` = **1** tick | **EQUIVALENT** (both advance one kernel tick per job) |
| 3 | Absolute vs relative wait | Absolute target via `osDelayUntil(next)`; CMSIS implements as remaining ticks from now | Absolute target argument, but wait is **`k_sleep(K_TICKS(delta))` relative** | **NOT EQUIVALENT** |
| 4 | Overdue target | Immediate return (`osErrorParameter`) | Immediate return (`delta <= 0`) | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** (same idea; different return/status) |
| 5 | Missed periods skip vs catch-up | No explicit skip; overdue → run immediately, then still `next += P` (catch-up iterations possible) | Same: overdue → run immediately, `next` still advances by 1 | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** |
| 6 | Kernel tick / conversion | 1000 Hz; period = 1 tick | 1000 Hz; period = 1 tick | **EQUIVALENT** (and both fragile by design) |
| 7 | Wake mechanism | FreeRTOS delayed-list / `vTaskDelayUntil` | Zephyr `k_sleep` timeout | **NOT EQUIVALENT** |
| 8 | Priority / preemption / tick config | Wheel AboveNormal; preempt; 1 kHz | Wheel prio 4; preempt; 1 kHz; timeslicing on | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** |
| 9 | Work before/after start stamp | `window_open` before sleep; **`s_exec` after sleep**; light edge work; finish after | `open` before sleep; **`start` after sleep**; edge delta; finish after | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** |
| 10 | Warmup / stop boundaries | 2 s warmup; stop when `window_open >= Tend` (pre-sleep) / out-of-window start | 2 s; stop when `open >= tend`; boundary/censored rows in dump | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** |

**Do not treat `vTaskDelayUntil` / `osDelayUntil` and Zephyr `rt_sleep_until`→`k_sleep` as semantically equivalent.** Same intended “sleep until absolute tick,” different primitives and observed wake spacing at P = 1 tick.

---

## Phase B — Measurement pipeline comparison

### FreeRTOS pipeline

1. Nominal release identity: `sched_wake_tick` (= `osDelayUntil` target).  
2. Sleep/unblock: `osDelayUntil`.  
3. Actual start: `s_exec_us = us_now()` (TIM2 µs) immediately after delay.  
4. Raw record: DMR / phase2 buffers → UART/CSV.  
5. Jitter (audited): mean `|Δs − P|` after **dropping first in-window start** (`rt_metrics_wall30.score_regularity`).  
6. Aggregation: often 3 files; some Wheel1000 cells collapse to one distinct timing trace (`jitter_summary.csv`).

### Zephyr pipeline

1. Nominal: `sched_wake_tick = next` after `+1` tick.  
2. Sleep/unblock: `rt_sleep_until` / `k_sleep`.  
3. Actual start: `start = rt_now_us()` after sleep (Gate 3 UART proves TIM2 1 MHz).  
4. Raw: `RT_REC` UART rows.  
5. Jitter (firmware): mean `|interval − PERIOD|` over **all consecutive recorded starts** (includes first pair).  
6. Aggregation: **n = 1** per Gate 3 cell; noattest B64 has 3 runs (same pattern).

### Measurement dimension classes

| Topic | Class | Notes |
|---|---|---|
| TIM2 clock / units | **EQUIVALENT** (Gate 3 / FreeRTOS both 1 MHz µs) | Exact Zephyr `rt_now_us` source lines missing on Windows |
| Recorded timestamp semantics | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** | Both: post-sleep start; FreeRTOS also logs `window_open` |
| Interval formula | **EQUIVALENT** | `s[i]-s[i-1]`; jitter `|Δ−P|` |
| Expected period | **EQUIVALENT** | 1000 µs |
| First-sample exclusion | **NOT EQUIVALENT** | FreeRTOS primary drops first; Zephyr firmware does not |
| Boundary filtering | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** | Both have censored/boundary concepts; IDs differ |
| Per-run aggregation | **NOT EQUIVALENT** | FreeRTOS multi-run / distinct-trace rules vs Zephyr n=1 |
| Catch-up iterations | **NOT EQUIVALENT in effect** | Both *can* catch up; Zephyr *systematically* pairs catch-up at 1000 Hz |
| Incomplete records | **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL** | Different status enums / Formal DMR quarantine on Zephyr |
| Independent runs | **NOT EQUIVALENT** | FreeRTOS typically 3; Zephyr Gate 3 single-shot |

**Non-equivalence that matters for the 900+ µs claim:** not the `|Δ−P|` algebra, but the **start-time process** feeding it (Zephyr’s systematic 1990/6 wake pattern).

---

## Phase C — Mechanism validation (from raw evidence)

### C.1 Why Zephyr shows ~1990 / ~6 µs alternation

Gate 3 Original B64 Wheel1000 (`fresh-uart.log`):

- First intervals: `1995, 9, 1988, 7, 1993, 7, ...`
- Buckets: **1499** intervals `< 50 µs`, **1500** near 2 ms, **0** near 1 ms
- `sched_wake_tick` deltas: **always +1** (nominal 1 kHz identity advances correctly)
- Mean interval ≈ 1000.3 µs (two starts per ~2 ms), mean `|Δ−P|` ≈ 993 µs

No-attest B64 run1:

- Intervals: `1992, 6, 1994, 6, ...` (same structure)
- `window_open_before_ms` counts: **(0, 1500), (2, 1500)** exactly alternating  
  - After long (~2 ms) gap: `open_before_ms = 2` → sleep lasted ~2 ms with window opened before sleep  
  - After short (~6 µs) gap: `open_before_ms = 0` → **immediate overdue return** (catch-up)

**Mechanism (verified):** with `CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000` and period = 1 tick, Zephyr `k_sleep(K_TICKS(1))` in this harness consistently produces ~2 ms blocking, so the next absolute tick target is already overdue → immediate second start → pair pattern. Mean `|Δ−1000|` ≈ 994 µs is the arithmetic consequence, not attestation load.

### C.2 FreeRTOS release / catch-up

FreeRTOS Original B64 and noattest Wheel1000 formal DMR `s_us`:

- Intervals ≈ `999–1000` µs throughout  
- Buckets: **~3000 near 1 ms**, **0** short, **0** near 2 ms  
- `sched_wake_tick` also advances by 1 each job  

So FreeRTOS uses the same *nominal* +1-tick catch-up rule, but **does not** enter the systematic overdue-pair regime at B64 / noattest. Original B1024 shows some stretched intervals (attestation interference) but **not** the Zephyr 1990/6 lattice.

### C.3 1 ms kernel tick boundary

Both use 1 kHz ticks with P = 1 ms. FreeRTOS’s own profile comment warns this is fragile — yet FreeRTOS measurements remain near-period-locked. Zephyr’s relative `k_sleep` path does not. **The difference is therefore not “1 ms tick makes 1 kHz impossible on any RTOS,”** but “this Zephyr sleep harness + 1-tick period produces a different wake process than FreeRTOS `osDelayUntil`.”

### C.4 Nominal targets

Zephyr `sched_wake_tick` monotonic +1 ⇒ nominal identities advance correctly. The failure mode is **wake/start time vs nominal**, not stalled indices.

### C.5 Two starts per ~2 ms cycle

Yes — empirically half the intervals are ~6 µs catch-ups; half are ~1990 µs. That is two recorded starts per ~2 ms wall cycle while claiming 1 ms period.

### C.6 No-attestation isolation

`WHEEL1000_NOATTEST_CONTROL_REPORT.md`: B64 3/3 and B1024 1 valid run reproduce ~994 µs and the same bins with `SMARM_RT_ATTEST_ENABLE=0`. **Attestation is ruled out as the cause of the ~992 µs floor.**

---

## Phase D — Scientific verdict

### Explicit answers

1. **Same 1000 Hz periodic-workload semantics?**  
   **No.** Nominal period and +1-tick indexing match; **actual release/wake semantics do not.** Class: **NOT EQUIVALENT**.

2. **Same actual task-start measurement?**  
   **Mostly yes in intent** (post-sleep TIM2/µs start). Minor harness differences remain. Class: **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL**. Exact Gate 3 `rt_now_us` source text: **INSUFFICIENT EVIDENCE** on Windows (binary/UART still show TIM2).

3. **Consistent inter-start jitter calculation?**  
   Formula `|Δs−P|` matches; **first-sample policy and aggregation do not.** Class: **FUNCTIONALLY SIMILAR BUT NOT IDENTICAL**. Not the driver of the 900+ µs gap.

4. **Is the 900+ µs difference from a verified implementation difference?**  
   **Yes.** Verified Zephyr absolute-target + `k_sleep(1-tick)` catch-up pair pattern (including no-attest). Not explained as “Zephyr is just a slower RTOS” or “attestation.”

5. **Does current Zephyr Wheel-1000Hz fairly replicate FreeRTOS?**  
   **No for inter-start jitter / start regularity.**  
   **Partial** for coarse availability / “jobs still logged ~3000 times in 3 s” (mean rate ~1 kHz via paired starts).  
   Class: **NOT EQUIVALENT** as a fairness claim for jitter.

6. **Would any correction be a new protocol needing approval?**  
   **Yes.** Changing tick rate, replacing `k_sleep` with absolute timeouts / TIM2 alarms, or redefining catch-up would alter the experiment and require researcher approval before re-running or re-claiming cross-RTOS numerical jitter parity.

### Summary classification table (critical dimensions)

| Dimension | Verdict |
|---|---|
| Intended 1 kHz Wheel profile (P, warmup, duration, NS board) | FUNCTIONALLY SIMILAR BUT NOT IDENTICAL |
| Kernel tick = 1 ms = P | EQUIVALENT |
| Sleep primitive / wake spacing at 1000 Hz | **NOT EQUIVALENT** |
| Overdue immediate catch-up *rule* | FUNCTIONALLY SIMILAR BUT NOT IDENTICAL |
| Overdue catch-up *observed behavior* | **NOT EQUIVALENT** |
| Start timestamp after wake (TIM2 µs) | FUNCTIONALLY SIMILAR BUT NOT IDENTICAL |
| Jitter algebra | EQUIVALENT |
| Jitter sampling/aggregation policy | NOT EQUIVALENT |
| Fair 1000 Hz jitter replication | **NOT EQUIVALENT** |
| Exact Gate 3 TIM2 `rt_workload.c` text on Windows | **INSUFFICIENT EVIDENCE** |

---

## Scientific consequences

1. **Do not attribute the FreeRTOS 11–85 µs vs Zephyr ~993 µs Wheel-1000Hz gap to SMARM+ attestation or to a general Zephyr vs FreeRTOS performance ranking.**
2. **Wheel-1000Hz jitter is not currently a valid cross-RTOS equivalence metric** under the existing Zephyr harness.
3. Dual-10 Hz / Wheel-100 Hz comparisons (P ≫ tick) remain the stronger portability evidence for qualitative stress trends; 1000 Hz needs a scheduling-equivalence fix before numerical jitter claims.
4. Manuscript / thesis: FreeRTOS remains primary. Zephyr 1000 Hz may be described as **supporting availability evidence with a documented non-equivalent 1-tick sleep artifact**, not as matched inter-start jitter.

---

## Minimal proposed correction (NOT implemented)

Any one of the following would be a **new experimental protocol** (researcher approval required):

1. **Prefer:** absolute timeout API (e.g. Zephyr absolute tick timeout / timer) so wait is until `target`, not `k_sleep(relative delta)` at P = 1 tick.  
2. Raise `CONFIG_SYS_CLOCK_TICKS_PER_SEC` (e.g. 10 kHz) so P spans multiple ticks — still a protocol change; recalibrate all profiles.  
3. Drive releases from a **hardware compare** (TIM2 alarm) shared with the timebase.  
4. After a fix: new no-attest + Original B64/B1024 Wheel1000 campaign; keep old Gate 3 1000 Hz jitter labeled as pre-correction harness-limited.

**Do not** silently edit firmware to force numerical agreement with FreeRTOS, and do not rewrite Gate 3 results in place.

---

## Unresolved questions

1. Exact Gate 3 TIM2 `rt_now_us` / init source lines (file not archived on Windows).  
2. Why FreeRTOS `osDelayUntil` avoids the 2 ms overshoot at the same 1 kHz tick while Zephyr `k_sleep(1)` does not — kernel-internal timeout accounting (needs Zephyr kernel source citation on the build machine).  
3. Whether Zephyr Dual10/Wheel100 cells using the same `rt_sleep_until` are affected only mildly because P ≥ 10 ticks (expected, not re-proven here in depth).

---

## Final statement

**The two Wheel-1000Hz implementations are not experimentally equivalent for release/start regularity.**  
They share nominal 1 kHz indexing and post-wake TIM2-style start logging intent, but Zephyr’s `rt_sleep_until`→`k_sleep(K_TICKS(delta))` path at a 1 ms tick produces a verified ~1990/6 µs catch-up lattice (including no-attest), which FreeRTOS does not. The ~900+ µs jitter gap is a **verified harness/scheduling non-equivalence**, not an attestation effect.

**STOP.** Awaiting researcher approval before any correction, rebuild, or re-measurement.
