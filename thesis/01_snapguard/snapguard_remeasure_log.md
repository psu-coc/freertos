# Snap/Guard remeasure log (paper redo)

Config for all runs below unless noted:
- Branch: `smarm` (unified harness)
- Window: `|M|=128 KiB` @ `0x08060000–0x0807FFFF`
- `ATTEST_PASSES=1` (single shuffled traversal)
- Platform: STM32L552ZE @ 110 MHz, FreeRTOS / TrustZone-M
- Rounds: 10 per configuration
- Timer: TIM2 (same harness as before)

Goal: same binary family for Baseline / Snap / Guard. Do **not** mix with old 128×4 or 512 KiB single-pass dumps.

---

## Runtime — no NormalTask (Table I)

### Guard — B = 512

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **125 / 125 / 125 ms**
- **SAU_config_avg_cycles:** **94** (all 10 rounds)
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Note:** Previous 4-pass run at same B was 499 ms ≈ 4 × 125 ms

```text
SMARM-Guard B=512 |M|=128KiB @0x08060000, 1 pass/round (=128KiB hashed; SAU every block) (10 rounds).
Round 1–10: Runtime=125 ms, NS=0/125, FAR=0.0000, SAU_config_avg_cycles=94
=== SUMMARY === Mean: 125 ms | Min: 125 | Max: 125 | FAR mean=0.0000
```

### Snap — B = 512

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **132 / 132 / 133 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Guard:** +7 ms (Snap slower than Guard, as expected)

```text
SMARM-Snap B=512 |M|=128KiB @0x08060000, 1 pass/round (=128KiB hashed; memcpy IRQ-off) (10 rounds).
Round 1–10: Runtime=132–133 ms, NS=0, FAR=0.0000
=== SUMMARY === Mean: 132 ms | Min: 132 | Max: 133 | FAR mean=0.0000
```

### Baseline — B = 512

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **125 / 125 / 125 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

```text
SMARM Baseline B=512 |M|=128KiB @0x08060000, 1 pass/round (=128KiB hashed; HMAC IRQ-off) (10 rounds).
Round 1–10: Runtime=125 ms, NS=0, FAR=0.0000
=== SUMMARY === Mean: 125 ms | Min: 125 | Max: 125 | FAR mean=0.0000
```

**B=512 three-mode note (reviewer Q1):** Baseline 125 = Guard 125; Snap 132.
At B=512 there are 256 blocks; SAU overhead ≈ 256 × 94 cycles ≈ 0.22 ms @ 110 MHz, below the 1 ms UART reporting quantum. So Guard ≈ Baseline is expected here; Guard is not faster than Baseline.

### Baseline — B = 128

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=128`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **142 / 142 / 142 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Note:** Higher than B=512 (125 ms) due to more per-block IRQ on/off overhead (1024 blocks vs 256).

### Snap — B = 128

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=128`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **149 / 149 / 149 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +7 ms

### Guard — B = 128

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=128`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **140 / 140 / 140 ms**
- **SAU_config_avg_cycles:** **88**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **−2 ms** (Guard still slightly faster; reviewer Q1 not fully gone at this B)
- **SAU cost estimate:** 1024 blocks × 88 cycles ≈ 0.82 ms @ 110 MHz — so Guard was expected ≈ Baseline or +1 ms, not −2 ms. Keep measuring; may need a later explanation (path/cache/IRQ-off overhead).

### Baseline — B = 256

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=256`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **131 / 131 / 131 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

### Snap — B = 256

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=256`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **138 / 138 / 138 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +7 ms

### Guard — B = 256

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=256`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **130 / 130 / 130 ms**
- **SAU_config_avg_cycles:** **90**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **−1 ms** (same mild pattern as B=128)

**Priority B trio status (reviewer Q1):**

| B | Baseline | Snap | Guard | Guard−Baseline |
|---:|---:|---:|---:|---:|
| 128 | 142 | 149 | 140 | −2 |
| 256 | 131 | 138 | 130 | −1 |
| 512 | 125 | 132 | 125 | 0 |

### Baseline — B = 64

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=64`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **164 / 164 / 165 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

### Snap — B = 64

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=64`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **165 / 165 / 166 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +1 ms

### Guard — B = 64

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=64`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **159 / 159 / 159 ms**
- **SAU_config_avg_cycles:** **87**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **−5 ms** (largest Guard-faster gap so far on the new 128 KiB×1 harness)

### Baseline — B = 1024

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=1024`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **123 / 123 / 123 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

### Snap — B = 1024

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=1024`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **130 / 130 / 130 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +7 ms

### Guard — B = 1024

- **Date:** 2026-10-04
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=1024`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **122 / 122 / 122 ms**
- **SAU_config_avg_cycles:** **100–101**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **−1 ms**

### Baseline — B = 2048

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=2048`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **121 / 121 / 121 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

### Snap — B = 2048

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=2048`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **128 / 128 / 128 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +7 ms

### Guard — B = 2048

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=2048`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **121 / 121 / 121 ms**
- **SAU_config_avg_cycles:** **115**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **0 ms**

### Baseline — B = 4096

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `B=4096`, `1 pass/round (=128KiB hashed; HMAC IRQ-off)`
- **Runtime mean / min / max:** **120 / 120 / 120 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)

### Snap — B = 4096

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `B=4096`, `1 pass/round (=128KiB hashed; memcpy IRQ-off)`
- **Runtime mean / min / max:** **128 / 128 / 128 ms**
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** +8 ms

### Guard — B = 4096

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `B=4096`, `1 pass/round (=128KiB hashed; SAU every block)`
- **Runtime mean / min / max:** **121 / 121 / 121 ms**
- **SAU_config_avg_cycles:** **142–144** (mean ≈ 143)
- **NS / FAR:** 0 / 0.0000 (expected: no NormalTask)
- **Delta vs Baseline:** **+1 ms** (first B where Guard is not faster than Baseline)

**Table I (128 KiB × 1, no NormalTask) — complete.**

| B | Baseline (ms) | Snap (ms) | Guard (ms) | Guard−Baseline | Snap−Baseline |
|---:|---:|---:|---:|---:|---:|
| 64 | 164 | 165 | 159 | −5 | +1 |
| 128 | 142 | 149 | 140 | −2 | +7 |
| 256 | 131 | 138 | 130 | −1 | +7 |
| 512 | 125 | 132 | 125 | 0 | +7 |
| 1024 | 123 | 130 | 122 | −1 | +7 |
| 2048 | 121 | 128 | 121 | 0 | +7 |
| 4096 | 120 | 128 | 121 | +1 | +8 |

---

## Summary table (fill as measured)

| B | Baseline (ms) | Snap (ms) | Guard (ms) | Guard SAU cycles |
|---:|---:|---:|---:|---:|
| 64 | **164** | **165** | **159** | **87** |
| 128 | **142** | **149** | **140** | **88** |
| 256 | **131** | **138** | **130** | **90** |
| 512 | **125** | **132** | **125** | **94** |
| 1024 | **123** | **130** | **122** | **101** |
| 2048 | **121** | **128** | **121** | **115** |
| 4096 | **120** | **128** | **121** | **143** |

---

## FAR — with NormalTask

Config:
- `|M|=128 KiB` @ `0x08060000`, `ATTEST_PASSES=1`
- `NormalTask` enabled, `TARGET_FREQ_HZ=1000`
- 10 rounds per configuration
- Success check: banner says `FAR @1000 Hz` / `+ NormalTask`, and `NS` is not `0/...`

### Guard — B = 4096 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Banner check:** `FAR: Guard + NormalTask 1000 Hz`, `FAR @1000 Hz`
- **Runtime mean:** **122 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 122/122 every round
- **SAU_config_avg_cycles:** 142–144

```text
SMARM-Guard B=4096 |M|=128KiB @0x08060000, 1 pass/round, FAR @1000 Hz
Round 1–10: Runtime=122 ms, NS=122/122, FAR=1.0000
=== SUMMARY === Mean: 122 ms | FAR mean=1.0000
```

### Baseline — B = 4096 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Banner check:** `FAR: Baseline + NormalTask 1000 Hz`
- **Runtime mean:** **121 ms**
- **FAR mean / min / max:** **0.2735 / 0.2727 / 0.2810**
- **NS:** typically 33/121 (one round 34/121)
- **Note:** Matches paper ~0.27 at B=4096 (t_disabled ≈ 3.75 ms → ~1/3.75 of ticks survive)

```text
SMARM Baseline B=4096 ... FAR @1000 Hz
Round 1–10: Runtime=121 ms, NS=33–34/121, FAR≈0.27
=== SUMMARY === Mean: 121 ms | FAR mean=0.2735
```

### Snap — B = 4096 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Banner check:** `FAR: Snap + NormalTask 1000 Hz`
- **Runtime mean:** **130 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 130/130 every round

```text
SMARM-Snap B=4096 ... FAR @1000 Hz
Round 1–10: Runtime=130 ms, NS=130/130, FAR=1.0000
=== SUMMARY === Mean: 130 ms | FAR mean=1.0000
```

**B=4096 @ 1000 Hz — three modes complete (reviewer Q4 key point):**

| Mode | Runtime | FAR mean |
|---|---:|---:|
| Baseline | 121 ms | **0.2735** |
| Snap | 130 ms | **1.0000** |
| Guard | 122 ms | **1.0000** |

### Baseline — B = 2048 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **122 ms**
- **FAR mean / min / max:** **0.5401 / 0.5328 / 0.5574**
- **NS:** 65–68 / 122
- **Note:** Matches paper ~0.55 at B=2048

### Baseline — B = 1024 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **124 ms** (min 124, max 125)
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 124–125 / 124–125
- **Note:** Old paper listed ≈0.98 here; new 128 KiB×1 harness gives 1.0000 (t_disabled ≈ 0.94 ms < 1 ms tick)

### Baseline — B = 512 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **127 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 127/127 every round

### Baseline — B = 256 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **133 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 133/133 every round

### Baseline — B = 128 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **144 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 144/144 every round

### Baseline — B = 64 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=0`
- **Runtime mean:** **167 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 167/167 every round

### Baseline FAR @1000 Hz — complete

| B | Runtime (ms) | FAR mean |
|---:|---:|---:|
| 64 | 167 | **1.0000** |
| 128 | 144 | **1.0000** |
| 256 | 133 | **1.0000** |
| 512 | 127 | **1.0000** |
| 1024 | 124 | **1.0000** |
| 2048 | 122 | **0.5401** |
| 4096 | 121 | **0.2735** |

### Snap — B = 64 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **168 ms** (min 168, max 169)
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 168–169 / matching expected

### Snap — B = 128 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **152 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 152/152 every round

### Snap — B = 256 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **140 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 140/140 every round

### Snap — B = 512 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **135 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 135/135 every round

### Snap — B = 1024 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **132 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 132/132 every round

### Snap — B = 2048 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=0`, `USE_SNAP=1`
- **Runtime mean:** **130 ms** (min 130, max 131)
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 130–131 / matching expected

### Snap FAR @1000 Hz — complete

| B | Runtime (ms) | FAR mean |
|---:|---:|---:|
| 64 | 168 | **1.0000** |
| 128 | 152 | **1.0000** |
| 256 | 140 | **1.0000** |
| 512 | 135 | **1.0000** |
| 1024 | 132 | **1.0000** |
| 2048 | 130 | **1.0000** |
| 4096 | 130 | **1.0000** |

### Guard — B = 64 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **162 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 162/162 every round
- **SAU_config_avg_cycles:** **87**

### Guard — B = 128 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **142 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 142/142 every round
- **SAU_config_avg_cycles:** **88**

### Guard — B = 256 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **132 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 132/132 every round
- **SAU_config_avg_cycles:** **90**

### Guard — B = 512 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **127 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 127/127 every round
- **SAU_config_avg_cycles:** **94**

### Guard — B = 1024 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **124 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 124/124 every round
- **SAU_config_avg_cycles:** **100–101**

### Guard — B = 2048 @ 1000 Hz

- **Date:** 2026-10-05
- **Mode:** `USE_SAU_APPROACH=1`, `USE_SNAP=0`
- **Runtime mean:** **123 ms**
- **FAR mean / min / max:** **1.0000 / 1.0000 / 1.0000**
- **NS:** 123/123 every round
- **SAU_config_avg_cycles:** **115**

### Guard FAR @1000 Hz — complete

| B | Runtime (ms) | FAR mean | SAU cycles |
|---:|---:|---:|---:|
| 64 | 162 | **1.0000** | 87 |
| 128 | 142 | **1.0000** | 88 |
| 256 | 132 | **1.0000** | 90 |
| 512 | 127 | **1.0000** | 94 |
| 1024 | 124 | **1.0000** | ~101 |
| 2048 | 123 | **1.0000** | 115 |
| 4096 | 122 | **1.0000** | ~143 |

### FAR @1000 Hz — all modes complete

| B | Baseline | Snap | Guard |
|---:|---:|---:|---:|
| 64 | 1.0000 | 1.0000 | 1.0000 |
| 128 | 1.0000 | 1.0000 | 1.0000 |
| 256 | 1.0000 | 1.0000 | 1.0000 |
| 512 | 1.0000 | 1.0000 | 1.0000 |
| 1024 | 1.0000 | 1.0000 | 1.0000 |
| 2048 | **0.5401** | 1.0000 | 1.0000 |
| 4096 | **0.2735** | 1.0000 | 1.0000 |

---

## t_disabled

- **Status:** instrumented on branch `snapguard-paper-reviewer` (DWT CYCCNT in Secure)
- **Method:** bracket each `__disable_irq`…`__enable_irq` window; report avg + max cycles (+ ms @ 110 MHz)
- **Config switch:** `Secure_nsclib/attest_overhead_cfg.h` only (`USE_SAU_APPROACH` / `USE_SNAP` / `ATTEST_BLOCK_SIZE`)
- **Sweep order:** Guard all B → Snap all B → Baseline all B (B = 64,128,256,512,1024,2048,4096)
- **Board sweep:** in progress

### Guard — B = 2048 (t_disabled)

- **Date:** 2026-10-05
- **Runtime mean:** 123 ms, FAR=1.0000
- **SAU_config_avg_cycles:** 114–115  (= lock+unlock per block)
- **t_disabled avg / max:** **57 / 119 cy** ≈ **0.52 / 1.08 µs**
- **Note:** avg = mean of each lock/unlock window; max ≈ one SAU reconfig. Old fig ~1.04 µs matches SAU≈115 cy. Print was coarse (0.000 ms); fixed to µs next build.

### Guard — B = 64 (t_disabled)

- **Runtime mean:** 162 ms, FAR=1.0000
- **SAU_config_avg_cycles:** 87
- **t_disabled avg / max:** **43 / 70 cy** (round1 max 79) ≈ **0.390 / 0.636 µs**

### Guard t_disabled table

| B | avg cy | max cy | avg µs | max µs | SAU (L+U)/blk |
|---:|---:|---:|---:|---:|---:|
| 64 | 43 | 70 | 0.390 | 0.636 | 87 |
| 128 | 44 | 80 | 0.400 | 0.727 | 88 |
| 256 | 45 | 80 | 0.409 | 0.727 | 90 |
| 512 | 46 | 80 | 0.418 | 0.727 | 93 |
| 1024 | 50 | 80 | 0.454 | 0.727 | 100 |
| 2048 | 57 | 119 | 0.52 | 1.08 | 115 |
| 4096 | 71 | 119 | 0.645 | 1.081 | 143 |

### Guard t_disabled — complete

All B remain ≪ 1 ms (max ≈ 1.08 µs). Matches claim: SAU lock/unlock only.

### Snap — B = 64 (t_disabled)

- **Runtime mean:** 165 ms, FAR=1.0000
- **t_disabled avg / max:** **126 / 303 cy** ≈ **1.145 / 2.754 µs**
- **vs old Snap fig:** B=64 was `<0.007 ms` (=`<7 µs`) — new avg 1.1 µs is inside that bound

### Snap t_disabled — in progress (next: B=128)

| B | avg cy | max cy | avg µs | max µs |
|---:|---:|---:|---:|---:|
| 64 | 126 | 303 | 1.145 | 2.754 |
| 128 | 851 | 1607 | 7.736 | 14.609 |
| 256 | 1687 | 2468 | 15.336 | 22.436 |
| 512 | 3360 | 4120 | 30.545 | 37.454 |
| 1024 | 6703 | 7375 | 60.936 | 67.045 |
| 2048 | 13391 | 13974 | 121.736 | 127.036 |
| 4096 | 26766 | 27162 | 243.327 | 246.927 |

### Snap t_disabled — complete (≈±5% vs old Table)

| B | new avg µs | old paper |
|---:|---:|---:|
| 64 | 1.145 | &lt;7 µs |
| 128 | 7.736 | 7 µs |
| 256 | 15.336 | 15 µs |
| 512 | 30.545 | 29 µs |
| 1024 | 60.936 | 58 µs |
| 2048 | 121.736 | 116 µs |
| 4096 | 243.327 | 240 µs |

### Baseline t_disabled — complete

| B | avg cy | max cy | avg ms | max ms |
|---:|---:|---:|---:|---:|
| 64 | 6648 | 7345 | 0.060 | 0.067 |
| 128 | 13062 | 13786 | 0.119 | 0.125 |
| 256 | 25889 | 26682 | 0.235 | 0.243 |
| 512 | 51544 | 52335 | 0.469 | 0.476 |
| 1024 | 102858 | 103505 | 0.935 | 0.941 |
| 2048 | 205399 | 205852 | 1.867 | 1.871 |
| 4096 | 410471 | 411279 | 3.731 | 3.738 |

### Baseline vs old paper + FAR tick model

| B | new avg ms | old ms | FAR@1000 | 1ms/t_dis |
|---:|---:|---:|---:|---:|
| 64 | 0.060 | 0.062 | 1.000 | — |
| 128 | 0.119 | 0.120 | 1.000 | — |
| 256 | 0.235 | 0.237 | 1.000 | — |
| 512 | 0.469 | 0.472 | 1.000 | — |
| 1024 | 0.935 | 0.939 | 1.000 | — |
| 2048 | 1.867 | 1.877 | **0.542** | 0.536 |
| 4096 | 3.731 | 3.749 | **0.277** | 0.268 |

### t_disabled summary (all modes, avg)

| B | Baseline | Snap | Guard |
|---:|---:|---:|---:|
| 64 | 0.060 ms | 1.15 µs | 0.39 µs |
| 128 | 0.119 ms | 7.74 µs | 0.40 µs |
| 256 | 0.235 ms | 15.3 µs | 0.41 µs |
| 512 | 0.469 ms | 30.5 µs | 0.42 µs |
| 1024 | 0.935 ms | 60.9 µs | 0.45 µs |
| 2048 | 1.867 ms | 122 µs | 0.52 µs |
| 4096 | 3.731 ms | 243 µs | 0.65 µs |

**Status: t_disabled sweep DONE** (branch `snapguard-paper-reviewer`)

---

## t_disabled mean/min/max/std re-check (oak / plot error bars)

Full log: [`tdis_mean_std_log.md`](tdis_mean_std_log.md)

- **Baseline B=128 (2026-10-14):** mean 13086 cy → **0.118963 ms** ≈ table **0.119 ms** (same); std ≈ 1.51 µs; max 126.5 µs. Keep table mean.
- **Full mean/min/max/std sweep DONE 2026-10-14:** Baseline + Snap + Guard, all B∈{128…4096}. See `tdis_mean_std_log.md` (plot-ready table at end).

---

## Guard 256 KiB fault demo (reviewer Q3)

- **Status:** DONE 2026-10-06 (debug screenshot kept by author)
- **UART:** stops after `Calling SECURE_ShuffledHMAC_secure (round 1)...` (no return)
- **Debugger:** halted in `HardFault_Handler` → `while (1)` (`Secure/Src/stm32l5xx_it.c`). May be SecureFault escalated to HardFault; either way attestation does not complete.
- **Config:** `ATTEST_FAULT_DEMO=1`, Guard, B=1024, `|M|=256 KiB @ 0x08040000`
- **Claim supported:** Guard cannot safely cover live NS XIP; paper window remains 128 KiB @ `0x08060000`.
- **Not a production recovery path.**
- **Next:** set `ATTEST_FAULT_DEMO` back to `0`.
