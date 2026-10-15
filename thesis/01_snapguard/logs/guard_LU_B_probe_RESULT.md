# Guard L vs U probe — does t_disabled grow with B?

Date: 2026-10-14  
Firmware: `TDIS_RAW_DUMP=1`, Guard, 10 rounds, DWT @ 110 MHz  
Logs: `guard_B128_raw.txt`, `guard_B4096_raw.txt`  
Parser: `parse_tdis_lu_compare.py` / `parse_tdis_raw.py`

## Result (pooled over 10 rounds)

| | B=128 | B=4096 | Δ (µs) |
|--|------:|-------:|-------:|
| LOCK mean (L) | 0.432 | 0.584 | **+0.152** |
| UNLOCK mean (U) | 0.375 | 0.723 | **+0.348** |
| ALL windows mean | 0.404 | 0.654 | +0.250 |
| L std | 0.037 | 0.141 | |
| U std | 0.062 | 0.359 | |
| max observed | 0.827 (L) | **1.082 (U)** | |

Sample counts: B=128 → 10240 L + 10240 U; B=4096 → 320 L + 320 U.

## Conclusion

1. SAU path is still O(1) register work — not hash-sized IRQ-off work.
2. Mean still rises mildly with B (matches paper table 0.40 → 0.65 µs).
3. **Unlock grows ~2.3× more than lock** → supports the post-hash microarch hypothesis
   (U is measured immediately after `hmac_sha256_update` of size B).
4. At B=4096, U has the heavy tail (std 0.36 µs, max 1.08 µs); L max stays 0.80 µs.
5. At B=128, U is actually *faster* than L on average — growth with B is not uniform.

## Oral one-liner

> Lock and unlock are the same SAU sequence in code, but measured unlock latency rises more with B because it runs right after a B-byte hash; the floor stays ~0.36 µs and we never disable IRQs for the hash itself.
