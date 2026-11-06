# Gate A Result — Digest vs Golden (B1024)

**PASS** with patterned \|M\| (rejected all-0xFF as mode-collision).

Challenge: `0123456789abcdeffedcba9876543210`

| Mode | host == on-target digest |
|------|--------------------------|
| original | `782422b4b7ae3468b415b689d9dbb46bf0b92451beff2c11ff038245a22de0dc` |
| prng | `f5150b6a80211fb887bc1779235a75b435d8c35639a8257608ad18eaf694c17c` |
| fpe | `3244441f1c6eee9039c47c753d846c34ac94d46b12205ac31ff5d55e3a1a2f65` |

Evidence: `*_COMPARE.txt`, `*_uart_patterned.txt` / `original_B1024_uart.txt`, `patterned_M_128KiB.bin`, probe builds under `builds/digest_probe_*_B1024/`.

**FAR closure addendum (2026-10-23):** Original B1024 was the missing Gate-A cell; host==on-target after writing `patterned_M_128KiB.bin` @ `0x08060000`. Digests of all three modes are **distinct** on patterned \|M\|.
