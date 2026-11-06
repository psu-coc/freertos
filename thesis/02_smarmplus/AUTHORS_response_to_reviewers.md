# Author’s Response to Reviewers — IEEE Access (SMARM+)

**How to use this file when submitting on ScholarOne / IEEE Author Portal**

Typical Access resubmission package:
1. **Revised manuscript** (clean PDF)
2. **Highlighted / tracked-changes PDF** (mark what you changed)
3. **Author’s Response to Reviewers** (this document — one file covering all reviewers)

Format reviewers expect (per comment):
- Quote or briefly restate the concern
- Say Agree / Partially agree / Clarify
- Explain what you changed (or why you did not change the protocol)
- Point to the revised location (Section / Algorithm / page)

Tone: polite, specific, no defensiveness. Prefer “We agree … We have clarified …” over long debates when the fix is documentation.

---

## Response to Reviewer 1

### R1.1 — Challenge binding into attestation evidence

**Reviewer’s comment**  
Algorithm 1 does not clearly bind the challenge into the final attestation evidence. The algorithm derives the permutation seed from Hash(chal, k), but the final evidence is essentially the HMAC accumulated over the shuffled memory blocks. The challenge affects the order of the blocks, but apparently is not explicitly MACed into the final transcript. This may still produce challenge-dependent evidence because the concatenation order changes.

**Our response**  
We agree that Algorithm 1 does not include an explicit `HMAC.Update(chal)` step, and that this was not stated clearly enough in the submitted manuscript.

However, the challenge is already cryptographically bound into the attestation evidence through the **measurement order**. Because  
`σ = FisherYates(Hash(chal, k), …)`  
and the HMAC is accumulated over memory blocks in the order `σ(0), …, σ(n−1)`, a different challenge yields a different concatenation order and thus a different tag (except with negligible collision probability). As described in Section II-A, the verifier, given `k` and `chal`, reconstructs `σ` and recomputes the expected evidence; verification therefore fails for a stale or substituted challenge. This is the same challenge-binding mechanism as in the original SMARM design. The reviewer’s observation that challenge-dependent evidence can still arise “because the concatenation order changes” is exactly this intended binding.

Nevertheless, to make the binding explicit and independent of this argument, and in line with Reviewer 3 (comment 5), the revised protocol now also authenticates `chal` directly. It is placed in a fixed-length header that the HMAC absorbs before the first memory block.

**Changes in the revised manuscript**  
1. **Section II-A (Finalization):** added an explicit statement of order-based challenge binding in SMARM, and noted that our realization additionally authenticates `chal` explicitly.
2. **Section III, new paragraph “Authenticated attestation context”:** the evidence is now `HMAC-SHA256(k_mac, hdr ‖ M_σ(0) ‖ … ‖ M_σ(n−1))` with `hdr = sid ‖ B ‖ n ‖ base ‖ |M| ‖ T ‖ chal`.
3. **Algorithms 1–3:** `HMAC.Update(hmac, hdr)` follows `HMAC.Init`. The caption of Algorithm 1 states that `chal` is bound both explicitly and through `σ`.

The header adds at most one SHA-256 compression per attestation, which is negligible relative to the reported runtimes.

---

### R1.2 — Between-block transition / continuation invariant

**Reviewer’s comment**  
The transition between block measurements needs much more careful and detailed analysis. Interrupts are disabled only during a block measurement and are then re-enabled, creating an adversarial interval between blocks. The crucial invariant should be precisely stated: before returning control to the Non-Secure world, all state required for the next secure measurement—HMAC state, loop counter, permutation state/bitmap/FPE state—must reside exclusively in Secure memory and must resume at exactly the authenticated continuation point. The paper argues this informally through TrustZone-M entry integrity, but for this protocol this is not an implementation detail; it is fundamental to correctness.

**Our response**  
We agree. The between-block interval is security-critical, and the required continuation invariant should be stated explicitly. Our design already places all attestation continuation state in Secure memory and relies on TrustZone-M to prevent Non-Secure code from reading/writing that state or redirecting Secure control flow. In particular, after each block measurement interrupts may be re-enabled, but: (i) the HMAC context, loop index, and order-related state (`σ`, or the PRNG bitmap / FPE state in SMARM+) remain in Secure RAM; (ii) Non-Secure software cannot access these locations; and (iii) upon resumption, Secure execution continues at the interrupted point (or re-enters only through the designated NSC entry), and cannot be redirected to an arbitrary Secure-world location. We also keep interrupts disabled during each individual block measurement, so a block cannot be partially measured under Non-Secure interference.

The submitted manuscript discussed these points mainly through (S1) and the TrustZone-M architecture, without naming a single continuation invariant that enumerates the protected state. We therefore treat this as a clarity/analysis gap rather than a protocol redesign.

**Changes in the revised manuscript**  
1. Section III (SMARM on RTOS using TrustZone-M): added a paragraph **Between-block continuation invariant** that enumerates the protected Secure state and the resume requirement.  
2. SMARM+PRNG and SMARM+FPE security subsections: each now states which order-related Secure state the invariant covers for that variant.  
3. Appendix, Theorem A.1 (S1): explicitly links the formal argument to this continuation invariant.

---

### R1.3 — Timing leakage via FindKthZero

**Reviewer’s comment**  
SMARM+PRNG has a potentially serious timing-leakage problem. If the scan terminates when the r-th zero is found, then its execution time is potentially correlated with the chosen block index and current bitmap state. Because the attacker controls the Non-Secure world and can observe interrupt/resumption timing, this could reveal partial information about the supposedly private measurement order. This directly conflicts with the key protocol requirement S3.

**Our response**  
We agree that an early-exit bitmap scan would be a timing side channel against S3. Our SMARM+PRNG implementation does **not** use early exit. `FindKthZero` always traverses all *n* entries of the `used` bitmap: after the *r*-th zero is identified, the remaining entries are still scanned so that the procedure’s duration is independent of the selected index `idx` (and thus does not reveal which remaining block was chosen). The wall-clock cost may still depend on *n* (public from the attested size and block size *B*), but not on the secret order.

The submitted manuscript described the scan functionally (“locate the *r*-th zero”) and its asymptotic cost as “up to O(n)”, which left room for the early-exit interpretation assumed in the review. We therefore treat this as a **documentation gap**, not as an unprotected timing leak in the implemented design.

**Changes in the revised manuscript**  
1. Algorithm 2 (`FindKthZero` line): added comment that the routine always scans all *n* bits (no early exit).  
2. Secure-storage vs.\ runtime trade-off: replaced “up to O(n)” with an explicit full-pass / Θ(n) statement and independence from `idx`.  
3. SMARM+PRNG security (S3): restored and clarified the timing side-channel mitigation paragraph (always-scan).

---

### R1.4 — Domain separation of keys

**Reviewer’s comment**  
There is no explicit domain separation in key derivation. The same master key k appears to be involved in seed derivation, HMAC evidence, and—in SMARM+FPE—the derivation of the FFX key/tweak. Please elaborate.

**Our response**  
We agree. In the submitted manuscript a single pre-shared master secret `k` was used both to derive the measurement order and to compute the attestation HMAC (and, for SMARM+FPE, to derive FFX material), without labeled domain separation. Although these uses already went through different algorithms (`Hash`/`PRNG`/`KDF` vs `HMAC`), that is not a substitute for explicit purpose separation from a master key.

**Changes in the revised manuscript**  
1. Section III: added a **Key hierarchy and domain separation** paragraph defining  
   `k_prp ← KDF(0 ‖ k)` and `k_mac ← KDF(1 ‖ k)`,  
   with `KDF(ℓ ‖ k) := HMAC-SHA256(k, ℓ)`, citing Krawczyk’s HKDF work (CRYPTO 2010) and NIST SP 800-108 for labeled/context-based key derivation.  
2. Algorithms 1–3: updated to derive and use `k_prp` / `k_mac` (FFX keys are further derived from `k_prp` and `chal`).  
3. Supporting prose for SMARM+PRNG / SMARM+FPE updated accordingly.

The formal Appendix proofs still refer to the master-key notation in places; we will align those games with the derived-key hierarchy in coordination with the appendix revision (Reviewer 1 comment 5 / advisor).

---

*(R1.5 is assigned to the advisor’s Appendix rewrite.)*

---

### R1.6 — Real-time impact beyond FAR

**Reviewer’s comment**  
FAR isolates the interference produced by atomic measurements, which is useful, but it does not measure the total real-time impact of running SMARM. CPU time consumed by permutation generation, PRNG scanning, FPE, hashing, and wrapper/service execution can still affect task scheduling.

**Our response**  
We agree that FAR is intentionally narrow and must not be read as a complete measure of real-time impact. The revision states what FAR does and does not capture, separates the two kinds of attestation cost, and adds an analytical bound for arbitrary task sets.

- **Two kinds of cost.** In all three schemes, interrupts are masked only while one block is hashed. Order generation (Fisher–Yates, `FindKthZero`, `FFX.Enc`), HMAC setup and finalization, and the Wrapper Task run with interrupts enabled at the lowest application priority.
  - The non-preemptible part delays higher-priority tasks. FAR measures this part.
  - The preemptible part consumes only the processor time that higher-priority tasks leave over, apart from context-switch overhead. It does, however, delay any task whose priority is not above the Wrapper Task, and it lengthens attestation under load. The end-to-end runtime and energy tables measure this total work.
- **Why the variants leave FAR unchanged.** The variants change only preemptible work, so the blocking seen by higher-priority tasks is the same for a given `B`. This matches the measured FAR.

**Changes in the revised manuscript**  
1. **Section IV-C, expanded Remark on FAR.** The Remark lists the preemptible attestation work and whom it affects, points to runtime/energy for total cost, and states that FAR does not report response times, jitter, or deadline misses.
2. **Section IV, new subsection “Worst-Case Blocking for Schedulability Analysis”.** It models attestation as a lowest-priority task with non-preemptible sections of length `t_disabled(B)`.
   - Under preemptive fixed-priority scheduling, each higher-priority task gets one blocking term of `t_disabled(B)` in the standard response-time recurrence (Joseph–Pandya; Audsley et al.; Buttazzo). This has the same form as blocking by a lower-priority non-preemptive section (Sha et al.).
   - The term depends on `B` and the platform, not on the scheme (e.g., 0.94 ms at `B = 1024` on FreeRTOS). It also bounds the added release jitter.
   - The subsection also states what the analysis does not cover: the tick loss when `t_disabled` exceeds one tick, and tasks at or below the Wrapper Task's priority. The completion time of attestation under load can be bounded by the same recurrence and lengthens the window of exposure.
3. **Section VI, new “Limitations” subsection.** It covers the metric, the synthetic single-task workload, the single board and 1 kHz tick, FreeRTOS-only variant evaluation, the different Secure runtimes of the two RTOS setups (bare-metal Secure image vs. TF-M), and attestation completion under load, which is not measured.
4. **Claims narrowed.** The Abstract and Contributions now speak of interference as measured by FAR instead of “real-time compatibility”. `B = 1024` is now described as the largest block size with FAR ≈ 1.0 at our 1 kHz tick, not as a generally recommended operating point.

---

## Response to Reviewer 2

### R2.1 — DOIs in the bibliography

**Reviewer’s comment**  
For each entry in the bibliography, please provide its DOI (when available).

**Our response**  
We agree and thank the reviewer. We checked every bibliography entry against Crossref and the publishers’ records.

**Changes in the revised manuscript**  
1. A DOI is now printed for every entry that has one, including the NIST publications and the arXiv preprint.  
2. While checking, we also corrected metadata errors in several entries (each verified against at least two sources: Crossref plus OpenAlex, the publisher's page, or the original PDF):
   - PRoM: corrected venue to *IEEE Systems Journal*, vol. 16, no. 1, pp. 789–800, 2022.
   - HAtt: corrected the DOI and the complete author list.
   - Sancus 2.0: corrected the DOI and author first names.
   - ATRIUM: corrected the author list and pages.
   - Kuang et al. (RA survey, *Computers & Security*): corrected the author list.
   - Luo et al. (TrustZone-M security, *IEEE IoT-J*): corrected author order and pages.
   - TZmCFI: corrected issue number, pages and author first names.
   - ASAP: added the missing co-author and corrected pages.
   - SIMON/SPECK (DAC 2015): corrected the title and an author's first name.
   - TOCTOU (CCS 2021), PISTIS (USENIX Security 2022), ITERATOR (*IEEE IoT-J*): corrected page ranges.
   - PEARTS: updated from “to appear” to the published IEEE S&P 2025 version.
3. Some entries have no DOI: papers published by USENIX and NDSS 2012, which do not assign DOIs; the 1938 Fisher–Yates tables; the FFX NIST submission; and vendor or web documentation. These keep their venue information or a stable URL with the access date.

---

### R2.3 — Attested memory size and window-of-exposure

**Reviewer’s comment**  
When presenting the results about the total computation time required for complete attestation (i.e., all memory blocks), please specify the total memory size attested and comment that the longer the attestation time, the wider the time for which an attack goes undetected (i.e., wider window-of-exposure).

**Our response**  
We agree. End-to-end attestation runtime should be reported together with the attested memory size, and longer attestation latency implies a longer window during which compromise may remain undetected until the round completes.

**Changes in the revised manuscript**  
1. **Section IV (Implementation), new paragraph “Attested memory region”.**
   - All runtime and energy results cover the flash address range `0x08000000–0x0807FFFF`, i.e., `|M| = 512 KiB`, in one shuffled traversal per attestation round, with `n = |M|/B`.
   - The paragraph also identifies the 256 KiB Non-Secure image as the attestation target and explains why the reported costs are upper bounds for attesting that image alone (see our response to Reviewer 3, comment 12).
2. **Table I caption.** It now states the attested size and address range: “one shuffled traversal over `|M| = 512 KiB`”.
3. **Section VI (Overall Attestation Runtime).**
   - We restate `|M|` and note that changing `B` changes `n` and the per-block work, but not the number of bytes attested per round.
   - A new paragraph, “Window of exposure”, explains that the prover may remain compromised and undetected until a round completes, so longer attestation widens this window. Concrete examples: 490 ms (SMARM) and 726 ms (SMARM+FPE) at `B = 1024` bytes, versus 9.3 s for SMARM+PRNG at `B = 64` bytes.
4. **Storage discussion.** It now states explicitly that storage is determined by `n = |M|/B` with `|M| = 512 KiB`.

The runtime values themselves are unchanged. This comment required clarification only, not new measurements.

---

## Response to Reviewer 3

*(Numbering follows the reviewer’s original comment order. Items not yet written are pending.)*

*(R3.1 and R3.2 are assigned to the advisor’s Appendix rewrite, together with Reviewer 1, comment 5. The timing behavior that R3.2 asks to include in the proofs is specified in Section V-A, “Timing behavior of order generation”; see comment 3.)*

### R3.3 — Constant-time FindKthZero and bounded sampling

**Reviewer’s comment**  
Either implement FindKthZero and bounded random sampling in constant time, or demonstrate experimentally or analytically that timing reveals no useful information.

**Our response**  
We now do both. The design specifies constant-time forms of every step that runs between two block measurements. We also explain analytically why the evaluated prototype’s timing depends only on public quantities, and we are explicit about what that argument assumes.

**Changes in the revised manuscript**  
1. **Section V-A, new paragraph “Timing behavior of order generation”.**
   - **Requirement:** the Secure run time between two blocks may depend only on `n` and `rem`, both of which KFV malware knows by assumption, and on events that are independent of the selected block.
   - **FindKthZero** is branch-free:
     - for every `i`, `z = 1 − used[i]`; the match flag `e = [c = r−1]` is computed arithmetically; then `idx |= −(z·e) & i` and `c += z`;
     - every `i` executes the same instruction sequence over a bitmap in uncached Secure SRAM, so the scan time is a function of `n` only.
   - **RandInt:**
     - Rejection happens with probability below 2⁻¹⁹. Since draws are independent and the accepted output is uniform, the rejection count is independent of the output.
     - The reduction uses Lemire’s multiply-high method (ACM TOMACS 2019), so there is no operand-dependent division on the secret word. The only division, `2³² mod m`, involves the public `m`.
   - **Ciphers:**
     - SHA-256/HMAC and Speck (add–rotate–xor) have no secret-indexed memory accesses.
     - The AES tables used by the PRNG are placed in Secure SRAM, because flash reads may be served by the instruction cache.
   - **SMARM+FPE** is less exposed: FFX over Speck has no secret-dependent branches or memory accesses, and cycle walking never occurs for power-of-two `n`.
2. **The evaluated prototype, stated honestly.**
   - It used a full-pass FindKthZero with data-dependent branches, `UDIV`, and AES tables in flash.
   - Its branch outcomes are fixed by the number of zeros (`rem`, public) plus one match. On a core whose branch cost depends only on the outcome, its scan time is therefore a function of `(n, rem)`. We state that we do not rely on this microarchitectural property.
   - The constant-time forms change only a few instructions per bitmap entry and per random word, so the reported runtimes remain representative.
3. **Section IV, “Cryptographic instantiation”:** the RandInt item now refers to the constant-time reduction.
4. **Section V-A (S3):** the argument now points to the full timing analysis.

---

### R3.4 — Specification of the cryptographic building blocks

**Reviewer’s comment**  
Define PRNG, RandInt, KDF, domain separation, FFX domain encoding, number of rounds, Speck parameters, and concrete security level.

**Our response**  
We agree that these details were missing. The revised manuscript now specifies every building block, so that the construction can be reimplemented from the paper alone.

**Changes in the revised manuscript**  
1. **Section IV (Implementation), new paragraph “Cryptographic instantiation”.**
   - **KDF and domain separation:** `KDF(ℓ‖k) = HMAC-SHA256(k, ℓ)` with one-byte labels `0x00` (order generation, `k_prp`) and `0x01` (attestation MAC, `k_mac`). The master key `k` is 256 bits, and no key serves two purposes (see also Reviewer 1, comment 4).
   - **Seed:** `Hash(chal, k_prp) = HMAC-SHA256(k_prp, chal)`, split into a 128-bit key and a 128-bit initial counter block.
   - **PRNG:** AES-128 in counter mode (128-bit big-endian counter), with the keystream consumed as 32-bit words.
   - **RandInt(m):** rejection sampling. Draw a 32-bit `r`, accept if `r < m·⌊(2³²−1)/m⌋`, and return `r mod m`.
     - The output is exactly uniform, and the rejection probability is below 2⁻¹⁹ for all evaluated `n`.
     - The number of rejections is independent of the returned value.
     - `RandInt(1, rem)` in Algorithm 2 is `1 + RandInt(rem)`.
   - **Fisher–Yates (SMARM):** the Durstenfeld variant using `RandInt`.
   - **Concrete security:**
     - Order generation uses 128-bit keys (AES-128, Speck128/128), i.e., a 128-bit security level against exhaustive key search.
     - The MAC uses a 256-bit key and a 256-bit tag.
     - Beyond the inherent `1/rem` guessing probability, the adversary’s advantage in predicting the order is bounded by the PRF/PRP advantages of HMAC-SHA256 and AES-128 (resp. FFX over Speck128/128). The Appendix makes this reduction explicit.
2. **Section V-B (SMARM+FPE), new paragraph “FFX instantiation”.**
   - **Key and tweak:** the FFX key is the first 16 bytes of `HMAC-SHA256(k_prp, chal)`, so every attestation uses a fresh key. The 32-bit public tweak is the first four bytes of `chal`.
   - **Domain encoding:** `ℓ = ⌈log₂ n⌉`. The index is an ℓ-bit string, split into `A` (high ⌊ℓ/2⌋ bits) and `B` (low ⌈ℓ/2⌉ bits).
   - **Rounds:** 10 Feistel rounds with the radix-2 round structure of FF1. The round function takes the first 32 bits of a CBC-MAC over (parameters ‖ tweak ‖ round index ‖ `B`), added modulo `2^m`.
   - **Speck parameters:** Speck128/128, i.e., 128-bit block and key, 64-bit words, rotations (8, 3), 32 rounds.
   - **Cycle walking:** re-encryption under the same key and tweak when the output is ≥ `n`. This is never triggered in our evaluation, because `n` is a power of two.
   - **Standards and small domains:**
     - We explicitly do not claim SP 800-38G conformance, because Speck is not an approved cipher.
     - Our domains (2⁷–2¹³) meet the mandatory minimum of 100 but not the recommended one million.
     - We explain why the small-domain attacks behind that recommendation (Hoang–Tessaro–Trieu, CRYPTO 2018) do not apply: they require FPE ciphertexts, whereas FFX outputs (block indices) never leave the Secure world and each key is used for a single traversal.

---

### R3.5 — Authenticated protocol metadata and key separation

**Reviewer’s comment**  
Explicitly authenticate the challenge, scheme identifier, block size, block count, memory base, and memory length, and derive separate keys for permutation generation and attestation.

**Our response**  
We agree with both requests and have revised the protocol accordingly.

**Changes in the revised manuscript**  
1. **Authenticated metadata (Section III, new paragraph “Authenticated attestation context”; Algorithms 1–3).**
   - Before any memory block, the HMAC now absorbs a fixed-length 34-byte header `hdr = sid ‖ B ‖ n ‖ base ‖ |M| ‖ T ‖ chal`:
     - `sid`: one-byte scheme identifier (SMARM / SMARM+PRNG / SMARM+FPE);
     - `B`, `n`, `base`, `|M|`: 32-bit big-endian integers;
     - `T`: one-byte number of shuffled traversals (see comment 7);
     - `chal`: the 128-bit challenge.
   - The evidence is `HMAC-SHA256(k_mac, hdr ‖ M_σ(0) ‖ … ‖ M_σ(n−1))`, and the verifier recomputes it with the same header. Hence:
     - the challenge is authenticated explicitly, in addition to its binding through the measurement order;
     - evidence cannot be reinterpreted under a different scheme, block size, block count, or memory layout;
     - the fixed-length encoding is unambiguous.
   - `sid`, `B`, `base`, `|M|`, and `T` are compile-time constants of the Secure image, not NSC arguments. The Non-Secure caller therefore cannot change the attested region or the number of traversals; it supplies only `chal` and the output buffer.
   - The header costs at most one extra SHA-256 compression per attestation.
2. **Key separation (Section III, “Key hierarchy and domain separation”; Section IV, “Cryptographic instantiation”; Algorithms 1–3).**
   - `k_prp = HMAC-SHA256(k, 0x00)` is used only for order generation (seed, PRNG, FFX key).
   - `k_mac = HMAC-SHA256(k, 0x01)` is used only for the attestation MAC.
   - See also Reviewer 1, comment 4, and our response to comment 4 above.

---

### R3.6 — Platform security configuration

**Reviewer’s comment**  
Explain how the implementation masks Secure and Non-Secure exceptions, handles NMI and faults, prevents DMA and other bus masters from changing measured memory, establishes secure boot, provisions the key, validates NSC arguments, and fixes the attested memory boundaries.

**Our response**  
We agree that these assumptions must be explicit. Each of the seven points is now addressed in a new subsection, and the threat model states the platform assumptions they rely on.

**Changes in the revised manuscript**  
1. **Section III, System & Threat Model (new text).**
   - Physical side-channel analysis, fault injection, and invasive hardware attacks are stated as orthogonal and out of scope, following recent TrustZone-M work (Kiri et al., VeCoDI, 2026).
   - Timing observable by Non-Secure software through interrupts is explicitly *in* scope; it is handled in Section V-A (see Reviewer 1, comment 3).
   - We now state the platform assumptions: secure boot, a key provisioned into Secure-only flash, and a security DMA controller (the GTZC on STM32L552).
2. **Section III, new subsection “Platform Security Configuration”.** This subsection addresses the reviewer’s seven points in order:
   - **Exception masking:** `CPSID i` executed in Secure state sets `PRIMASK_S`. This masks every configurable-priority exception of *both* security states, and Non-Secure software cannot modify `PRIMASK_S` (Armv8-M ARM; ST PM0264). Only NMI and HardFault can preempt a block measurement.
   - **NMI and faults:**
     - `AIRCR.BFHFNMINS = 0`, so NMI, HardFault, and BusFault target the Secure state and are handled by Secure handlers.
     - Non-Secure software cannot pend an NMI, because `ICSR.PENDNMISET` is RAZ/WI from Non-Secure state (PM0264).
     - Faults are handled fail-closed: the measurement is never resumed and no evidence is produced, so the verifier rejects on timeout.
   - **DMA and bus masters:**
     - All attestation state (keys, σ/bitmap, PRNG/FFX state, HMAC context) lies in SRAM that the GTZC MPCBB marks as Secure. Non-Secure DMA therefore cannot read or modify it (ST AN5347; VeCoDI).
     - The attested region is Non-Secure flash, which changes only through the flash controller’s program/erase sequence.
     - In addition, the Secure Service checks before each block that no Non-Secure DMA channel is enabled, and aborts attestation otherwise.
   - **Secure boot:** a unique Secure boot entry via `SECBOOTADD0` + `BOOT_LOCK`, with RDP level 2 disabling debug (AN5347). We note that our measurements were taken at RDP level 0 for debugging, which does not change runtime.
   - **Key provisioning:** `k` is provisioned into Secure-watermarked flash during manufacturing (e.g., ST secure firmware installation) and is never exposed to the Non-Secure world. In the prototype, it is a Secure-image constant.
   - **NSC argument validation:**
     - The single entry point accepts only a 16-byte challenge and a 32-byte output buffer.
     - `cmse_check_address_range` (the `TT` instruction) checks that both buffers lie entirely in Non-Secure memory accessible to the caller, which prevents confused-deputy writes into Secure memory.
     - The challenge is copied into Secure memory before use, the output is written only after finalization, and re-entrant calls are rejected while an attestation is in progress.
   - **Attested-memory boundaries:**
     - `base`, `|M|`, and `B` are Secure compile-time constants authenticated in `hdr` (see comment 5).
     - Every block index is checked to satisfy `idx < n` before the address `base + idx·B` is formed.
3. **Section IV (Implementation).** We corrected the Secure SRAM size from 128 KB to 96 KB and state how the split is enforced: the flash secure watermark for flash and the GTZC MPCBB (`0x20000000–0x20017FFF` Secure) for SRAM.
4. **References.** We added Kiri et al. (VeCoDI, arXiv 2026, DOI 10.48550/arXiv.2606.07470), ST AN5347 (Rev. 7), and ST PM0264 (Rev. 8).

---

### R3.7 — Multiple shuffled traversals in the evaluation

**Reviewer’s comment**  
Evaluate the number of independent shuffled traversals required for the stated malware-detection probability, and include that complete configuration in the runtime and energy results.

**Our response**  
We agree that the original tables reported a single traversal while the background cited the multi-traversal guarantee. The revision specifies how `T` traversals are performed, derives the `T` needed for a given escape probability, and reports the complete configuration.

**Changes in the revised manuscript**  
1. **Section III, new paragraph “Multiple traversals”.**
   - Traversal `t` derives a fresh order from `seed_t = Hash(chal ‖ t, k_prp)`.
   - All `T` traversals are absorbed into one HMAC, which is finalized once.
   - `T` is a Secure compile-time constant and is authenticated in `hdr`, so the Non-Secure caller cannot lower it (see comment 5).
   - The order state is reinitialized in place for each traversal.
2. **Section VI, new subsection “Number of Traversals and Complete Configuration” with a new table.**
   - **Required `T`:** optimal KFV malware escapes one traversal with probability ≈ 1/e and `T` traversals with probability `p^T` (Carpent et al., SMARM). Both variants inherit this bound, because their order is pseudorandom (Appendix). With `p = 0.37`, `T = ⌈ln ε / ln 0.37⌉` gives `T = 5` for ε = 10⁻², `T = 7` for ε = 10⁻³ (the example we cite from SMARM), and `T = 14` for ε = 10⁻⁶.
   - **Complete configuration at B = 1024 (ε = 10⁻³, T = 7):**

     | | SMARM | SMARM+PRNG | SMARM+FPE |
     |---|---|---|---|
     | Runtime | 3.43 s | 3.68 s | 5.08 s |
     | Energy | 1.02 J | 1.07 J | 1.55 J |

     The table also gives T = 1, 3, 5, and 14.
   - **Why multiplying by `T` is valid:**
     - Each traversal repeats the same data-independent work, so cost is linear in `T`.
     - Our single-traversal measurements include per-round setup (key derivation, header, finalization) that is done only once for `T` traversals, so `T` × the measured value is an upper bound.
     - Relative overheads are unchanged (7% / 48% runtime, 5% / 52% energy). The same factor applies to every block size in the runtime and energy tables.
   - **What `T` does not change:**
     - the interrupt-disabled interval per block, and hence the worst-case blocking;
     - the FAR during attestation;
     - the Secure storage.
   - **What `T` does change:** the window of exposure, which becomes 3.4–5.1 s at T = 7 and B = 1024, and about 65 s for SMARM+PRNG at B = 64. We advise choosing `T` jointly with the attestation period.
3. **Captions.** The runtime and energy tables now state `T = 1` explicitly, and the Implementation section points to the new subsection.

*Note:* the numbers for T > 1 are derived from the T = 1 measurements. A direct measurement at T = 7 can be added if the reviewer considers it necessary.

---

### R3.8 — Classical real-time metrics

**Reviewer’s comment**  
Supplement or replace FAR with maximum and percentile response time, release jitter, interrupt latency, deadline-miss ratio, worst-case blocking, and response-time or schedulability analysis. FAR alone should not support broad real-time claims.

**Our response**  
We agree that FAR alone cannot support broad real-time claims. In this revision we (1) add the worst-case blocking and a response-time analysis, which follow analytically from the design; (2) explain what FAR measures on our platform; and (3) narrow the real-time claims and state the remaining gaps as limitations.

**Changes in the revised manuscript**  
1. **Worst-case blocking and response-time analysis (new subsection in Section IV).** Attestation adds one blocking term `t_disabled(B)` to the fixed-priority response-time recurrence of every higher-priority task, and at most `t_disabled(B)` of release jitter. See our response to Reviewer 1, comment 6.
2. **Mechanism behind FAR (Section IV-D).** On FreeRTOS, FAR drops once `t_disabled` exceeds the 1 ms tick, because the masked SysTick interrupts collapse into one pending tick. This predicts FAR ≈ 1 ms / `t_disabled` (0.53 and 0.27 at `B = 2048` and `4096`, measured 0.55 and 0.27). The `B ≤ 1024` threshold is therefore tied to the tick period, and FAR = 1.0 still allows a job to be released up to `t_disabled` late (see comment 9).
3. **Narrowed claims and limitations.** See Reviewer 1, comment 6, items 3–4.

*Not done in this revision:* measured response-time percentiles, interrupt latency, and deadline-miss ratios under multi-task workloads. The analytical bounds above cover worst-case blocking and jitter; measurements under a representative application are listed as a limitation.

---

### R3.9 — Specification of the real-time task model

**Reviewer’s comment**  
Specify each task’s WCET, period, deadline, priority, utilization, computation, release and catch-up semantics, phase, and experimental duration.

**Our response**  
We agree that the original manuscript gave only the task frequencies. All parameters below were already fixed in our experimental setup; the revision now reports them.

**Changes in the revised manuscript**  
1. **Section IV-C, new Table (“Real-time task and measurement parameters”) and an expanded setup paragraph.** The same parameters apply to FreeRTOS and Zephyr.
   - **Task set:** one periodic task `T_RT`; no other application task is active.
   - **Period / deadline:** 1, 10, 100 ms (1000, 100, 10 Hz), with an implicit deadline `D = P`.
   - **Computation / WCET / utilization:** the job body only increments a release counter. We chose a minimal body so that the task's own utilization is negligible and FAR isolates the interference caused by attestation. We did not measure a separate WCET for this body.
   - **Priority and scheduling:** `T_RT` has a higher priority than the Wrapper Task, under preemptive fixed-priority scheduling on a 110 MHz core.
   - **Release and catch-up semantics:** jobs are released by an absolute periodic delay (`osDelayUntil`) on the 1 kHz kernel tick. If a release time has already passed in kernel time, the next job runs immediately; there is no other catch-up mechanism.
   - **Phase:** the first round starts 3 s after boot, and each round is followed by 2 s of idle time. We did not vary the release phase relative to the start of a round.
   - **Duration and repetitions:** 10 rounds per configuration, i.e., about 5–6.5 s of attestation per data point. Each round is timed with a 32-bit hardware timer (TIM2, 137.5 kHz, 7.27 µs resolution) that keeps counting while interrupts are masked. The original text wrongly said that the CPU cycle counter was used; this has been corrected.
   - **Baseline:** with interrupts left enabled, `T_RT` ran at its nominal frequency (FAR = 1.0), so `f_baseline = f`.
   - **Precision at 10 Hz:** a round spans only about five periods, so the per-round FAR changes in steps of about 0.2; the text states this.
2. **Section IV-D, new paragraph explaining the FAR threshold.** While a block is measured, all SysTick interrupts collapse into one pending exception, so the FreeRTOS tick count advances by at most one tick per block.
   - For `t_disabled < 1 ms` (0.94 ms at `B = 1024`), no tick is lost and every job is released, at most `t_disabled` late.
   - For longer blocks, releases proceed at roughly `1 ms / t_disabled` of the nominal rate. This predicts 0.53 at `B = 2048` and 0.27 at `B = 4096`, against measured FAR of 0.55 and 0.27. It also explains why FAR hardly depends on the task frequency.
   - The paragraph also states two limits: the `B ≤ 1024` threshold is tied to the 1 kHz tick, and FAR = 1.0 still allows each job to be released up to `t_disabled` late. This complements our response to comment 8.

---

### R3.10 — Broader real-time experimental campaign

**Reviewer’s comment**  
Evaluate multiple task sets, at least one representative real-time application, different release phases, wall-clock attestation completion under preemption, and both SMARM+ variants on both RTOSes.

**Our response**  
We agree that our real-time evaluation is narrow. We used a single periodic task with a minimal body on purpose: it isolates the interference caused by the non-preemptible block measurement. It does not cover the configurations the reviewer lists. In this revision we (1) cover parts of the request analytically, (2) state the scope and its limits explicitly, and (3) narrow the claims to the evaluated configuration.

**Changes in the revised manuscript**  
1. **Multiple task sets (new subsection in Section IV, “Worst-Case Blocking for Schedulability Analysis”).** For any fixed-priority task set, attestation adds one blocking term `t_disabled(B)` to the response-time recurrence of every task above the Wrapper Task, and at most `t_disabled(B)` of release jitter. A task set can therefore be checked without a new experiment for each set. See our response to Reviewer 1, comment 6.
2. **Attestation completion under preemption (same subsection).** The completion time of attestation is bounded by the same recurrence, with the measured runtime (Table I) as its execution time and without the blocking term. The subsection notes that the longer completion time lengthens the window of exposure.
3. **Both variants.** Interrupts are masked only during the block measurement, which is identical in SMARM, SMARM+PRNG and SMARM+FPE, so the blocking term does not depend on the scheme. The variants differ only in work that runs with interrupts enabled.
4. **Both RTOSes (Section III and Section IV-D).** The revision now states that the Zephyr prototype runs the Secure Service as a TF-M secure partition, while the FreeRTOS prototype uses a bare-metal Secure image. Zephyr shows the same FAR threshold (`B ≤ 1024` bytes); the quantitative explanation of the threshold is given for FreeRTOS.
5. **New “Limitations” subsection (Section VI).** It lists what we did not evaluate: multi-task sets, representative applications and phase variations; one board and a 1 kHz tick; runtime, energy and footprint of SMARM+ on FreeRTOS only; and no measurement of attestation completion under load.
6. **Narrowed claims.** Abstract, contributions, Section VI and the conclusion now describe interference “as measured by FAR” in our configuration, and `B = 1024` bytes as the largest block size with FAR ≈ 1.0 at our 1 kHz tick, rather than a general operating point.

*Not done in this revision:* measurements with multiple task sets, a representative application, release-phase variations, attestation completion under load, and SMARM+ FAR on Zephyr.

---

### R3.11 — Measurement methodology

**Reviewer’s comment**  
Report repetitions, confidence intervals, power-meter accuracy, calibration, integration procedure, and idle-energy treatment.

**Our response**  
We agree. The revision now describes the full procedure, including its main source of error.

**Changes in the revised manuscript**  
1. **Section VI (Energy Consumption), expanded methodology paragraph.**
   - **Setup:** an FNIRSI FNB58 on the 5 V USB supply measures the whole board, including the on-board ST-LINK.
   - **Accuracy and calibration:** the manufacturer specifies a 16-bit ADC with ±(0.02% + 2 digits) for voltage and ±(0.05% + 2 digits) for current (new reference: FNB58 user manual). We rely on this specification and did not calibrate the meter against a reference instrument; the text states this.
   - **Integration procedure:** `V·I`, sampled at 100 Hz, is integrated between the `ATTEST_START` and `ATTEST_END` UART markers printed immediately before and after the NSC call.
   - **Dominant error:** marker alignment is accurate to one sample (10 ms) at each end. At about 0.29 W (5.02 V, 57–58 mA), this is at most about ±6 mJ per round, or 4% of the smallest reported value, which dominates the meter's own error.
   - **Idle energy:** we report gross board energy and do not subtract the idle draw. Board power is nearly constant across configurations, so energy is approximately proportional to runtime, and differences between schemes reflect runtime differences.
   - **Repetitions:** energy values are means over 10 rounds, with `T_RT` active at 1000 Hz. Runtimes are means over 10 rounds without `T_RT`. For SMARM, the minimum and maximum runtime over the 10 rounds differ by at most 3 ms (0.5%, at `B = 64`).
2. **Captions of the runtime and energy tables** now state the number of rounds, whether `T_RT` was active, and that energy is gross board energy.
3. **FAR experiments:** repetitions, time base, and baseline are described under comment 9.

*Note:* because per-round spreads are at most 0.5% of the mean for runtime and are bounded by the marker alignment for energy, we report means with these bounds rather than formal confidence intervals.

---

### R3.12 — Attested region versus Secure flash allocation

**Reviewer’s comment**  
The platform allocates half of the 512 KB flash to the Secure world, while the storage values appear to assume a 512 KB measurement region. Please clarify the exact attested addresses and size.

**Our response**  
We thank the reviewer for spotting this ambiguity. The reviewer is correct: our storage values, and all runtime and energy values, are based on a 512 KiB measurement region. The original manuscript did not explain how this region relates to the Secure/Non-Secure flash split.

The flash layout of our prototype is:
- **Upper 256 KiB (`0x08040000–0x0807FFFF`):** the Non-Secure firmware image (RTOS and application). This is the attestation target in our threat model.
- **Lower 256 KiB (Secure alias `0x0C000000–0x0C03FFFF`):** the Secure-world image (SMARM Service and NSC veneers). It belongs to the TCB and is protected by TrustZone-M, so our threat model does not require attesting it.

To evaluate a larger workload, the runtime and energy benchmarks let the SMARM Service traverse the whole flash address range `0x08000000–0x0807FFFF` in one shuffled traversal per round. Therefore `|M| = 512 KiB` and `n = |M|/B`, and Fig. 9 uses the same `n`.

Covering the Secure half does not distort the results, for two reasons:
- SHA-256/HMAC, the PRNG, and FFX run in data-independent time, so the cost depends only on the number of bytes hashed and on `n`, not on which region supplies the bytes.
- Hashing dominates the per-byte cost: about 100 cycles per byte (481 ms for 512 KiB at 110 MHz, `B = 4096`), whereas a flash word read costs only a few cycles.

Our numbers therefore correspond to attesting a 512 KiB memory region, and they are upper bounds for a deployment that attests only the 256 KiB Non-Secure image. For such a deployment:
- `n` halves for every `B`;
- hashing and the per-block FFX costs scale linearly with `|M|`;
- the SMARM+PRNG bitmap-scan cost scales with `n²`;
- the storage values shrink accordingly. For example, at `B = 1024` bytes, SMARM’s permutation shrinks from 576 to 256 bytes.

**Changes in the revised manuscript**  
1. **Section IV (Implementation), new paragraph “Attested memory region”.** It gives the attestation target (the Non-Secure image), the exact benchmark address range and size, the single traversal per round, why covering the Secure half does not affect the measured cost, and the upper-bound argument for a Non-Secure-only deployment.
2. **Table I caption and the “Secure Storage” and “Overall Attestation Runtime” subsections.** They now state `|M| = 512 KiB` and the address range explicitly, so that every reported number is tied to this region.

---

### R3.13 — Measured peak Secure RAM and flash

**Reviewer’s comment**  
Report measured peak Secure RAM and flash use, including bitmap packing, PRNG/FFX contexts, round keys, tweak, HMAC state, stack, code, and library overhead.

**Our response**  
We agree. Fig. 9 now shows permutation state only (see comment 14). The complete footprint is reported in a new subsection and table, measured on the same Secure builds that produced the runtime results. We report it even where it narrows our advantage.

**Changes in the revised manuscript**  
1. **Section VI, new subsection “Measured Secure Memory Footprint” with Table (tab:footprint).**
   - **Method:**
     - Builds: GCC 13.3.1, `-Os`.
     - Code and static data come from the ELF symbol table.
     - Worst-case stack is the maximum over the call graph (extracted from the disassembly) of the per-function frames reported by `-fstack-usage`. There is no recursion, and the single indirect call (Speck variant dispatch) is bounded by its largest target.
   - **Flash (attestation path):** code 2,100 / 2,128 / 2,704 B plus constant tables 538 / 538 / 55 B (SMARM / SMARM+PRNG / SMARM+FPE). The tables are the AES S-boxes and round constants, the Speck parameters, and the key.
   - **Secure RAM:**

     | Component | SMARM | SMARM+PRNG | SMARM+FPE |
     |---|---|---|---|
     | HMAC context | 232 B | 232 B | 232 B |
     | Speck/FF1 cipher object | — | — | 628 B |
     | Permutation state | 4n B | n/8 B | — |
     | Worst-case stack | 1,024 B | 1,016 B | 760 B |

     - The AES-128 context (round keys and counter) is on the stack, so it is included in the worst-case stack.
     - A Non-Secure interrupt between blocks adds a 72-byte exception frame on the Secure stack.
   - **Peak Secure RAM:**

     | Block size | SMARM | SMARM+PRNG | SMARM+FPE |
     |---|---|---|---|
     | B = 64 | 34,096 B | 2,344 B | 1,692 B |
     | B = 1024 | 3,376 B | 1,384 B | 1,692 B |
     | B = 4096 | 1,840 B | 1,336 B | 1,692 B |

   - **Library overhead:**
     - Each complete Secure image is about 33–34 KB of flash, of which the attestation path is 2.6–2.8 KB. The rest (vendor HAL, startup, FreeRTOS Secure-context support, debug UART) is common to all schemes.
     - The attestation runs on the task’s 4,096-byte Secure-context stack, which covers the worst case.
2. **Honest reading of the numbers (same subsection and the Trade-off subsection).**
   - Because SMARM+FPE carries constant Speck state, it has the smaller total RAM only for B ≤ 128 bytes.
   - At the recommended B = 1024, SMARM+PRNG is smaller (1,384 vs. 1,692 B). The reductions in total Secure RAM relative to SMARM are 2.4× and 2.0×.
   - We state that the permutation-state ratios (9×, 36×) describe scaling with n, not total footprint at moderate n.
   - We also note that our Speck object reserves a generic 576-byte key-schedule buffer, of which Speck128/128 uses 256 bytes. A dedicated implementation would need about 300 bytes.
3. **Section VI introduction** now lists the complete footprint as part of the storage dimension.

---

### R3.14 — Figure 9 labeling

**Reviewer’s comment**  
If Figure 9 reports only permutation metadata, label it accordingly rather than “total secure storage.”

**Our response**  
We agree. Fig. 9 reports only analytical permutation state:
- SMARM: bit-packed σ, n·⌈log₂ n⌉ bits;
- SMARM+PRNG: the bitmap `used`, n bits;
- SMARM+FPE: the 16-byte FFX key.

It does not report total secure storage. We have relabelled the figure and the surrounding text accordingly, and we now state explicitly what is excluded and how the prototypes relate to the analytical values. The measured total Secure RAM/flash footprint is reported separately in response to comment 13.

**Changes in the revised manuscript**  
1. **Subsection title:** “Secure Storage” is now “Permutation-State Storage”.
2. **Fig. 9:**
   - The figure has been regenerated with larger fonts. The y-axis now reads “Permutation-state storage (bytes, log)”.
   - The caption states the formula used for each scheme and `|M| = 512 KiB`. It also states that constant-size state (HMAC context, PRNG state, Speck round keys, stack) is excluded.
3. **Text accompanying Fig. 9:**
   - The values are analytical and count only the secret state that encodes the measurement order.
   - The SMARM+PRNG prototype stores `used` as a bit-packed array, matching the analysis.
   - The SMARM prototype stores σ as 32-bit integers (4n bytes), so the plotted bit-packed value is a lower bound for SMARM and favours the baseline.
4. **Reduction ratios** (13×, 832×, 9×, 36×, in the storage subsection and the recommendations): now explicitly described as permutation-state reductions. We also corrected the SMARM+PRNG ratio for `B ≥ 512` bytes from “9×” to the exact range “7–10×”.

---

### R3.15 — Release of implementation and experimental artifacts

**Reviewer’s comment**  
Release code and artifacts for reproducibility.

**Our response**  
We agree. We will release the Secure and Non-Secure firmware of all three schemes, the FAR, runtime, and energy measurement harnesses, the raw measurement logs, and the scripts that generate the figures and tables, in a public repository upon publication. The release will state which build and configuration produced each table and figure.

**Changes in the revised manuscript**  
1. **New “Artifact Availability” statement after the Conclusion**, with the content above.

---

### R3.16 — Template metadata and overstated claims

**Reviewer’s comment**  
Correct the template metadata, and moderate claims such as “first systematic study”, “eliminates secure storage” and “preserves security guarantees” until revised evidence supports them.

**Our response**  
We agree. We have moderated each of these claims so that it states only what our evidence supports.

**Changes in the revised manuscript**  
1. **“First systematic study/evaluation”** (Abstract, Contributions, Conclusion): replaced with “a systematic study/evaluation”.
2. **“Preserves SMARM’s security guarantees”** (Abstract, Contributions including the statement of the key design challenge, Section III security, Section V intro, SMARM+PRNG security, Conclusion):
   - Now reads that the designs *satisfy the same security requirements (S1)–(S3)* under the TrustZone-M assumptions of our system and threat model.
   - The formal arguments are in the Appendix, which has been revised in response to Reviewer 1, comment 5, and Reviewer 3, comments 1–2.
3. **“Eliminates secure storage”** (SMARM+FPE section title, Section V overview, storage trade-off):
   - The subsection is retitled “SMARM+FPE – Constant-Size Secure Storage”.
   - The text now states that SMARM+FPE removes only the *n-dependent permutation state*. A constant-size secure state remains: the FFX key material (including the precomputed Speck round keys) and tweak, the HMAC context, and the loop index.
   - The caption of the related-work comparison table now clarifies that its “Secure Storage” column refers to permutation state as a function of *n*.
   - Storage ratios (9×, 36×, 832×) are labelled as *permutation-state* ratios throughout. The complete measured Secure RAM is reported separately (see comment 13). It shows smaller total reductions of 2.0–2.4× at B = 1024 and up to 20× at B = 64, and SMARM+FPE has the smaller total footprint only when n is large.
   - The Discussion no longer says that SMARM+FPE “minimizes Secure RAM” in general. It now says that its Secure RAM is bounded independently of the memory size and block size, and is the smallest when n is large.
   - The Conclusion now states the total-footprint reductions and the T = 7 cost of the 10⁻³ detection target (see comment 7), instead of only the permutation-state savings.
4. **Real-time claims** (Abstract, Contributions, Section V intro, Conclusion):
   - Statements that SMARM+ “preserves real-time behavior/compatibility” now explicitly say *as measured by FAR*, or *FAR comparable to SMARM at the same block size*.
   - This avoids implying schedulability guarantees that FAR does not provide (see also R3.8 and Reviewer 1, comment 6).
5. **Template metadata:**
   - The “VOLUME 11, 2023” footer came from the IEEE Access class default.
   - The revised manuscript uses the current IEEE Access template metadata, with no stale history or DOI placeholders.

---

### R3.17 — References: insufficient in some areas, inappropriate in others

**Reviewer’s comment**  
The references are relevant but insufficient in several areas: real-time analysis, interrupt blocking, Armv8-M exceptions and security state, TF-M, secure boot and provisioning, DMA-aware threats, constant-time implementation, unbiased bounded sampling, and concrete FPE security. The comparison with RTOS-aware and interruptible attestation should also be strengthened. References [1], [2] and [8] support market claims and should be removed or replaced with primary or peer-reviewed sources, or the claims dropped. Reference [14] should not be used for general security assertions, though it may remain as a device-specific example. Reference [46] should be kept, but the construction should be revised to a clear HMAC-SHA-256/KDF and [46] cited only for what it supports.

**Our response**  
We agree and have revised the references as follows.

**Changes in the revised manuscript**  
1. **[1] Statista and [2] Kaspersky (Introduction).**
   - We removed the device-count market figure.
   - The statement that IoT devices are targets of large-scale malware is now supported by peer-reviewed sources on the Mirai botnet (Antonakakis et al., USENIX Security 2017; Kolias et al., IEEE Computer 2017).
2. **[8] Arm community blog (Introduction and Section II-B).**
   - We removed the “22 billion devices” claim.
   - The availability of TrustZone-M is now supported by primary sources: the Armv8-M Architecture Reference Manual, the peer-reviewed TrustZone-M security study by Luo et al., and the STM32L552 platform documentation.
3. **[14] Silicon Labs AN1271.**
   - It is no longer cited for the general assertion about TrustZone-M misconfiguration. That sentence now relies only on the peer-reviewed study by Luo et al.
   - AN1271 remains only where it serves as a device-specific example of another Secure-world service (secure key storage).
4. **[46] Bellare–Canetti–Krawczyk.**
   - The appendix assumption formerly titled “SHA-256 as PRF” now states that HMAC-SHA-256 is a PRF. It cites [46] and Bellare’s CRYPTO 2006 proof of HMAC’s PRF security.
   - We state explicitly that every keyed use of SHA-256 in the paper (the KDF, the order-generation seed, and the attestation MAC) is instantiated with HMAC-SHA-256. No other keyed construction of raw SHA-256 is assumed to be a PRF.
   - Key derivation now also cites HKDF (Krawczyk, CRYPTO 2010) and NIST SP 800-108r1 (see Reviewer 1, comment 4).
5. **Additional references**, added together with the corresponding technical revisions:
   - **Armv8-M exceptions and security state:** Armv8-M Architecture Reference Manual; ST PM0264 (Cortex-M33 programming manual) (comment 6).
   - **Secure boot, key provisioning, and DMA-aware threats:** ST AN5347 (TrustZone features of STM32L5/U5/U3, including BOOT_LOCK, RDP, secure firmware installation, and GTZC); Kiri et al., VeCoDI (arXiv 2026, DOI 10.48550/arXiv.2606.07470), for the TrustZone-M trust model with a security DMA controller (comment 6).
   - **Unbiased bounded sampling and constant-time implementation:** Lemire, “Fast Random Integer Generation in an Interval” (ACM TOMACS 2019, DOI 10.1145/3230636); Fisher–Yates/Durstenfeld (comments 3–4).
   - **Concrete FPE security:** NIST SP 800-38G (FF1); Bellare–Rogaway–Spies (FFX); Hoang–Tessaro–Trieu, “The Curse of Small Domains” (CRYPTO 2018); Speck (Beaulieu et al.) (comment 4).
   - **Key derivation:** HKDF (Krawczyk, CRYPTO 2010) and NIST SP 800-108r1 (Reviewer 1, comment 4).
   - **Measurement equipment:** FNIRSI FNB58 user manual, for the power-meter accuracy specification (comment 11).

   - **Real-time analysis and interrupt blocking:** Joseph–Pandya (Computer J. 1986, DOI 10.1093/comjnl/29.5.390); Audsley et al. (Softw. Eng. J. 1993, DOI 10.1049/sej.1993.0034); Sha–Rajkumar–Lehoczky (IEEE TC 1990, DOI 10.1109/12.57058); Buttazzo, *Hard Real-Time Computing Systems* (Springer 2011, DOI 10.1007/978-1-4614-0676-1) (comment 8).

   - **TF-M:** Trusted Firmware-M documentation, including its platform support for our board. A new item in Section III (Platform Security Configuration) states the Secure runtime of each prototype: with FreeRTOS, a bare-metal Secure image (which the other configuration items describe); with Zephyr, TF-M, where the Secure Service is a TF-M secure partition called through TF-M's client API and TF-M's bootloader provides the secure-boot assumption. It also states that TF-M's Initial Attestation service (boot-time measurements) is complementary to run-time memory attestation (comment 6). The Implementation section, the caption of Fig. 7, the FAR results (the tick-loss explanation is given for FreeRTOS; Zephyr shows the same threshold) and the Limitations now state this difference, and note that the FreeRTOS–Zephyr comparison therefore does not isolate the RTOS kernel alone.
   - **Security in real-time scheduling:** Mishra et al., CFI survey for real-time embedded systems (ACM TECS 2022, DOI 10.1145/3538275); RECFISH (RTNS 2023, DOI 10.1145/3575757.3575762); Kansal et al. (RTCSA 2023, DOI 10.1109/RTCSA58653.2023.00033).
   - **Interruptible attestation with hardware support:** IDA (NDSS 2024, DOI 10.14722/ndss.2024.23059).
6. **Strengthened comparison with RTOS-aware and interruptible attestation (Section VII).** The comparison no longer relies only on asymptotic storage.
   - **New paragraph “Comparison with RTOS-aware attestation”** covers ISC-FLAT, PEARTS, RealSWATT, and IDA. It compares:
     - what is attested (memory contents vs. control flow or execution);
     - the trust anchor and TCB (TrustZone-M Secure image vs. no hardware anchor on a multi-core MCU vs. an added hardware monitor);
     - the real-time cost (one non-preemptible interval per block, measured by FAR and bounded analytically);
     - the practical Secure memory (1.4–1.7 KB for SMARM+ vs. 3.4 KB for SMARM at `B = 1024`, measured).
   - **New paragraph “Real-time scheduling of security mechanisms”** positions our blocking analysis relative to RECFISH and Kansal et al.
   - **Revised paragraph on Joia Neto et al. (tick loss under atomic Secure services).** It explains that our FAR drop for `B ≥ 2048` is an instance of that effect. Their compensation mechanism would restore the time base but not the blocking, so the two approaches can be combined. The FAR Results paragraph now cites their tick-loss model.
   - **The caption of the positioning table** now points to the measured footprint table.

---

### Reviewer 3 — Additional questions

**Novelty.** We agree that the order-generation methods build on established techniques. The third contribution in Section I now says so explicitly: both variants build on sampling without replacement over a bitmap and on format-preserving encryption, and our contribution lies in applying them to shuffled-measurement attestation on TrustZone-M and evaluating their cost. The claims “first systematic study”, “eliminates secure storage” and “preserves security guarantees” have been moderated (comment 16).

**Technical soundness.** Each point in this summary is addressed under the corresponding comment:
- FAR versus deadlines, response time, jitter and worst-case blocking, and the use of FAR for broad claims and the 1024-byte operating point: comments 8 and 10, and Reviewer 1, comment 6.
- One task, the three frequencies, and the missing task parameters and experiment duration: comment 9.
- Variable-length `FindKthZero` and the first scan from an all-zero bitmap: comment 3. The revised algorithm scans all `n` bits with a branchless update, so its running time does not depend on the rank or the selected index.
- The full-permutation MOI game and timing in the proofs: comments 1 and 2 (Appendix revision).
- The inconsistent hash notation in the Appendix and the scope of [46]: comment 17, item 4. The main text defines every keyed use of SHA-256 as HMAC-SHA-256; the Appendix games are being aligned with this notation and with the derived keys `k_prp` / `k_mac` in the Appendix revision.
- FFX over Speck without encoding, rounds, key schedule and concrete security: comment 4.
- One traversal despite multi-traversal detection claims: comment 7.

**Comprehensiveness.** The revision adds a “Limitations” subsection (Section VI) and an “Artifact Availability” statement (after the Conclusion), and reports the platform configuration (comment 6) and the real-time task parameters (comment 9). The Limitations state that all measurements use one board and one tick rate; evaluating further hardware configurations is left to future work.
