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

**Changes in the revised manuscript**  
1. Section II-A (Finalization): added an explicit statement of *order-based challenge binding*.  
2. Caption of Algorithm 1: noted that `chal` is not an explicit HMAC input and is bound via `σ`; stated that the same binding applies to Algorithms 2 and 3 (SMARM+PRNG / SMARM+FPE).

We did not add an explicit `HMAC.Update(chal)` step, because it is unnecessary for the intended security property and would change the protocol relative to the original SMARM challenge-binding style without improving the soundness argument above.

---

*(R1.2–R1.6 will be filled in as we address each comment.)*
