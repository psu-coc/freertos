SMARM+ IEEE Access — Reviewer Concerns (Detailed Checklist)
Use this to mark each item: Agree / Clarify (we already did it) / Partial / Reject with rebuttal.

Reviewer 1
R1.1 — Challenge not bound into attestation evidence
Concern: Algorithm 1 derives the permutation seed from Hash(chal, k), but the final evidence is essentially HMAC over the shuffled memory blocks. The challenge affects block order, but is not explicitly MACed into the final transcript.

Why they raise it: Without explicit chal in the MAC input, a verifier/protocol reader may worry about freshness binding, transcript completeness, or analysis clarity—even if order dependence makes evidence challenge-dependent indirectly.

Likely paper gap: Final evidence construction is not written as MAC(k, chal || … || measurement) (or equivalent).

R1.2 — Between-block transition / continuation integrity underspecified
Concern: Interrupts are re-enabled between blocks, creating an adversarial interval. Before returning to Non-Secure world, all state needed for the next secure measurement (HMAC state, loop counter, permutation/bitmap/FPE state) must reside only in Secure memory and resume at an authenticated continuation point.

Why they raise it: TrustZone-M entry integrity is treated as an implementation detail; for this protocol it is fundamental to correctness under inter-block preemption.

Likely paper gap: Missing an explicit invariant for Secure state across EnableInterrupts → NS execution → next Secure entry.

R1.3 — SMARM+PRNG timing leakage via FindKthZero
Concern: If the scan terminates when the r-th zero is found, execution time correlates with the chosen block index / bitmap state. A Non-Secure adversary observing interrupt/resumption timing could learn partial information about the private order → conflicts with S3.

Why they raise it: Algorithm/prose say “scan to locate the r-th zero” / “up to O(n)”, which reads as early-exit variable time.

Your position (candidate “they didn’t read carefully” / missing text): Implementation always scans all n bitmap entries (as in thesis); time independent of selected idx. Access manuscript likely omitted this mitigation paragraph.

R1.4 — No explicit key domain separation
Concern: The same master key k appears involved in seed derivation, HMAC evidence, and (for SMARM+FPE) FFX key/tweak derivation.

Why they raise it: Classic crypto hygiene: one key used for multiple purposes without KDF / labeled domain separation.

Likely paper gap: No explicit KDF(k, “perm”), KDF(k, “mac”), KDF(k, “ffx”) (or equivalent) description.

R1.5 — Theorem A.1 reduction is invalid
Concern: The proof argues roughly: predicting σ implies recovering the PRF-derived seed; since the seed is pseudorandom, prediction is impossible. That implication is not generally valid (predicting a function of a secret ≠ recovering the secret).

What they want: A proper reduction: a successful permutation predictor yields a distinguisher between Fisher–Yates output from the PRF-derived seed and an appropriately random permutation (or equivalent PRF/PRNG game hop done correctly).

Likely paper gap: Proof structure / wording of the reduction, not necessarily the informal security intuition.

R1.6 — Evaluation insufficient beyond FAR
Concern: FAR isolates interference from atomic measurements, which is useful, but does not measure total real-time impact of running SMARM (CPU for permutation generation, PRNG scanning, FPE, hashing, wrapper/service).

Why they raise it: Paper claims real-time coexistence largely via FAR; reviewer wants broader runtime/scheduling impact.

Reviewer 2
R2.1 — Bibliography DOIs
Concern: For each bibliography entry, provide DOI when available.

R2.2 — Figure text too small
Concern: Make text in all figures bigger, or make figures two-column wide; currently nearly unreadable when printed on A4.

R2.3 — Attestation time vs attested size / window of exposure
Concern: When presenting total computation time for complete attestation (all blocks), specify total attested memory size, and comment that longer attestation ⇒ longer undetected-attack window (window-of-exposure).

Reviewer 3
R3.1 — Reformulate security analysis around real attacker objective
Concern: Replace the exact full-permutation MOI game with a definition capturing:

next-block prediction
partial-order leakage
resulting roving-malware evasion
R3.2 — Timing must enter the proofs
Concern: Incorporate all timing observations into the proofs.

Related to R1.3: If always-scan is true, rebuttal + write it into analysis; if not proven/documented in Access version, they treat it as open.

R3.3 — Constant-time FindKthZero / bounded sampling
Concern: Either implement FindKthZero and bounded random sampling in constant time, or demonstrate experimentally/analytically that timing reveals no useful information.

Candidate clarify: Always-full-scan already aims at constant time w.r.t. selected index; may still need clearer algorithm box + optional measurement.

R3.4 — Fully specify crypto building blocks
Concern: Define PRNG, RandInt, KDF, domain separation, FFX domain encoding, number of rounds, Speck parameters, and concrete security level.

R3.5 — Explicitly authenticate protocol metadata + separate keys
Concern: Explicitly authenticate:

challenge
scheme identifier
block size
block count
memory base
memory length
And derive separate keys for permutation generation and attestation.

Overlaps: R1.1, R1.4.

R3.6 — Platform / systems security details
Concern: Explain how the implementation:

masks Secure and Non-Secure exceptions
handles NMI and faults
prevents DMA / other bus masters from changing measured memory
establishes secure boot
provisions the key
validates NSC arguments
fixes attested memory boundaries
R3.7 — Multiple shuffled traversals in evaluation
Concern: Evaluate the number of independent shuffled traversals required for the stated malware-detection probability, and include that complete configuration in runtime and energy results.

Why they raise it: Paper cites ~37% evasion for one traversal and <10⁻³ with seven traversals, but evaluation appears to report a single traversal.

R3.8 — Strengthen / replace FAR with classical RT metrics
Concern: Supplement or replace FAR with:

max / percentile response time
release jitter
interrupt latency
deadline-miss ratio
worst-case blocking
response-time / schedulability analysis
R3.9 — Fully specify the real-time task model
Concern: Specify each task’s WCET, period, deadline, priority, utilization, computation, release and catch-up semantics, phase, and experimental duration.

R3.10 — Broader real-time experimental campaign
Concern: Evaluate:

multiple task sets
at least one representative real-time application
different release phases
wall-clock attestation completion under preemption
both SMARM+ variants on both RTOSes
R3.11 — Measurement methodology rigor
Concern: Report repetitions, confidence intervals, power-meter accuracy, calibration, integration procedure, and idle-energy treatment.

R3.12 — Clarify attested region vs Secure flash allocation
Concern: Platform allocates half of 512 KB flash to Secure world, while storage values appear to assume a 512 KB measurement region — clarify exact attested addresses/size.

R3.13 — Report measured peak Secure RAM and flash use
Concern: Include bitmap packing, PRNG/FFX contexts, round keys, tweak, HMAC state, stack, code, and library overhead.

R3.14 — Figure 9 labeling
Concern: If Figure 9 reports only permutation metadata, label it accordingly rather than “total secure storage.”

R3.15 — Release implementation and experimental artifacts
Concern: Release code/artifacts for reproducibility.

R3.16 — Fix references / template metadata; moderate claims
Concern: Correct reference usage and template metadata; moderate claims such as:

“first systematic study”
“eliminates secure storage”
“preserves security guarantees”
until revised evidence supports them.

R3.17 — References insufficient / some inappropriate
Concern:

Add authoritative refs on RTA, interrupt blocking, Armv8-M exceptions/security state, TF-M, secure boot/provisioning, DMA-aware threats, constant-time, unbiased bounded sampling, concrete FPE security.
Strengthen comparison with RTOS-aware / interruptible attestation (threat assumptions, deadline metrics, TCB, practical memory)—not only asymptotic storage.
Remove/replace [1], [2], [8] (market claims) with primary/peer-reviewed sources or drop associated claims.
Replace [14] where used for general security assertions (may keep as device-specific example).
Keep [46] but revise construction to clear HMAC-SHA-256 / KDF and cite [46] only for what it actually supports.
Extra statements in R3 “Additional Questions” (same themes, sharper)
R3.AQ.1 — Novelty framing
Main novelty is systems integration/evaluation; order-generation methods use established techniques. Must correct security analysis and strengthen RT evaluation before a reliable contribution.

R3.AQ.2 — Technical soundness attack summary
FAR ≠ deadline satisfaction / response / jitter / WCB.
One task; only 10/100/1000 Hz; missing WCET/deadline/priority/utilization/release/duration/uncertainty.
Paper admits FAR ≠ schedulability but still uses FAR for broad RT claims and 1024-byte operating point.
Algorithm 2 variable-length FindKthZero; adversary sees timestamps; first scan from all-zero bitmap can reveal rank/index; proofs ignore timing.
MOI full-permutation game too weak against next-block/prefix leakage.
Appendix inconsistent: Hash(chal,k), SHA256(chal||k), SHA256(k,·), HMAC-SHA256; [46] supports HMAC-style, not unspecified keyed SHA-256.
FFX-Speck assumed PRP without encoding/rounds/key schedule/concrete security/constant-time.
Eval reports one traversal despite multi-traversal detection claims.
R3.AQ.3 — Comprehensiveness gaps
Missing dedicated limitations/reproducibility section; missing many platform and RT task details listed above; should evaluate more hardware configurations.

Suggested “they didn’t read carefully / manuscript omitted it” candidates
Mark these first when you review:

ID	Candidate issue
R1.3 / R3.3 / R3.AQ.2 timing
Strongest candidate: always-full-scan exists in thesis, likely missing in Access text → reviewer assumed early exit
R1.1
May be partially “already challenge-dependent via order”; still fair to ask for explicit bind
R3.14
May be labeling/caption issue only
R3.12
May already be clear in text/figures; check if they missed it
R2.*
Usually not “misread”—just polish requests
R1.5, R3.1
Usually not misread—proof/game design critique
R3.8–R3.11
Scope expansion; not “didn’t read,” they want more
Compact ID list (for your markup)
R1.1  challenge bind
R1.2  between-block Secure invariant
R1.3  FindKthZero timing / S3
R1.4  key domain separation
R1.5  Theorem A.1 reduction
R1.6  FAR incomplete for total RT impact
R2.1  DOIs
R2.2  figure font/size
R2.3  attested size + window-of-exposure
R3.1  MOI → next-block/partial-order game
R3.2  timing in proofs
R3.3  constant-time FindKthZero / RandInt
R3.4  specify PRNG/KDF/FFX/Speck
R3.5  auth metadata + split keys
R3.6  exceptions/NMI/DMA/boot/NSC/bounds
R3.7  multi-traversal in runtime/energy
R3.8  RT metrics beyond FAR
R3.9  full task parameters
R3.10 multi task sets / apps / phases / both RTOS
R3.11 CI / power methodology
R3.12 attested region clarity
R3.13 peak Secure RAM/flash breakdown
R3.14 Fig.9 label
R3.15 artifacts
R3.16 moderate claims + refs/template
R3.17 bibliography fixes