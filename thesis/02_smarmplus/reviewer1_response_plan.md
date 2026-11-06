# SMARM+ IEEE Access — Reviewer 1 (แผนตอบ + แก้ paper)

เอกสารนี้รวมเฉพาะ **Reviewer 1** ทั้ง 6 ข้อ  
รูปแบบแต่ละข้อ: คอมเมนต์เดิม → อธิบายไทย → reviewer ถูกแค่ไหน → Response → Action / สิ่งที่ต้องทำ

**สถานะภาพรวม:** ไม่ต้องออกแบบโปรโตคอลใหม่ทั้งก้อน และส่วนใหญ่ **ไม่ต้องรันทดลองใหม่**  
งานหลัก = ตอบ reviewer + เติม/แก้ข้อความ (และบางข้อแก้ proof / แยก key ใน algorithm)

---

## R1.1 — Challenge ไม่ถูก MAC เข้า evidence โดยตรง

### คอมเมนต์เดิม (Reviewer)
> Algorithm 1 does not clearly bind the challenge into the final attestation evidence. The algorithm derives the permutation seed from Hash(chal,k), but the final evidence is essentially the HMAC accumulated over the shuffled memory blocks. The challenge affects the order of the blocks, but apparently is not explicitly MACed into the final transcript. This may still produce challenge-dependent evidence because the concatenation order changes.

### อธิบายเป็นไทย
Algorithm 1 ใช้ `chal` ตอนสร้างลำดับการวัด (`σ`) แต่ตอนจบ HMAC **ไม่ได้เอา `chal` ยัดเข้าข้อความที่ MAC โดยตรง**  
Reviewer กลัวว่า challenge ไม่ผูกกับหลักฐานชัด  
แต่เขาก็รับเองว่าอาจยังผูกได้เพราะ **ลำดับบล็อกเปลี่ยนตาม chal → ค่า HMAC เปลี่ยน**

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูกบางส่วน** — ใน Algorithm 1 จริงๆ ไม่มี `HMAC.Update(chal)` |
| ผิด/เกินไหม | **ไม่ใช่ช่องโหว่ใหญ่** — paper มีแล้วใน Section II-A ว่า verifier ใช้ `k`+`chal` สร้าง `σ` ใหม่แล้วเทียบ |
| สรุป | เป็นเรื่อง **เขียนให้ชัด** มากกว่า protocol พัง |

### เราเลือกทางไหน
**ทางแรก:** ไม่แก้โค้ดเพิ่ม `HMAC(chal)` — แค่ rebuttal + เขียนใน paper ว่า bind ผ่านลำดับ

### Response (อังกฤษ — ใช้ส่งได้)
We agree that Algorithm 1 does not show an explicit `HMAC.Update(chal)` step. However, the challenge is already cryptographically bound into the attestation evidence through the measurement order. Because `σ = FisherYates(Hash(chal, k), …)` and the HMAC is accumulated over blocks in the order `σ(0),…,σ(n−1)`, a different challenge yields a different concatenation order and thus a different tag (except with negligible collision probability). As stated in Section II-A, the verifier, given `k` and `chal`, reconstructs `σ` and recomputes the expected evidence; verification therefore fails for a stale or substituted challenge. This is the same challenge-binding mechanism as in the original SMARM design. The reviewer’s observation that challenge-dependent evidence can still arise “because the concatenation order changes” is exactly this intended binding.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| แก้ firmware / ทดลองใหม่ | **ไม่** |
| Response to reviewers | ตามด้านบน |
| แก้ paper | **ใช่ — น้อยๆ** |

**ใส่ใน paper:**
1. Caption/หมายเหตุใต้ Algorithm 1 (และโยง Algorithm 2/3): `chal` ไม่ได้ HMAC โดยตรง แต่ bind ผ่านลำดับ `σ`
2. Section II-A Finalization: เติม 1 ประโยคว่า freshness = order-based binding

---

## R1.2 — ช่วงระหว่างบล็อก (adversarial interval) / continuation invariant

### คอมเมนต์เดิม (Reviewer)
> The transition between block measurements needs much more careful and detailed analysis. The main security idea is that interrupts are disabled only during a block measurement and are then re-enabled. But this means there is an adversarial interval between blocks. The crucial invariant should be precisely stated: before returning control to the Non-Secure world, all state required for the next secure measurement—HMAC state, loop counter, permutation state/bitmap/FPE state—must reside exclusively in Secure memory and must resume at exactly the authenticated continuation point. The paper argues this informally through TrustZone-M entry integrity, but for this protocol this is not an implementation detail; it is fundamental to correctness.

### อธิบายเป็นไทย
ระหว่างบล็อก **เปิด interrupt** → Non-Secure / malware รันได้  
Reviewer อยากให้เขียนกฎชัดๆ ว่าก่อนเปิดให้ NS กลับมา:

1. สถานะที่ต้องใช้ต่อ (HMAC, ตัวนับบล็อก, `σ`/bitmap/FPE) อยู่แค่ Secure  
2. กลับมา Secure ต้องต่อจุดเดิม ไม่ให้โดนสั่งกระโดดไปที่อื่น  

เค้าบอกว่าเรื่องนี้เป็นหัวใจโปรโตคอล ไม่ใช่แค่รายละเอียด TrustZone

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูก** — ควรมี invariant ชัด |
| ของเรามีอยู่แล้วไหม | มีแนวคิดใน (S1) + Appendix แล้ว แต่ **ไม่ได้ไล่รายการ state** แบบที่เค้าขอ |
| สรุป | **เขียนไม่ละเอียดพอ** ไม่ใช่ design ผิด |

### Response (อังกฤษ — ใช้ส่งได้)
We agree that the between-block interval is security-critical and that the required continuation invariant should be stated explicitly. Our design already places all attestation continuation state in Secure memory and relies on TrustZone-M to prevent Non-Secure code from reading/writing that state or redirecting Secure control flow (Section III and Theorem A.1). In particular, after each block measurement interrupts may be re-enabled, but: (i) the HMAC context, loop index, and order-related state (`σ`, or the PRNG bitmap / FPE state in SMARM+) remain in Secure RAM; (ii) Non-Secure software cannot access these locations; and (iii) upon resumption, Secure execution continues at the interrupted point (or re-enters only through the designated NSC entry), and cannot be redirected to an arbitrary Secure-world location. We also keep interrupts disabled during each individual block measurement, so a block cannot be partially measured under Non-Secure interference. The original manuscript discussed these points mainly through (S1) and the TrustZone-M architecture, without naming a single continuation invariant that enumerates the protected state. We therefore treat this as a clarity/analysis gap rather than a protocol redesign.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| แก้โค้ด / ทดลองใหม่ | **ไม่** |
| Response | ตามด้านบน |
| แก้ paper | **ใช่ — เพิ่มย่อหน้า** |

**ใส่ใน paper:**
1. Section III: หัวข้อ/ย่อหน้า **Between-block continuation invariant** (HMAC, loop index, order state อยู่ใน Secure; resume จุดเดิม)
2. SMARM+PRNG / +FPE: อย่างละ 1 ประโยคว่า state คืออะไร
3. Appendix (S1): โยงกลับไป invariant นี้

---

## R1.3 — Timing leak ของ SMARM+PRNG (`FindKthZero`)

### คอมเมนต์เดิม (Reviewer)
> SMARM+PRNG has a potentially serious timing-leakage problem. If the scan terminates when the r-th zero is found, then its execution time is potentially correlated with the chosen block index and current bitmap state. Because the attacker controls the Non-Secure world and can observe interrupt/resumption timing, this could reveal partial information about the supposedly private measurement order. This directly conflicts with the key protocol requirement S3.

### อธิบายเป็นไทย
ถ้าสแกน bitmap แล้ว **เจอ zero ตัวที่ r แล้วหยุด** เวลาจะสั้น/ยาวตามตำแหน่งที่เลือก  
attacker ดูจังหวะ interrupt ได้ → เดาลำดับได้ → ชน S3  

แต่ของเราตั้งใจให้ **สแกนครบทุกช่องเสมอ** (เจอแล้วก็วิ่งต่อ) → เวลาไม่ขึ้นกับว่าเลือกบล็อกไหน  
ปัญหาคือ **Access PDF ไม่ได้เขียน** เรื่องนี้ (thesis มี) เลยถูกสมมุติว่า early-exit

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูกถ้า early-exit** — แต่ implementation ของเราไม่ใช่แบบนั้น |
| ถูกที่คอมเมนต์จาก paper ไหม | **ถูก** — เพราะข้อความ Access ฟังเหมือนสแกนหาแล้วจบ |
| สรุป | **documentation gap** ไม่ใช่ redesign |

### Response (อังกฤษ — ใช้ส่งได้)
We agree that an early-exit bitmap scan would be a timing side channel against S3. Our SMARM+PRNG implementation does **not** use early exit. `FindKthZero` always traverses all *n* entries of the `used` bitmap: after the *r*-th zero is identified, the remaining entries are still scanned so that the procedure’s duration is independent of the selected index `idx` (and thus does not reveal which remaining block was chosen). The wall-clock cost may still depend on *n* (public from the attested size and block size *B*), but not on the secret order. The submitted manuscript described the scan functionally (“locate the *r*-th zero”) and its asymptotic cost, but did not state this constant-time-with-respect-to-`idx` implementation choice explicitly—leaving room for the interpretation assumed in the review. We therefore treat this as a **documentation gap**, not as an unprotected timing leak in the implemented design.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| แก้โค้ด | **ไม่** ถ้า already always-scan (ควรเช็คโค้ดหนึ่งครั้งก่อนส่ง) |
| ทดลองใหม่ | **ไม่จำเป็น** |
| Response | ตามด้านบน |
| แก้ paper | **ใช่ — สำคัญ** |

**ใส่ใน paper:**
1. ใต้ (S3) ของ SMARM+PRNG: ย่อหน้า side-channel + always scan all *n*
2. Algorithm 2: comment `▷ always scans all n bits; no early exit`
3. แก้คำว่า “up to O(n)” ให้สื่อว่าเป็น full pass / Θ(n)

---

## R1.4 — ไม่มี domain separation ของ key

### คอมเมนต์เดิม (Reviewer)
> There is no explicit domain separation in key derivation. The same master key k appears to be involved in seed derivation, HMAC evidence, and—in SMARM+FPE—the derivation of the FFX key/tweak. Please elaborate.

### อธิบายเป็นไทย
กุญแจหลัก `k` ถูกใช้หลายงาน:
- สร้าง seed / ลำดับ
- HMAC หลักฐาน
- FPE: `KDF(k, chal)` ได้ ffx key แต่ HMAC ยังใช้ `k` เดิม  

Crypto ปกติควรแยกกุญแจตามหน้าที่ (domain separation)

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูก** — paper ไม่มี labeled domain separation |
| ร้ายแรงแค่ไหน | hygiene / key management — ควรแก้ให้ชัด |
| สรุป | ต้อง **elaborate + ทำให้ชัดใน algorithm** |

### Response (อังกฤษ — ใช้ส่งได้)
We thank the reviewer for this comment. In the submitted manuscript, a single pre-shared master secret `k` is used as the root key, following the original SMARM keying style: the measurement order is derived from `Hash(chal, k)` (or a PRNG/FFX keyed from `k` and `chal`), while attestation evidence is computed with HMAC-SHA256 under `k`. For SMARM+FPE, Algorithm 3 already derives `(ffx_key, tweak) ← KDF(k, chal)`, but the HMAC step still uses the master key `k` directly, and the paper did not define labeled domain separation between permutation-keying and attestation-MAC uses. We agree this should be elaborated and made explicit.

These uses already go through different algorithms/interfaces (`Hash`/`PRNG`/`KDF` vs `HMAC`), but that is not a substitute for explicit domain-separated key derivation from a master secret. Lack of labeled separation is a key-management hygiene issue rather than an attack demonstrated against our evaluation; nevertheless, it is a valid cryptographic-engineering concern.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| ทดลอง FAR/energy ใหม่ | **ไม่** |
| แก้ paper | **ใช่** — อธิบาย + อัปเดต Algorithm 1–3 |
| แก้โค้ด | **แนะนำให้ทำ** ให้ตรง paper (งานน้อย) |

**ใส่ใน paper:**
1. ย่อหน้า Key hierarchy: `k_mac`, `k_ord`, `(ffx_key, tweak)` จาก KDF คนละ label  
2. อัปเดต Algorithm 1/2/3 ให้ใช้ derived keys  
3. (S2): กล่าวว่า derived keys ก็อยู่ใน Secure

---

## R1.5 — Theorem A.1 reduction ผิดรูป

### คอมเมนต์เดิม (Reviewer)
> Theorem A.1 argues essentially that if an adversary can predict the permutation σ, this implies that it can recover the PRF-derived seed; since the seed is pseudorandom, permutation prediction is therefore impossible. That implication is not generally valid. Being able to predict some deterministic function of a secret value does not imply recovery of the secret value itself. A formal reduction should instead construct a distinguisher showing that a successful permutation predictor can distinguish the Fisher-Yates output generated from the PRF-derived seed from an appropriately distributed random permutation.

### อธิบายเป็นไทย
Proof เดิมบอกว่า: เดา `σ` ได้ ⇒ ได้ `seed` ⇒ แต่ seed เดาไม่ได้ ⇒ เดา `σ` ไม่ได้  

ลูกศรแรกผิด: รู้ค่าที่คำนวณจากความลับ **≠** ได้ความลับ  
ควรสร้าง distinguisher / hybrid แทน

หมายเหตุ: **Theorem A.2 (PRNG) เขียนแบบถูกกว่าแล้ว** — แค่ A.1 (S3) อ่อน

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูกเต็มๆ** |
| สรุป | ต้อง **rewrite proof** ไม่ใช่แค่ rebut |

### Response (อังกฤษ — ใช้ส่งได้)
We agree. The (S3) argument in Theorem A.1 as submitted incorrectly treated predictability of the deterministic function `σ = FisherYates(seed, ·)` as implying recovery of `seed`. As the reviewer notes, predicting a function of a secret does not in general imply recovering the secret. The intended claim is computational indistinguishability of the induced measurement order from a uniform random permutation (under the MOI game), which requires a reduction via a distinguisher / hybrid argument, not a “predict-`σ` ⇒ recover-`seed`” step.

We note that Theorem A.2 already uses the correct style of argument for SMARM+PRNG (hybrids replacing the PRF-derived seed and then the shuffle randomness, until the order is uniform). Theorem A.1’s (S3) proof should have followed the same pattern for Fisher–Yates under a PRF-derived seed. We thank the reviewer for catching this inconsistency.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| โค้ด / ทดลอง | **ไม่** |
| Response | ตามด้านบน |
| แก้ paper | **ใช่ — แก้ Appendix Theorem A.1 (S3)** |

**ใส่ใน paper:**  
แทน proof เดิมด้วย hybrid G0 (real seed) → G1 (uniform seed + PRF reduction) → Fisher–Yates ให้ uniform permutation → MOI advantage negligible  
และระบุชัดว่า **ไม่ claim** predict σ ⇒ recover seed

---

## R1.6 — Evaluation / FAR ไม่ครอบคลุมผลกระทบ RT ทั้งก้อน

### คอมเมนต์เดิม (Reviewer)
> Evaluation is not sufficient. In the experiment, FAR isolates specifically the interference produced by atomic measurements, which is useful, but it does not measure the total real-time impact of running SMARM. CPU consumed by permutation generation, PRNG scanning, FPE, hashing, and wrapper/service execution can still affect task scheduling.

### อธิบายเป็นไทย
FAR วัดแค่ผลกระทบจาก **ช่วงปิด interrupt**  
CPU จาก permutation / PRNG / FPE / hash / wrapper ตอนเปิด IRQ ก็แย่ง CPU ได้ — FAR ไม่จับครบ

### Reviewer พูดถูกแค่ไหน
| | |
|--|--|
| ถูกไหม | **ถูกเรื่องขอบเขต FAR** |
| Paper มีอะไรชดเชยไหม | มี — Remark ว่า FAR ≠ schedulability; มี Table I/Fig.10 runtime และ Table II energy |
| สรุป | ไม่ใช่ “ไม่มีข้อมูลเลย” แต่ **เชื่อม/จำกัด claim ยังไม่ชัด**; ไม่จำเป็นต้องรับทำ R3 ทั้งก้อน |

### Response (อังกฤษ — ใช้ส่งได้)
We agree that FAR is intentionally narrow and should not be read as a complete real-time impact metric. As stated in Section IV-C, FAR compares the real-time task frequency under SMARM with interrupt-disabled block measurements against a baseline in which SMARM still runs but interrupts remain enabled during each block hash. Therefore FAR specifically quantifies degradation caused by **non-preemptible** block measurement windows, not the full CPU cost of attestation relative to an idle system without SMARM. We also already remark that FAR does not replace schedulability analysis.

The submitted manuscript separately reports the **total computational cost** of completing an attestation round—including permutation/PRNG/FPE work, hashing, and Secure-service execution—via overall runtime (Table I, Figure 10) and board-level energy (Table II). Empirically, for a fixed block size \(B\), SMARM+PRNG/FPE preserve FAR behavior while changing these end-to-end costs, which supports our claim that FAR is driven primarily by the IRQ-off window length (controlled by \(B\)), whereas PRNG/FPE mainly affect preemptible work and total attestation latency/energy.

We acknowledge that the paper did not sufficiently spell out this **division of metrics**: FAR for non-preemptible interference vs.\ runtime/energy for total attestation compute. We also agree that preemptible attestation work can still contend with real-time tasks for CPU even when FAR \(\approx 1.0\); our evaluation does not claim otherwise, and we will state this limitation more explicitly.

### Action / สิ่งที่ต้องทำ
| งาน | ทำไหม |
|-----|--------|
| ทดลองใหม่ทั้งชุด | **ไม่จำเป็น** ถ้าจำกัด claim |
| ทดลองเสริม (optional) | เทียบ task ตอนไม่ attest vs attest เปิด IRQ ตลอด — ถ้าอยากแข็งขึ้น |
| Response | ตามด้านบน |
| แก้ paper | **ใช่ — clarify + limitations** |

**ใส่ใน paper:**
1. ขยาย Remark ใน Section IV-C (FAR isolate IRQ-off; CPU preemptible ถูก cancel ใน ratio)
2. ต้น Section VI-B: โยงว่า runtime = total compute cost
3. Limitations: FAR=1 ≠ ไม่มี CPU contention; ไม่ได้ทำ full schedulability

---

## ตารางสรุป Reviewer 1 ทั้งชุด

| ข้อ | ใจความ | Reviewer ถูกแค่ไหน | ทดลองใหม่? | แก้โค้ด? | แก้ paper? |
|-----|--------|-------------------|------------|----------|------------|
| **R1.1** | chal ไม่ได้อยู่ใน HMAC ตรงๆ | ถูกบางส่วน (clarity) | ไม่ | ไม่ (ตามที่เลือก) | ใช่ น้อย |
| **R1.2** | ต้องมี continuation invariant | ถูก (ต้องเขียนชัด) | ไม่ | ไม่ | ใช่ เพิ่มย่อหน้า |
| **R1.3** | timing leak FindKthZero | ถูกจากข้อความ paper; design มี always-scan | ไม่ | เช็คแล้วไม่ต้อง | ใช่ สำคัญ |
| **R1.4** | domain separation ของ k | ถูก | ไม่ | แนะนำให้แยก key | ใช่ + อัป algorithm |
| **R1.5** | proof A.1 ผิดรูป | ถูกเต็ม | ไม่ | ไม่ | ใช่ แก้ proof |
| **R1.6** | FAR ไม่พอสำหรับ total RT | ถูกเรื่องขอบเขต; มี runtime/energy แล้ว | ไม่จำเป็น | ไม่ | ใช่ clarify + limit |

---

## Checklist งานที่คุณต้องทำ (Reviewer 1)

- [ ] เขียน Response ทั้ง 6 ข้อลงไฟล์ “Author's Response”
- [x] R1.1: เพิ่มข้อความ order-based challenge binding (Alg + II-A) — done in smarmplus_main.tex + AUTHORS_response_to_reviewers.md
- [x] R1.2: เพิ่ม Between-block continuation invariant (+ PRNG/FPE + Appendix) — done
- [x] R1.3: เพิ่ม always-scan / no early exit (+ comment Algorithm 2); เช็คโค้ด FindKthZero — done (paper; firmware check optional before submit)
- [x] R1.4: ย่อหน้า key hierarchy + อัปเดต Alg 1–3; (แนะนำ) แก้ firmware ให้ตรง — done in paper (+ bib refs); firmware optional
- [ ] R1.5: rewrite Theorem A.1 (S3) แบบ hybrid/distinguisher
- [ ] R1.6: ขยาย Remark FAR + โยง Table I/II + limitations
- [ ] Highlight PDF ตามจุดที่แก้
- [ ] Clean manuscript สำหรับ resubmit

---

*อัปเดตจากบทสนทนา review SMARM+ IEEE Access — Reviewer 1 เท่านั้น*
