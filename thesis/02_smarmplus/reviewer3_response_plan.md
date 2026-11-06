# SMARM+ IEEE Access — Reviewer 3 (อ่านง่าย)

เอกสารนี้รวม **Reviewer 3** แบบละเอียด  
รูปแบบแต่ละข้อใกล้เคียง Reviewer 1/2:

1. Reviewer พูดอะไร (ภาษาไทยง่ายๆ + บางทีมีคำเดิม)  
2. ใน paper มีอะไรอยู่แล้ว / ช่องว่าง  
3. Reviewer ถูกแค่ไหน  
4. Response (อังกฤษ copy ส่งได้)  
5. Action / สิ่งที่ต้องทำ  
6. สรุปสั้นท้ายข้อ

---

## อ่านก่อนทำ — Reviewer 3 คืออะไร

Reviewer 3 **ยาวและเข้มที่สุด**  
ในฟอร์มเค้าให้คะแนนประมาณว่า:

- มีศักยภาพเป็น contribution ได้  
- แต่ตอนนี้ **technically sound = ไม่ผ่าน**  
- นำเสนอยังไม่ครบพอจะซัพพอร์ต claim หลัก  

โทนคล้าย checklist ยาว (บางคนสงสัยว่า AI ช่วยเขียน) แต่ IEEE Access ยังคาดให้คุณ **address** ประเด็นหลักในการ resubmit **ครั้งเดียว**

### อย่าทำทุกอย่างที่ลิสต์

ถ้าทำครบทุกอย่าง = เหมือนเขียน paper ใหม่ + ทดลองใหม่ทั้งก้อน  

แบ่งงานเป็น 3 กอง:

| กอง | ความหมาย | ตัวอย่าง |
|-----|----------|----------|
| **ต้องทำ** | กระทบว่า paper “sound” หรือ claim เกินจริง | เกม security, timing FindKthZero, แยก key, 1 รอบ vs 7 รอบ, ขนาด memory, ลดคำแรง |
| **ทำโดยเขียนจำกัดขอบเขต** | อธิบายว่าเราไม่เคลมสิ่งนั้น | DMA, secure boot, NMI |
| **ตอบว่าเกินรอบนี้ / future work** | ถูกในอุดมคติ แต่ทำไม่ไหวใน revise ครั้งเดียว | วัด RT ครบสูตร, หลายแอป, หลายบอร์ด |

หลายข้อ **ซ้ำ Reviewer 1 และ 2** — ใน response ให้อ้างว่าแก้ข้อนั้นไปแล้วด้วย

---

# หมวด 1 — Security analysis

---

## R3.1 — เกม MOI “เดา permutation ทั้งก้อน” อ่อนเกินไป

### Reviewer พูดอะไร (ง่ายๆ)

ตอนนี้ใน Appendix เกม MOI ตั้งเงื่อนไขชนะว่า:  
malware ต้อง **ทายลำดับการวัดทั้งก้อนถูกทุกบล็อก** (`σ` ทั้งชุด)

แต่ malware จริงๆ (roving / KFV) ไม่จำเป็นต้องรู้ทั้งก้อน  
แค่รู้ว่า **บล็อกถัดไปคืออะไร** หรือรู้ลำดับบางส่วน ก็อาจย้ายตัวหนีการวัดได้แล้ว  

ถ้าเกมยากเกินจริง → paper อาจบอกว่า “secure” ทั้งที่ยังรั่วข้อมูลที่มีประโยชน์ต่อการหนี

Reviewer เลยขอให้เปลี่ยนนิยามเกมให้ใกล้ attacker จริง:

- ทำนายบล็อกถัดไป (next-block)  
- รั่วลำดับบางส่วน (partial-order)  
- ผลต่อการหนีของ roving malware  

### ใน paper มีอะไรอยู่แล้ว

- มี Definition เกม MOI แบบทาย `σ` ทั้งก้อน  
- มีพูดใน Background ว่า 1 รอบหนีได้ประมาณ 37%, ทำ 7 รอบแล้วโอกาสหนีต่ำมาก  

### ช่องว่าง

เกมใน Appendix **ไม่ค่อยตรง** กับเรื่องที่เล่าในตัว paper (malware ย้ายทีละบล็อก)

### Reviewer พูดถูกแค่ไหน

| | |
|--|--|
| ถูกไหม | **ถูกมาก** |
| แค่ตอบโดยไม่แก้เกม พอไหม | **ไม่พอ** |
| ต้องรันบอร์ดใหม่ไหม | **ไม่** สำหรับข้อนี้ — เป็นงานเขียนนิยาม/proof |

### Response (copy-ready)

**Reviewer’s concern**  
Replace the exact full-permutation MOI game with a definition that captures next-block prediction, partial-order leakage, and resulting roving-malware evasion.

**Our response**  
We agree. The submitted MOI game treats the adversary as winning only if it outputs the entire measurement permutation. That winning condition is harder than the operational goal of KFV/roving malware in our threat model, which mainly needs to predict the next (or remaining) blocks well enough to relocate and evade detection. As a result, the full-permutation game can classify settings with harmful next-block or partial-order leakage as “secure,” which does not match the paper’s own security narrative.

**Action**  
In the revised manuscript we will redefine the appendix game around next-block / remaining-block prediction and connect it to SMARM’s evasion-probability discussion. Predicting the full permutation may remain as a stronger corollary, but it will not be the only winning condition. We will update the (S3) proofs to match the new game. No new board experiments are required for this definitional fix alone.

### สิ่งที่ต้องทำใน paper

1. แก้ Definition ของเกมใน Appendix  
2. ปรับ proof (S3) ให้ตามเกมใหม่  
3. ระวังคำว่า “preserves security guarantees” — อย่าใช้แรงเกินจนกว่าเกมใหม่จะเขียนครบ  

### สรุปข้อนี้

**ต้องแก้ใน paper (นิยาม + proof)**  
ไม่ใช่รันทดลอง FAR ใหม่

---

## R3.2 — Timing ต้องอยู่ใน proof / FindKthZero ต้องไม่รั่วเวลา

### Reviewer พูดอะไร (ง่ายๆ)

ในเกม security เองก็ให้ adversary ดูได้ว่า **แต่ละบล็อกจบแล้วเปิด interrupt เมื่อไหร่** (timestamp)

แต่ตอนพิสูจน์ เรานึกถึงแค่ “อ่าน Secure RAM ไม่ได้”  
ไม่ได้คุยว่า **เวลาที่ใช้เลือกบล็อกถัดไป** อาจบอกใบ้ได้ว่าเลือกบล็อกไหน

โดยเฉพาะ SMARM+PRNG:

- สุ่มเลข `r`  
- แล้วเดินดู bitmap หา “ช่องว่างชิ้นที่ r” (`FindKthZero`)  

ถ้าเจอแล้ว **หยุดทันที** → รอบที่สุ่มได้ตำแหน่งต้นๆ จะเร็ว ตำแหน่งท้ายๆ จะช้า  
attacker ดูเวลาได้ → เดาลำดับได้ → กระทบ S3

Reviewer เลยบอกว่าต้องอย่างใดอย่างหนึ่ง:

1. ทำให้ `FindKthZero` / การสุ่มเลขใช้เวลาคงที่ (constant-time) หรือ  
2. แสดงด้วยการวิเคราะห์/ทดลองว่า timing **ไม่ให้ข้อมูลที่มีประโยชน์**

### ซ้อนกับ Reviewer 1 ข้อ 3

เราตอบ R1.3 ไปแล้วว่า:  
implementation **สแกนครบทุกช่องเสมอ** แม้เจอแล้วก็วิ่งต่อ → เวลาไม่ขึ้นกับว่าเลือก index ไหน  

แต่ Access paper **ไม่ได้เขียน** → Reviewer 3 จึงยังทัก

### ใน paper มีอะไรอยู่แล้ว

- มี Algorithm 2 และคำว่า “scan เพื่อหา zero ตัวที่ r”  
- **ไม่มี** ย่อหน้า always-scan / timing side-channel ใน Access PDF  

### Reviewer พูดถูกแค่ไหน

| | |
|--|--|
| ถูกที่ paper ไม่ได้คุย timing ใน proof | **ถูก** |
| ถูกที่ design ของเรา leak แน่ๆ | **ไม่แน่** — ถ้า always-scan จริง จะไม่ leak แบบที่เค้ากลัว |
| สรุป | เป็นช่องว่างเอกสาร + ต้องใส่ timing ในคำอธิบายความปลอดภัย |

### Response (copy-ready)

**Reviewer’s concern**  
Incorporate timing observations into the proofs, and either implement `FindKthZero` and bounded random sampling in constant time or demonstrate that their timing reveals no useful information.

**Our response**  
We agree that timing belongs in the security discussion because the adversary model already allows observing timestamps when interrupts are re-enabled after each block. A naive early-exit `FindKthZero` would indeed make scan duration depend on the selected index and could threaten (S3).

Our SMARM+PRNG implementation does **not** terminate early: `FindKthZero` always scans all `n` bitmap entries, so its duration is independent of which remaining block is chosen (it may still depend on public `n`, derived from `|M|` and `B`). The submitted manuscript omitted this implementation choice and did not connect it to the timestamp-capable adversary. We will document the full-scan behavior, discuss bounded integer sampling (`RandInt`), and revise the (S3) argument so timing is treated explicitly rather than ignored.

**Action**  
1. Add the always-scan / no-early-exit explanation to Section V and Algorithm 2 (same as Reviewer 1 comment 3).  
2. Mention timing in the revised appendix argument for SMARM+PRNG.  
3. Optionally add a small timing check that per-iteration scan time does not vary with the selected index.  
No protocol redesign is required if the implementation already performs a full scan. We will verify the firmware matches this claim before resubmission.

### สิ่งที่ต้องทำ

| งาน | ทำไหม |
|-----|--------|
| เขียนใน paper เรื่อง always-scan | **ต้อง** |
| ใส่ timing ในคำอธิบาย/proof | **ต้อง** |
| เช็คโค้ดว่าสแกนครบจริง | **ควร** |
| วัดเวลาทดลองเสริม | ถ้าทำได้จะดี ไม่บังคับถ้าข้อความชัด |
| เปลี่ยน FAR ทั้งชุด | **ไม่** |

### สรุปข้อนี้

เหมือน R1.3 แต่ Reviewer 3 ย้ำว่าต้องโยงเข้า **proof/เกม** ด้วย  
ไม่ใช่แค่ประโยคใน implementation

---

## R3.3 — ต้องนิยาม PRNG / RandInt / KDF / FFX / Speck ให้ชัด

### Reviewer พูดอะไร (ง่ายๆ)

ตอนนี้ paper พูดถึงเครื่องมือคริปโตหลายอย่าง แต่ยังคลุมเครือ เช่น:

- PRNG คือตัวไหน  
- `RandInt` สุ่มเลขในช่วงยังไง (biased หรือไม่)  
- KDF แยกกุญแจยังไง  
- FFX ใช้ encoding แบบไหน จำนวนรอบ Speck กี่รอบ ความปลอดภัยระดับไหน  

Appendix ยังใช้คำไม่สม่ำเสมอ เช่น `Hash(chal,k)`, `SHA256(chal∥k)`, `SHA256(k,·)`, HMAC — และอ้าง [46] ซึ่งจริงๆ พูดเรื่อง HMAC/keyed hash แบบหนึ่ง ไม่ใช่ “เอา SHA-256 มาใช้ยังไงก็ได้”

### ซ้อนกับ

- Reviewer 1 ข้อ 4 (domain separation)  
- Reviewer 1 ข้อ 5 (proof)  

### Reviewer พูดถูกแค่ไหน

**ถูก** — ต้องเขียนให้ชัดเจนขึ้น  
ไม่จำเป็นต้อง invent crypto ใหม่

### Response (copy-ready)

**Reviewer’s concern**  
Define the PRNG, `RandInt` procedure, KDF, domain separation, FFX domain encoding, number of rounds, Speck parameters, and concrete security level. Fix inconsistent hash/HMAC notation and the use of Reference [46].

**Our response**  
We agree the cryptographic instantiations were underspecified and notationally inconsistent. In the revision we will add a concrete “Cryptographic instantiations” subsection that specifies: the PRNG; how `RandInt` samples integers in `{1,…,rem}`; the KDF and domain-separation labels used to derive attestation and order-generation keys; and the FFX-with-Speck parameter set used in SMARM+FPE (domain encoding over `[n]`, rounds / construction reference, and the intended PRP assumption). We will make appendix notation consistent and cite Reference [46] only for HMAC-style / KDF claims that it actually supports—not for an unspecified keyed use of raw SHA-256.

**Action**  
Paper/appendix clarification + align with Reviewer 1 key-separation edits. No FAR re-run required.

### สรุปข้อนี้

**ต้องเขียนเพิ่มใน paper**  
งานเอกสาร/นิยาม ไม่ใช่ทดลองบอร์ดใหญ่

---

## R3.4 — Authenticate challenge + พารามิเตอร์ + แยก key

### Reviewer พูดอะไร (ง่ายๆ)

อยากให้หลักฐาน attestation “ผูก” สิ่งเหล่านี้ชัดๆ:

- challenge  
- ว่าใช้ scheme ไหน (SMARM / PRNG / FPE)  
- ขนาดบล็อก `B`  
- จำนวนบล็อก `n`  
- ที่อยู่เริ่มต้นของ memory ที่วัด  
- ความยาว memory ที่วัด  

และให้แยกกุญแจ: กุญแจสร้างลำดับ คนละดอกกับกุญแจทำ HMAC

### ซ้อนกับ Reviewer 1 ข้อ 1 และ 4

เราเลือกทาง R1.1 แล้วว่า **chal ผูกผ่านลำดับ** ไม่เพิ่ม `HMAC(chal)` ในโค้ดก็ได้ แต่ต้องเขียนชัด  

สำหรับข้อนี้ Reviewer 3 ขอมากกว่า chal — ขอ metadata ทั้งชุด + แยก key

### Reviewer พูดถูกแค่ไหน

| | |
|--|--|
| แยก key | **ถูก — ควรทำ** (คู่ R1.4) |
| ผูก chal | มีทาง order-binding แล้ว — เขียนให้ชัด |
| ผูก metadata ทั้งชุด | ถูกว่าควรมีในคำอธิบาย verifier; จะใส่ใน HMAC ด้วยก็ได้ (งานน้อย) |

### Response (copy-ready)

**Reviewer’s concern**  
Explicitly authenticate the challenge, scheme identifier, block size, block count, memory base, and memory length, and derive separate keys for permutation generation and attestation.

**Our response**  
We agree that key separation and an unambiguous verified transcript should be stated clearly. The revision will introduce domain-separated keys for order generation and attestation MAC (see also Reviewer 1 comment 4). For challenge binding, SMARM already binds `chal` into the evidence via the measurement order; we will state that explicitly (Reviewer 1 comment 1). We will also specify which public parameters (`scheme id`, `B`, `n`, attested base/length) are part of the verifier’s reference computation so that evidence cannot be reinterpreted under a different memory layout or block size. Where helpful for clarity, the revised algorithm description will include these fields in the authenticated transcript.

**Action**  
1. Domain-separated keys in Algorithms 1–3 (+ firmware ให้ตรงถ้าทำใน paper)  
2. Clarification of order-based `chal` binding  
3. Explicit list of authenticated/public parameters used by the verifier  
Optional: add those fields into `HMAC` input (small code change; no FAR campaign)

### สรุปข้อนี้

**ต้องทำในข้อความ (+ แนะนำแยก key ในโค้ด)**  
ไม่ต้องทดลอง FAR ใหม่

---

# หมวด 2 — Platform / ของบนบอร์ด

---

## R3.5 — Exception, NMI, DMA, secure boot, key provision, NSC, ขอบเขต memory

### Reviewer พูดอะไร (ง่ายๆ)

เค้าอยากรู้รายละเอียดระบบจริง เช่น:

- ปิด/จัดการ interrupt หรือ exception ฝั่ง Secure และ Non-Secure ยังไง  
- NMI / fault เกิดแล้วระบบทำอะไร  
- DMA หรือ bus master อื่น แก้ memory ที่กำลังวัดได้ไหม  
- มี secure boot ไหม กุญแจใส่เข้าเครื่องยังไง  
- NSC ตรวจ argument ที่ Non-Secure ส่งเข้ามาไหม  
- ขอบเขต memory ที่ attest ตรึงไว้ยังไง  

### ใน paper มีอะไรอยู่แล้ว

Threat model ประมาณว่า:

- Non-Secure ถูกโจมตีได้หมด  
- Secure + TrustZone กันไว้  
- malware แบบ KFV ขัดจังหวะระหว่างบล็อกได้  

แต่ **ไม่ได้ลงลึก** DMA, boot, NMI ฯลฯ

### Reviewer พูดถูกแค่ไหน

| | |
|--|--|
| ควรอธิบาย/จำกัดขอบเขต | **ถูก** |
| ต้องทำครบแบบ production ก่อน publish | **เกิน** สำหรับ revise ครั้งเดียว |

ถ้าไม่เขียน limitations คนอ่านอาจคิดว่าเราเคลมป้องกันทุกอย่างแล้ว

### Response (copy-ready)

**Reviewer’s concern**  
Explain exception masking (Secure/Non-Secure), NMI/faults, DMA/bus masters, secure boot, key provisioning, NSC argument validation, and fixed attested boundaries.

**Our response**  
We agree these platform issues matter for real deployments and should not be left implicit. Our evaluated prototype and threat model focus on a software adversary controlling the Non-Secure world (including KFV interrupts between blocks), under TrustZone-M isolation assumptions stated in the appendix. We did **not** claim that the prototype is a full production TEE stack resistant to DMA/bus-master writes, physical attacks, or incomplete secure boot/provisioning.

**Action**  
We will add a dedicated “Platform assumptions and limitations” subsection that states, item by item, what the prototype does today and what is out of scope (including DMA needing platform controls beyond CPU SAU attribution, and secure boot/key provisioning as deployment requirements). Exact attested address bounds will be stated explicitly (see also the attested-region comment below). We will not expand this revision into a full multi-master / production-hardening implementation campaign.

### สิ่งที่ต้องทำใน paper

ย่อหน้าหรือ subsection ใหม่ ไล่ทีละหัวข้อสั้นๆ:

- ทำแล้ว / ไม่ทำ / อยู่นอก threat model  

### สรุปข้อนี้

**เขียน limitations ให้ชัด**  
ไม่ต้อง implement DMA/boot ทั้งก้อนในรอบนี้

---

# หมวด 3 — การทดลองกับ claim ความปลอดภัยหลายรอบ

---

## R3.6 — ทำไมพูดถึง 7 รอบ แต่ตารางผลเป็นรอบเดียว

### Reviewer พูดอะไร (ง่ายๆ)

ใน Background paper บอกแนว SMARM ว่า:

- วัดแบบสุ่ม **1 รอบเต็ม** malware ยังหนีได้ประมาณ **37%**  
- ถ้าทำซ้ำอิสระประมาณ **7 รอบ** โอกาสหนีต่ำมาก (&lt; 10⁻³)  

แต่ตอน Evaluation (Table I runtime, Table II energy) ดูเหมือนวัดแค่ **1 รอบ** (หนึ่ง shuffled pass)

Reviewer กลัวว่าคนอ่านจะคิดว่า:

> “ใช้เวลา ~490 ms (ตัวอย่างที่ B=1024) แล้วได้ความปลอดภัยแบบทำ 7 รอบแล้ว”

ซึ่ง **ไม่จริง** ถ้ายังทำแค่ 1 รอบ

### สำคัญมาก — FAR ไม่ใช่ประเด็นตรงนี้

หลายคนงงจุดนี้:

**FAR ที่ขนาดบล็อก B เดิม โดยหลักเหมือนเดิม** แม้ทำกี่รอบ  
เพราะ FAR ดูผลกระทบจากช่วง **ปิด interrupt ต่อบล็อก** ไม่ได้เปลี่ยนความหมายแค่เพราะทำซ้ำ T รอบ

Reviewer ข้อนี้ไม่ได้บอกว่า “FAR ต้องเปลี่ยน”  
เค้าบอกว่า **ต้นทุนรวม (เวลา/พลังงาน) กับระดับการตรวจจับที่เคลม** ต้องคุยกันให้ตรง

| สิ่งที่ Background คุย | สิ่งที่ตารางโชว์ |
|------------------------|-----------------|
| อาจต้อง ~7 รอบเพื่อความปลอดภัยสูง | ตัวเลขของ **1** รอบ |

ถ้าทำ 7 รอบอิสระ เวลา/พลังงานโดยประมาณ **≈ 7 เท่าของ 1 รอบ**  
FAR ยังพูดเรื่องเดิมได้

### Reviewer พูดถูกแค่ไหน

**ถูกมาก** เรื่อง claim ไม่ตรงกับตัวเลขที่โชว์  
**ไม่ถูก** ถ้าตีความว่าต้องไปวัด FAR ใหม่ทั้งหมด

### Response (copy-ready)

**Reviewer’s concern**  
Evaluate the number of independent shuffled traversals required for the stated malware-detection probability and include that complete configuration in the runtime and energy results.

**Our response**  
We agree there is a mismatch between the background discussion of multi-traversal detection (about 37% evasion for one pass; on the order of seven passes for very low evasion) and Section VI, which reports runtime and energy for a **single** shuffled traversal. Readers could incorrectly assume that the tabulated single-pass cost already buys the multi-traversal detection level.

We clarify that **FAR is orthogonal here**: repeating independent traversals does not change the FAR characterization of per-block interrupt-disable interference for a fixed `B`. The issue is aligning **security strength** (`T` traversals) with **end-to-end cost**.

**Action**  
In the revision we will do one of the following:

**(A — recommended, little/no new board work)**  
Keep Tables I/II as **per-traversal (T=1)** cost, and explicitly state that achieving the cited multi-traversal detection level with `T` independent passes costs approximately `T×` that runtime and energy. We will illustrate `T=1` vs `T=7` numerically at the recommended operating point (e.g. `B=1024`).

**(B)**  
Actually run a `T=7` configuration and report measured totals.

We will also align the abstract/conclusion so they do not imply that single-pass experiments already achieve the multi-traversal evasion bound. No FAR re-campaign is required for this item.

### สิ่งที่ต้องทำ

| ทาง | รายละเอียด |
|-----|------------|
| **ทางสั้น (แนะนำ)** | เขียนให้ชัดว่าตาราง = 1 รอบ; ความปลอดภัยสูงต้องประมาณ T× ต้นทุน; ตัวอย่างคูณ 7 ที่ B=1024 |
| **ทางยาว** | รัน T=7 จริงแล้วใส่ตาราง |
| FAR ทั้งชุด | **ไม่ต้องทำใหม่เพราะข้อนี้** |

### สรุปข้อนี้

**ต้องแก้ความเข้าใจ claim ↔ ตัวเลข**  
ไม่ต้องคิดว่า “รันเพิ่มแล้ว FAR จะเปลี่ยน”

---

# หมวด 4 — Real-time evaluation

---

## R3.7 — ให้มี metric RT มากกว่า FAR

### Reviewer พูดอะไร (ง่ายๆ)

FAR ดูแค่ความถี่เฉลี่ยของ task  
ไม่ได้บอกว่า:

- ตอบสนองช้าสุดเท่าไร  
- jitter  
- พลาด deadline ไหม  
- blocking แย่สุดเท่าไร  
- วิเคราะห์ schedulability แบบตำรา RT หรือไม่  

และ paper เองก็ยอมรับว่า FAR ไม่แทน schedulability แต่ยังใช้ FAR ไปซัพพอร์ตคำว่า real-time compatibility และจุดแนะนำ B=1024

### ซ้อนกับ Reviewer 1 ข้อ 6

### Reviewer พูดถูกแค่ไหน

| | |
|--|--|
| FAR แคบ | **ถูก** |
| ต้องทำ metric ครบทุกตัวในลิสต์ก่อน publish | **เกิน** สำหรับรอบเดียว |

### Response (copy-ready)

**Reviewer’s concern**  
Supplement or replace FAR with response-time, jitter, interrupt latency, deadline-miss ratio, worst-case blocking, and schedulability analysis; do not let FAR alone support broad real-time claims.

**Our response**  
We agree FAR is a narrow frequency-level proxy for interference from interrupt-disabled block measurements, not a full real-time certification metric. Section IV-C already notes that FAR does not replace schedulability analysis; however, some wording around “real-time compatibility” and the recommended `B=1024` operating point could be read as stronger than what FAR justifies.

As also discussed for Reviewer 1 comment 6, we already report end-to-end attestation runtime and energy as complementary measures of total attestation work. In this revision we will (1) narrow real-time claims to what FAR actually measures, (2) clarify the division of metrics (FAR vs runtime/energy), and (3) add an explicit limitations paragraph. A complete multi-metric response-time and schedulability study is valuable future work; requiring it in full would turn this Access revision into a substantially new experimental paper.

**Action**  
Clarify + soften claims + limitations (ร่วม R1.6).  
Optional small extras only if data already exist.  
Do **not** promise a full classical RT suite in this resubmission.

### สรุปข้อนี้

**ต้องลดน้ำเสียง + เขียน limitations**  
ไม่ต้องรับทำ RT lab ใหม่ทั้งห้อง

---

## R3.8 — ระบุพารามิเตอร์ของ task ให้ครบ

### Reviewer พูดอะไร (ง่ายๆ)

ตอนนี้บอกแค่ task รัน 10 / 100 / 1000 Hz  
ยังไม่บอก เช่น:

- งานแต่ละครั้งกินเวลาเท่าไร (WCET)  
- deadline  
- priority  
- utilization  
- ปล่อยงานยังไง  
- ทดลองนานแค่ไหน  

### Reviewer พูดถูกแค่ไหน

**ถูก** ว่าควรมีตารางพารามิเตอร์  
ไม่จำเป็นต้องมีทฤษฎี WCET ลึก ถ้า task เป็นตัวสังเคราะห์ที่เรากำหนดเอง — แต่ต้อง **บอกของจริงที่ใช้**

### Response / Action

เพิ่มตารางสั้นๆ ใน Section IV เช่น:

- period = 1/frequency  
- priority สูงกว่า SMARM wrapper  
- deadline (ถ้าตั้งเท่า period ให้บอก)  
- วัดอย่างไร นานกี่วินาที/กี่รอบ attestation  
- ว่าเป็น synthetic task ไม่ใช่แอปโรงงานจริง  

**ต้องทำใน paper** — โดยทั่วไปไม่ต้องทดลองใหม่ถ้าค่าเหล่านี้มีใน setup อยู่แล้ว แค่ไม่เคยเขียน

---

## R3.9 — ขอทดลอง RT กว้างมาก (หลาย task, แอปจริง, สอง RTOS, ฯลฯ)

### Reviewer พูดอะไร (ง่ายๆ)

อยากให้มี:

- หลายชุด task  
- อย่างน้อยหนึ่งแอป real-time จริง  
- phase การปล่อยงานต่างกัน  
- วัดเวลาจบ attestation ตอนโดนแย่ง CPU  
- ทดลองทั้ง SMARM+PRNG และ FPE บนทั้ง FreeRTOS และ Zephyr  
- หลายบอร์ด  

### Reviewer พูดถูกแค่ไหน

ถูกในฐานะ “งานวิจัย RT สมบูรณ์แบบ”  
**เกินจริง** สำหรับ IEEE Access revise ครั้งเดียว  
paper ตั้งใจใช้ 1 task เป็น conservative stress อยู่แล้ว — แต่ต้องยอมรับ limitation

### Response (copy-ready)

**Our response**  
Our single high-priority real-time task was intentional: it is a conservative stress test to expose when interrupt-disabled block measurement breaks frequency accuracy. We will explain that scope more clearly and state what it does *not* cover (multi-task sets, application case studies, phase sweeps, and dual-RTOS×dual-variant schedulability matrices). Expanding to that full campaign is future work rather than a feasible complete rewrite of the evaluation in one Access resubmission.

**Action**  
Rebut + limitations + future work.  
Optional: only add experiments you already have or can finish quickly—not the whole list.

### สรุปข้อนี้

**ตอบว่าขอบเขตแคบ + future work**  
ไม่รับลิสต์ทดลองทั้งหมดเป็นเงื่อนไขเดียว

---

## R3.10 — วิธีวัดพลังงาน / สถิติยังไม่ละเอียด

### Reviewer พูดอะไร (ง่ายๆ)

อยากให้บอก:

- ทำซ้ำกี่ครั้ง  
- ช่วงเชื่อมั่น (CI)  
- มิเตอร์แม่นแค่ไหน เทียบมาตรฐานยังไง  
- ต่อวงจรยังไง  
- นับพลังงานตอน idle อย่างไร  

### ใน paper มีอะไรอยู่แล้ว

มีบอกใช้ FNIRSI FNB58 สุ่ม 100 Hz  
บางกราฟบอกเฉลี่ยหลายรัน  

### Reviewer พูดถูกแค่ไหน

**ถูก** ว่าควรละเอียดขึ้น

### Action

ขยายย่อหน้า measurement methodology ให้ครบเท่าที่ทำได้จริงจากของที่มี  
ไม่ต้องซื้อเครื่องมือใหม่ถ้าอธิบาย limitation ของมิเตอร์ได้ตรงๆ

---

# หมวด 5 — Memory / ตัวเลข storage / รูป

---

## R3.11 — ขนาด memory ที่ attest สับสนกับ Secure flash ครึ่งหนึ่ง

### Reviewer พูดอะไร (ง่ายๆ)

บนบอร์ด ระบุว่า flash 512 KB แล้วแบ่ง Secure ประมาณครึ่ง (256 KB)  

แต่ตัวเลขในส่วน storage / จำนวนบล็อก ดูเหมือนไปทางวัด memory รวมประมาณ **512 KB**  

เลยงงว่า:

- วัดของ Non-Secure ทั้งก้อน?  
- วัดรวม Secure?  
- ตัวเลขในตารางคิดจากอะไร  

### ซ้อนกับ Reviewer 2 ข้อ 3

### Reviewer พูดถูกแค่ไหน

**ถูก** — ข้อความตอนนี้ทำให้สับสนได้ง่าย

### Response / Action

เพิ่มแผนที่ความจำสั้นๆ:

1. Secure flash/RAM ใช้เก็บโค้ด Secure + ของลับ  
2. ช่วง Non-Secure ที่นำมา attest คือ address ไหน ยาวเท่าไร (`|M|`)  
3. ความสัมพันธ์ `n = |M| / B`  
4. ตัวเลขใน Table I/II/Fig.9 อิง `|M|` นี้  

**ยืนยันจาก firmware ก่อนใส่ตัวเลข**  
ไม่ต้องรัน FAR ใหม่ถ้าแค่ทำให้ข้อความตรงของจริง

---

## R3.12 — ขอเลข peak Secure RAM/flash แยกชิ้น

### Reviewer พูดอะไร (ง่ายๆ)

อย่าบอกแค่ “storage O(n)”  
อยากเห็นของจริงคร่าวๆ เช่น:

- bitmap กี่ไบต์  
- state ของ PRNG / FFX  
- key / tweak  
- HMAC state  
- stack  
- โค้ด Secure + library  

### Reviewer พูดถูกแค่ไหน

**ถูก** ว่าตัวเลขตอนนี้ฟังใหญ่เกินความจริงถ้าคนคิดว่าเป็น footprint ทั้ง Secure world

### Action

ตาราง breakdown (จาก `sizeof` / map file ก็ได้)  
แยก “metadata สำหรับลำดับ” กับ “TCB ทั้ง Secure service”

---

## R3.13 — ชื่อ Fig.9

### Reviewer พูดอะไร (ง่ายๆ)

ถ้า Fig.9 โชว์แค่ของที่เกี่ยวกับ permutation/bitmap ไม่ใช่การใช้ Secure ทั้งหมด  
อย่าตั้งชื่อแบบ total secure storage

### Action

แก้ caption / แกน Y ให้ตรงความจริง เช่น  
“Secure storage for measurement-order metadata”

งานเล็ก ทำได้ทันที

---

# หมวด 6 — Artifacts, claims, references

---

## R3.14 — ขอปล่อยโค้ดและข้อมูลทดลอง

### Reviewer พูดอะไร (ง่ายๆ)

อยากให้มี artifact สาธารณะ เพื่อทำซ้ำได้

### Reviewer พูดถูกแค่ไหน

ถูกในทาง reproducibility

### Action

- วาง repo หรือ archive (GitHub/Zenodo ฯลฯ)  
- ใส่ลิงก์ใน paper  
- ถ้าปล่อยไม่ได้ทั้งหมด ให้บอกว่าได้อะไร / ไม่ได้อะไร  

---

## R3.15 — ลดคำแรงใน claim

### Reviewer พูดอะไร (ง่ายๆ)

คำพวกนี้แรงเกินจนกว่าหลักฐานจะรองรับ:

- “first systematic study”  
- “eliminates secure storage” (FPE ยังมี key material อยู่)  
- “preserves security guarantees” (ทั้งที่เกม/ timing / platform ยังมีช่องว่าง)

### Action

ค้นในต้นฉบับแล้วปรับให้นุ่มลง  
ตรวจ metadata ของเทมเพลต IEEE Access

---

## R3.16 — References (จากฟอร์มข้อ 4–5a)

### Reviewer พูดอะไร (ง่ายๆ)

- References โดยรวมเกี่ยวข้อง แต่ยังไม่พอในบางหัวข้อ (RT analysis, Armv8-M, TF-M, DMA, constant-time, FPE ฯลฯ)  
- มีบางอันไม่เหมาะสมสำหรับ claim ที่ใช้อยู่  

### รายการที่เค้าชี้เป็นพิเศษ

| อ้างอิง | ทำอย่างไร |
|---------|-----------|
| **[1], [2], [8]** | ลบ หรือแทนด้วยแหล่งหลัก/งาน peer-reviewed หรือตัดประโยค market claim |
| **[14]** | ใช้เป็นตัวอย่างอุปกรณ์ได้ — อย่าใช้ยืนยัน security ทั่วไป |
| **[46]** | **อย่าลบ** — ใช้เฉพาะกับ HMAC/KDF ที่นิยามชัด |
| อื่นๆ | ตรวจว่า related work อ้างถูกเรื่อง |

### Action

แก้ bib + ประโยคในบทนำ/related work  
เพิ่ม ref เฉพาะที่คุณจะเขียนรองรับจริง ไม่ต้องใส่ทุกหัวข้อที่เค้าลิสต์แบบ checklist

---

# ฟอร์ม Additional Questions ของ Reviewer 3 แปลว่าอะไร

อันนี้เป็นคะแนนรวม ไม่ใช่โจทย์แยกใหม่ทั้งก้อน:

| คำถาม | คำตอบเค้า | แปลสั้นๆ |
|-------|-----------|----------|
| มี contribution ไหม | มีศักยภาพ แต่ novelty หลักคือ integrate/eval | ไม่ต้องเถียงว่า invent PRNG/FPE ใหม่ |
| Technically sound? | **ไม่** ในรูปปัจจุบัน | ต้องแก้แกน security + อย่าเคลม RT เกิน FAR |
| Comprehensive? | โครงสร้างดี แต่ยังไม่ครบพอซัพพอร์ต claim | ต้องเพิ่ม limitations / รายละเอียด |
| Refs พอไหม | เกี่ยวข้องแต่ไม่พอ | เติมเฉพาะที่จำเป็น |
| Refs ไม่เหมาะสม? | **มี** | แก้ [1][2][8][14][46] ตามตารางด้านบน |

ใน Author’s Response ไม่ต้องตอบทีละช่องฟอร์มแบบข้อสอบ  
ให้ตอบตามหัวข้อเทคนิคด้านบน แล้วสรุปท้ายว่า revisions address the soundness/comprehensiveness concerns

---

# สรุปภาพรวม — ทำอะไรก่อนหลัง

## ควรทำแน่ (Must)

1. แก้เกม S3 / MOI (R3.1)  
2. เขียน timing + always-scan แล้วโยง proof (R3.2 / R1.3)  
3. นิยาม crypto + แยก key + โน้ต Hash/HMAC ให้สม่ำเสมอ (R3.3–R3.4 / R1.4–R1.5)  
4. จัด claim 1 รอบ vs 7 รอบ ให้ตรงตาราง — คูณ T× หรือวัด T=7 (R3.6) **ไม่ใช่เรื่อง FAR**  
5. อธิบาย `|M|` / แผนที่ memory (R3.11 / R2.3)  
6. แก้ชื่อ Fig.9 + ตาราง storage แยกชิ้น (R3.12–R3.13)  
7. ลดคำแรง (R3.15)  
8. แก้ refs ตาม B5a (R3.16)  
9. ลด claim จาก FAR + ใส่ limitations (R3.7 / R1.6)  
10. ตารางพารามิเตอร์ task (R3.8)  

## ควรทำ (Should)

- Platform limitations: DMA/boot/NMI/NSC (R3.5)  
- วิธีวัดพลังงานละเอียดขึ้น (R3.10)  
- ปล่อย artifact (R3.14)  
- เพิ่ม references ที่จำเป็นจริง  

## ตอบว่ายังไม่ทำในรอบนี้ (Rebut / later)

- ชุด metric RT ครบสูตรทั้งก้อน  
- หลายแอป / หลายเฟส / หลายบอร์ด / matrix RTOS×variant ทั้งชุด (R3.9)  

---

## ประโยคเปิด Response ถึง Reviewer 3 (แนะนำ)

> We thank Reviewer 3 for the detailed comments. We agree that the security definition, timing discussion, cryptographic concreteness, alignment between multi-traversal detection claims and single-pass cost tables, memory accounting, and several over-strong claims require revision. Several other requests expand into a substantially new real-time experimental study. In this resubmission we correct the security analysis and claim/evaluation mismatches, clarify platform and metric limitations, and strengthen methodology and artifacts, while deferring a full multi-task schedulability campaign to future work.

---

## Checklist รวม

- [ ] R3.1 เกม S3 ใหม่  
- [ ] R3.2 timing + FindKthZero ใน paper/proof (+ เช็คโค้ด)  
- [ ] R3.3 crypto instantiations  
- [ ] R3.4 แยก key + metadata/chal clarification  
- [ ] R3.5 platform limitations  
- [ ] R3.6 single-pass vs T× / multi-traversal (ไม่ใช่ FAR)  
- [ ] R3.7–R3.8 ลด FAR claim + ตาราง task  
- [ ] R3.9 rebut + future work  
- [ ] R3.10 methodology พลังงาน  
- [ ] R3.11 แผนที่ memory / \|M\|  
- [ ] R3.12–R3.13 storage breakdown + Fig.9  
- [ ] R3.14 artifacts  
- [ ] R3.15 soften claims  
- [ ] R3.16 แก้ refs  

---

ใช้คู่กับ:

- `reviewer1_response_plan.md`  
- `reviewer2_response_plan.md`
