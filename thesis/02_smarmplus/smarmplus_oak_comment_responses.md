# SMARM+ — อ่านคอมเมนต์อาจารย์ (OAK) + ทำไมเราเขียนแบบนั้น + ร่างคำตอบ

**ไฟล์ที่อ่าน:** `smarmplus-comment.tex`  
**บริบท:** ข้อความสีน้ำเงิน `\ben{...}` ส่วนใหญ่มาจากรอบแก้ตาม **IEEE Access reviewers (โดยเฉพาะ R3)** ไม่ใช่เขียนยาวเองเล่นๆ  
**เป้าของเอกสารนี้:** ให้อ่านก่อนแก้ — แต่ละจุดมี (1) อาจารย์ว่าอะไร (2) เราเขียนทำไม (3) ควรตอบ/ทำอย่างไร  
**สถานะ:** ยังไม่แก้ tex ในรอบนี้ — อ่านก่อนตามที่คุยกัน

---

## สรุปสั้นๆ ก่อนอ่านทีละข้อ

| แนวคอมเมนต์อาจารย์ | ความหมายโดยรวม | คำแนะนำเบื้องต้น |
|---|---|---|
| delete / boring / appendix | ยาวเกินสำหรับ paper; รายละเอียด platform/crypto ควรย้ายหรือตัด | **ส่วนใหญ่ยอม** — ย้าย appendix หรือเหลือ 1–2 ประโยค + cite |
| อย่าผสม paper อื่น / อย่าใส่ S1–S3 ใน intro | โฟกัส narrative ของ SMARM+ | **ยอมได้** โดยไม่เสีย security claim ใน section หลัก |
| ต้องมี real-time task จริง / deadline-miss | อยากได้ metric แบบ RT จริง ไม่ใช่แค่ FAR | **ไม่โต้แบบปิดประตู** — อธิบายว่า FAR+RTA มาตอบ R3 แล้ว และ Option A (sensor sim) ที่ตกลงไว้คือทางออก |
| ทำไมเขียน / WTH | ไม่เห็นเหตุผลในตัว paper | ตอบว่า **Reviewer ขอ** — แล้วเสนอย่อ/ย้ายตามที่อาจารย์อยาก |

**หลักการตอบอาจารย์:**  
- ไม่เถียงยาวใน paper ว่า “reviewer บังคับ”  
- แต่ใน chat/meeting อธิบายสั้นๆ ว่ามาจากข้อไหนของ R3  
- ถ้าอาจารย์บอกลบ/ย้าย appendix → **ทำตาม** แล้วเก็บสาระที่ reviewer ต้องการไว้ที่สั้นกว่าเดิม หรือใน appendix/response letter

---

## 1. Introduction — botnet citations

**OAK:** *Mirai is old now. Cite more recent botnets*

**ทำไมเขียน:** ประโยคเปิดเรื่อง IoT ถูกโจมตี — cite Mirai เป็นตัวอย่างคลาสสิก (มีอยู่เดิม/รอบ revision)

**ตอบอาจารย์ (ร่าง):**  
Agree. We will replace/add more recent botnet references (e.g., recent IoT botnet campaigns / reports) and keep Mirai only if needed as historical context, or drop it.

**Action:** หา cite ใหม่ 2–3 ตัว แล้วอัปเดต `\cite{...}`

---

## 2. Contributions — “(S1)–(S3)” ในประโยค challenge

**OAK:** *dont put (S1)--(S3) here. It doesnt contribute meaningfully here*

**ข้อความเดิม (แนว):** *satisfy SMARM's security requirements (S1)--(S3) without relying on a formally verified microkernel...*

**ทำไมเขียน:** R3.15 / การลด overclaim — เปลี่ยนจาก “preserve security” แบบคลุมเครือ เป็นอ้าง (S1)–(S3) ให้ชัดว่าเราไม่ได้ invent security model ใหม่

**ตอบอาจารย์ (ร่าง):**  
Agree that listing (S1)–(S3) in the contribution bullet is unnecessary. We will rephrase to “satisfy the same security requirements as SMARM” (or similar) and keep the numbered properties only in Background / Security sections where they are defined.

**Action:** ตัด `(S1)--(S3)` ออกจาก contribution/challenge sentence

---

## 3. Contributions — “established techniques (bitmap / FPE)”

**OAK:** *this is too much detailed to be mentioned here. the paper is already too long.*

**ทำไมเขียน:** ตอบ R3 Additional Question เรื่อง novelty — ยอมรับว่าเทคนิคพื้นฐานไม่ใหม่ แต่ contribution คือการเอามาใช้กับ shuffled RA บน TrustZone-M + ประเมินต้นทุน

**ตอบอาจารย์ (ร่าง):**  
Agree to shorten. One short clause is enough, e.g. “using standard sampling-without-replacement and FPE building blocks.” Detailed novelty positioning can stay in Related Work / Discussion if needed.

**Action:** ตัดประโยคยาวออกจาก contributions

---

## 4. Background TrustZone-M — ประโยคซ้ำ

**OAK:** *this sentence largely overlaps with the first sentence.*

**ทำไมเขียน:** เติม citation Armv8-M / STM32L552 ให้ชัด (reviewer / clarity)

**ตอบอาจารย์ (ร่าง):**  
Agree — merge into one sentence and delete the duplicate.

**Action:** รวมประโยค / ลบตัวซ้ำ

---

## 5. TrustZone misconfiguration paragraph

**OAK:** *I dont understand this sentence. Why mentioning here? We need the same assumption, i.e., no TrustZone-M misconfiguration.*

**ข้อความเดิม:** อ้างงานที่โชว์ว่า misconfig ทำลาย isolation แล้วตามด้วย assumption ว่าเรา config ถูก

**ทำไมเขียน:** R3.5 (platform security) — อยากแสดงว่าเรารู้เรื่อง misconfig ในโลกจริง แต่สมมติว่า deployment config ถูก

**ตอบอาจารย์ (ร่าง):**  
Agree. Citing misconfiguration exploits here is confusing next to our trust assumption. We will state the assumption directly: TrustZone-M is correctly configured (no NS access to Secure memory/peripherals), and move any misconfiguration discussion out of this spot (or drop it).

**Action:** ลบ/ย่อเหลือ assumption สั้นๆ หนึ่งประโยค

---

## 6. Threat model — ควรใส่ secure boot / DMA / provisioning ที่นี่

**OAK:** *We should put secure boot, DMA protection, provisioning here.*

**และถัดไป OAK:** *delete this.* (บล็อก `\ben{...}` ยาวที่อธิบาย physical OOS + secure boot + GTZC + ชี้ไป `platform-config`)

**ทำไมเขียนบล็อกยาว:** ตอบ **R3.5 / R3.6** — reviewer ขอรายละเอียด exceptions/NMI/DMA/secure boot/key provisioning/NSC checks จึงเขียน subsection `Platform Security Configuration` และย่อใน threat model

**ความขัดแย้งในคอมเมนต์อาจารย์:**  
- จุดหนึ่งบอกให้ **ใส่** secure boot/DMA/provisioning ใน threat model  
- จุดถัดไปบอก **delete** บล็อกยาวที่พูดเรื่องนั้น + ชี้ไป platform-config

**ตีความที่สมเหตุสมผล:** อยากได้ **assumption สั้นใน threat model** (secure boot, DMA isolation, provisioning) แต่ **ไม่เอา prose ยาวแบบ report** — รายละเอียดย้าย appendix

**ตอบอาจารย์ (ร่าง):**  
Agree with both intents: we will keep a short threat-model assumption list (secure boot; Secure-only key provisioning; DMA/bus-master isolation, e.g. GTZC) and move the long platform checklist to the appendix (or a short appendix subsection), referenced from the threat model. We will delete the verbose threat-model paragraph that currently duplicates that material.

**Action:**  
1. Threat model: 3–5 bullet assumptions สั้น  
2. `sec:platform-config` → appendix (หรือตัดเหลือตารางสั้น)  
3. ลบ `\ben` บล็อกยาวที่อาจารย์บอก delete

---

## 7. Key hierarchy / domain separation

**OAK:** *below should be integrated into the component. It should be brief. It's a boring detail.*

**ทำไมเขียน:** **R3.4** — reviewer ขอ key separation / อย่าใช้ master key หลายหน้าที่; เราใส่ KDF labels `0‖k` / `1‖k`

**ตอบอาจารย์ (ร่าง):**  
Agree. We will fold one short sentence into the Secure Service description (e.g., “$k$ is never used directly; we derive $k_{\mathrm{prp}}$ and $k_{\mathrm{mac}}$ via a labeled KDF”) and move the formulas/details to the appendix / cryptographic instantiation.

**Action:** ย่อเหลือ 1 ประโยคใน Components; รายละเอียด → appendix

---

## 8. Authenticated attestation context (HMAC header)

**OAK:** *WTH is this below?*

**ทำไมเขียน:** **R3.4 / R3.5** — reviewer ขอให้ public metadata (scheme, B, n, base, |M|, chal, …) อยู่ในสิ่งที่ verifier คำนวณ; เราเลือกผูกเป็น `hdr` ใน HMAC

**ตอบอาจารย์ (ร่าง):**  
This paragraph was added to answer the Access review request that attestation metadata be bound into the verifier’s computation (so evidence under one scheme/layout cannot be accepted under another). We agree it is too long in the main text. Proposal: keep one sentence under Secure Service (“HMAC absorbs a fixed header binding scheme id, region parameters, $T$, and $chal$”) and move the byte layout to the appendix / algorithm caption.

**Action:** อย่าลบสาระ — **ย่อแรงมาก** หรือย้าย appendix

---

## 9. Multiple traversals paragraph (ใน Sec. III)

**OAK:** *just assume verifier sends one challenge per traversal... Delete it*

**ทำไมเขียน:** **R3.6** — reviewer ถามเรื่อง $T$ traversals กับ detection probability; เราเขียนวิธี derive seed ต่อ traversal + บอกว่า storage/blocking ไม่เปลี่ยน

**ตอบอาจารย์ (ร่าง):**  
We added this because a reviewer asked how $T>1$ is realized and how it relates to escape probability. If preferred, we can delete the mid-section paragraph and keep only the later dedicated subsection (or a short note: “for $T>1$, each traversal uses an independent order derived from $(chal,t)$; cost scales linearly”). Alternatively, follow your suggestion and assume one challenge per traversal in the main text.

**หมายเหตุภายใน (อย่าส่งอาจารย์ยาว):**  
สมมติ “one challenge per traversal” อาจง่ายกว่าใน main text แต่ต้องให้สอดคล้องกับ appendix/algorithms และตาราง $T=7$ ที่มีอยู่แล้ว — อย่าลบ `sec:traversals` ทั้งก้อนโดยไม่เช็ก response letter

**Action:** ลบ/ย่อ paragraph ใน Sec. III; คงสาระเรื่อง $T$ ไว้ที่ `sec:traversals` หรือ appendix

---

## 10. Between-block continuation invariant

**OAK:** *below shouldnt exist. We should discuss it as part of security analysis.*

**ทำไมเขียน:** **R1.2** — reviewer ขอความชัดเรื่อง state ระหว่างบล็อกเมื่อเปิด interrupt

**ตอบอาจารย์ (ร่าง):**  
Agree on placement. We will move this into the Security subsection (or appendix proof) instead of a standalone design paragraph, and keep only a one-line pointer if needed.

**Action:** ย้ายไป Security analysis / appendix — ไม่ต้องมีเป็น subsection กลาง Design

---

## 11. Platform Security Configuration (ทั้งก้อน)

**OAK:** *this should be in appendix and referred to by the threat model section. Also, we are not writing a report; we can safely exclude boring details.*

**ทำไมเขียน:** **R3.5** โดยตรง — PRIMASK, NMI, GTZC, secure boot, cmse checks, TF-M ฯลฯ

**ตอบอาจารย์ (ร่าง):**  
Agree. We will move this subsection to the appendix and cite it from the threat-model assumptions. We can also trim items that are pure engineering checklist if you want the appendix shorter.

**Action:** ย้าย appendix + อ้างจาก threat model (สอดคล้องข้อ 6)

---

## 12. Workflow figure comment (comment ใน tex)

**OAK (comment):** *Loop is confusing. Just write SMARM logic (Algorithm 1)*

**ทำไมมีรูป workflow:** อธิบาย end-to-end verifier ↔ wrapper ↔ Secure Service

**ตอบอาจารย์ (ร่าง):**  
We can simplify the figure/caption to match Algorithm 1 (challenge → NSC → per-block measure with IRQ windows → evidence), or drop redundant loop visuals if the algorithm already covers it.

**Action:** ตรวจรูป `smarm_workflow.pdf` — ทำให้ง่ายขึ้นตามที่อาจารย์บอก

---

## 13. Implementation — GTZC/watermark “boring sentence”

**OAK:** *boring sentence.*

**ทำไมเขียน:** แก้ตัวเลข Secure SRAM **96KB** (เคยผิดเป็น 128KB) และบอก mechanism ของการ split — มาจาก R3.5 / ความถูกต้องของ platform description

**ตอบอาจารย์ (ร่าง):**  
Agree to shorten to: “256 KiB flash and 96 KiB SRAM are allocated to the Secure world.” Drop option-byte/MPCBB addresses from the main text (keep in appendix if needed).

**Action:** ตัด technical clause ทิ้งจาก main text

---

## 14. Cryptographic instantiation

**OAK:** *appendix!*

**ทำไมเขียน:** **R3.3** — ระบุ PRNG / RandInt / KDF / Fisher–Yates / MAC ให้ครบ

**ตอบอาจารย์ (ร่าง):**  
Agree — move the cryptographic instantiation bullet list to the appendix and keep a one-sentence pointer in Implementation (“crypto primitives are instantiated as in Appendix X”).

**Action:** ย้าย appendix

---

## 15. Fig. t_disabled caption — FreeRTOS/TF-M

**OAK:** *why mention here?*  
**(ใน tex มี BEN: DELETE IT …)**

**ทำไมเขียน:** R3.16 follow-up — บอกว่า FreeRTOS = bare-metal Secure, Zephyr = TF-M เพื่อไม่ให้คนอ่านคิดว่า Secure stack เหมือนกันทั้งคู่

**ตอบอาจารย์ (ร่าง):**  
Agree — caption should stay about $t_{\mathrm{disabled}}$ measurement. Secure-runtime difference belongs in Implementation (one sentence), not in this figure caption.

**Action:** ลบประโยคนั้นออกจาก caption (ตามที่ BEN เองก็บอก DELETE IT)

---

## 16. FAR setup — real-world real-time task

**OAK:** *we should put real-world real-time task here.*

**ทำไมเขียนแบบ synthetic $T_{RT}$:**  
- ฉบับส่งใช้ FAR + counter task  
- R3 ขอ classical RT metrics / broader campaign  
- อาจารย์เคยเลือก **Option A**: simulate sensor readings + brake-style parse task (`HANDOFF` / `rt_experiment_spec.md`) — **ยังเป็นงานทดลองที่ค้าง** ไม่ใช่แค่แก้ประโยค

**ตอบอาจารย์ (ร่าง):**  
Agree on the direction. The current FAR harness uses a minimal periodic task to isolate attestation interference (as reported in the submitted evaluation). For the revision we already planned a more realistic workload (Option A: timer-interrupt “sensor” + periodic consumer task) and will replace/augment this setup accordingly. Until that experiment is ready, we should not overclaim “real-world task” in text-only edits.

**Action:** นี่คือ **งานทดลอง** ไม่ใช่แค่ลบประโยค — ต้องทำ Option A แล้วอัปเดต Sec. Evaluation

---

## 17. “whats P?”

**OAK:** *whats P?*

**ทำไมเขียน:** ตาราง/พารากราฟ FAR ใช้ $P = 1/f$ เป็น period — แต่ใน prose มีบรรทัดที่เขียนไม่เป็น math mode ชัด (`(T_{RT}) is a periodic...`) ทำให้สับสน

**ตอบอาจารย์ (ร่าง):**  
$P$ is the period of $T_{RT}$ ($P=1/f$). We will define it explicitly at first use and fix the typesetting so it is obvious.

**Action:** นิยาม $P$ ชัด + แก้ formatting

---

## 18. FAR remark — ควรแทนด้วย deadline-miss / jitter / response time

**OAK:** *we should replace this. We need to test it against real-time metrics... response time may not matter [link]...*

**ทำไมเขียน FAR + remark + RTA blocking:**  
ตอบ **R1.6 / R3.7 / R3.8** — reviewer บอก FAR อย่างเดียวไม่พอ; เราเพิ่ม  
- ยอมว่า FAR ≠ schedulability  
- ใส่ blocking term ใน RTA: $R_i = C_i + t_{\mathrm{disabled}}(B) + \sum \ldots$  
- ยังไม่ได้รัน deadline-miss campaign เต็มรูปแบบ (และอาจารย์เองก็เคยบอกว่า response time อาจไม่ใช่จุดหลัก)

**ตอบอาจารย์ (ร่าง) — ตรงๆ แต่มืออาชีพ:**  
We agree FAR alone is not a full RT evaluation. That is why the revision already adds a worst-case blocking term for response-time analysis (attestation as lowest-priority work with non-preemptive sections of length $t_{\mathrm{disabled}}$). We also agree experimental deadline-miss / jitter under a realistic task would strengthen the paper; that matches the Option A experiment we discussed. We do **not** plan to treat end-to-end response time of the Secure service as the main metric, for the reason you noted (RTA components are about the task set / blocking / interference, not “Secure-world response time” as a product KPI).

**Action:**  
- คง RTA blocking (มีประโยชน์)  
- แทน/เสริม synthetic FAR ด้วย Option A metrics เมื่อทดลองเสร็จ  
- อย่าลบ FAR ทั้งก้อนโดยไม่มีตัวแทน — FAR ยังเป็นผลหลักของฉบับส่ง

---

## 19. อย่าผสม paper อื่น (Joia Neto tick-loss)

**OAK:** *dont mix the other paper in this paper....*

**ทำไมเขียน:** อธิบายว่าทำไม FAR ตกที่ $B\ge2048$ (SysTick collapse / tick loss) และอ้าง Joia Neto et al. ซึ่งเป็นงาน joint ที่เกี่ยวข้อง — มาช่วยตอบ R3.8 / ความถูกต้องของคำอธิบาย

**ตอบอาจารย์ (ร่าง):**  
The tick-loss explanation is needed to interpret the FAR cliff; we can keep a self-contained FreeRTOS explanation (pending SysTick → lost releases when $t_{\mathrm{disabled}}>1$ ms) and **remove or minimize the cross-paper citation/discussion** so this manuscript stays standalone. If needed, one short footnote without expanding their model is enough.

**Action:** ตัด/ย่อส่วน Joia Neto — เก็บสมการง่ายๆ ของเราเอง

---

## 20. Worst-case blocking / WCET section

**OAK:** *the point of WCET is that we can still schedule real-time task along with attestation task I think.*

**ทำไมเขียน:** ตอบ reviewer เรื่อง classical RT — แปลง attestation เป็น blocking ใน RTA ไม่ได้วัด WCET ของแอปเอง

**ตอบอาจารย์ (ร่าง):**  
Agreed on the intended point: show that a task set can still be scheduled in the presence of attestation by accounting for $t_{\mathrm{disabled}}(B)$ as blocking. We will retitle/rephrase away from sounding like a WCET study of the application, and emphasize “schedulability with attestation blocking.”

**Action:** ปรับชื่อ/ประโยคนำใหตรงว่าเป็น schedulability + blocking ไม่ใช่ WCET campaign

---

## 21. Timing behavior of order generation (PRNG)

**OAK:** *this should be in security analysis, not its own section.*

**ทำไมเขียน:** **R3.3** — constant-time FindKthZero / RandInt / AES tables

**ตอบอาจารย์ (ร่าง):**  
Agree — fold into the PRNG security/soundness discussion (or appendix), not a free-standing design subsection.

**Action:** ย้ายไป Security analysis / appendix

---

## 22. FFX instantiation

**OAK:** *appendix*

**ทำไมเขียน:** **R3.3** — ระบุ FF1-style rounds / Speck / cycle walking / ไม่ claim NIST compliance

**ตอบอาจารย์ (ร่าง):**  
Agree — move FFX instantiation details to the appendix; keep one sentence in the FPE section.

**Action:** ย้าย appendix

---

## 23. Measured Secure Memory Footprint (ตารางละเอียด)

**OAK:** *why so much detail...*

**ทำไมเขียน:** **R3.12 / R3.13** — reviewer ขอ peak Secure RAM/flash จาก build ไม่ใช่แค่ permutation bits; และกันการ overclaim “O(1) storage” ทั้งที่ FPE มี Speck state

**ตอบอาจารย์ (ร่าง):**  
This table answers the review request for a complete Secure footprint (not only permutation metadata). We can keep a compact version (peak RAM at $B=64/1024/4096$ + one sentence on method) and move ELF/stack methodology to the appendix.

**Action:** ย่อตารางใน main text; รายละเอียดวัด → appendix — **อย่าลบตัวเลข peak RAM** เพราะเป็นคำตอบ reviewer โดยตรง

---

## 24. Runtime ควรโชว์หลาย traversals (T=3,7)

**OAK:** *this should be with multiple repetitions like 3 times, 7 times to achieve non-negligible detection prob.*

**ทำไมเขียน T=1 ในรูป runtime หลัก:** วัดหนึ่ง traversal เป็น baseline; มี `sec:traversals` + ตารางที่ scale ด้วย $T$ อยู่แล้ว (เช่น T=7 สำหรับ ~$10^{-3}$)

**ตอบอาจารย์ (ร่าง):**  
Agree that the main runtime discussion should foreground the multi-traversal operating point (e.g., $T=7$). We already include a traversals subsection with scaled runtime/energy; we will move those numbers earlier / into the main runtime table or figure annotation so readers do not think $T=1$ is the recommended deployment setting.

**Action:** ดันตัวเลข $T>1$ ให้อยู่ใกล้ runtime หลัก; ไม่จำเป็นต้อง re-measure ถ้ายังใช้ linear scaling ที่ประกาศไว้ (แต่ถ้าอาจารย์อยากกราฟ $T=7$ โดยตรง อาจ derive จากของเดิม)

---

## 25. Window of exposure paragraph

**OAK:** *whats the point of this?*

**ทำไมเขียน:** เชื่อม runtime กับ security — attestation ช้า = หน้าต่างที่ compromise ยังไม่ถูกจับยาวขึ้น; เกี่ยวกับ R3.6 / การเลือก $B$ เล็กเกินไป

**ตอบอาจารย์ (ร่าง):**  
It was meant to give a security reading of end-to-end latency (detection delay). If you find it unhelpful or redundant with the traversals discussion, we can delete it and rely on the $T$-traversal / escape-probability subsection instead.

**Action:** **ลบได้** ถ้าอาจารย์ไม่ชอบ — สาระซ้ำกับ traversals ได้

---

## 26. Energy methodology (comment ใน tex)

**OAK (comment):** *too much info below* — ส่วนหนึ่งถูก comment ไปแล้ว; แต่ `\ben{...}` เรื่อง FNB58 ยังยาว

**ทำไมเขียน:** **R3.10** — methodology ของ energy/statistics

**ตอบอาจารย์ (ร่าง):**  
Agree to keep only the essentials in the main text (meter, what is included, $N=10$, markers) and move accuracy/error-budget details to the appendix.

**Action:** ย่อ energy paragraph

---

## แผนรวม: อะไรควรลบ / ย้าย / เก็บ / ทดลอง

### ลบหรือย่อใน main text ได้เลย (อาจารย์ถูกเรื่องความยาว)
- Mirai-only → อัปเดต cite  
- (S1)–(S3) ใน contributions  
- ประโยค novelty ยาวใน contributions  
- TrustZone ประโยคซ้ำ  
- ย่อหน้า misconfiguration ที่งง  
- “boring” GTZC address clause  
- caption t_disabled เรื่อง TF-M  
- Window of exposure (ถ้าอาจารย์ยืนยันลบ)  
- ย่อ Joia Neto crossover discussion  

### ย้าย appendix (สาระต้องอยู่ เพื่อตอบ reviewer)
- Platform Security Configuration  
- Cryptographic instantiation  
- FFX instantiation  
- Key hierarchy formulas / HMAC header byte layout  
- Between-block invariant (หรือไป Security section)  
- Timing behavior of FindKthZero (ไป security/appendix)  
- รายละเอียดวัด footprint / energy error budget  

### เก็บใน main text แต่ทำให้สั้นและตรงจุด
- Threat-model assumptions: secure boot, provisioning, DMA  
- Peak Secure RAM numbers (ย่อ)  
- FAR + blocking/RTA (ปรับคำให้ตรง “schedulability”)  
- Traversals / $T=7$ operating point (ดันให้เด่นขึ้น)  

### งานทดลอง (ไม่จบด้วยการลบประโยค)
- Real-world / Option A RT task + deadline-miss / jitter ตามที่อาจารย์ขอ  
- (ถ้าต้องการ) โชว์ runtime ที่ $T=3,7$ ชัดใน figure หลัก  

---

## ร่างข้อความรวมสั้นๆ ส่งอาจารย์ (EN, meeting/email)

> Thank you for the comments. Most of the long blue paragraphs were added to answer specific IEEE Access reviewer requests (platform assumptions, crypto instantiation, metadata binding, complete Secure footprint, FAR limitations / blocking). We agree the main paper became too report-like.  
>  
> Plan: (1) keep short threat-model assumptions (secure boot, key provisioning, DMA isolation); (2) move platform/crypto/FFX/header/footprint methodology to the appendix; (3) delete redundant intro sentences and the confusing TrustZone-misconfiguration aside; (4) simplify FAR discussion and avoid mixing in the other paper; (5) foreground multi-traversal cost; (6) proceed with the more realistic RT experiment (Option A) for deadline-oriented metrics.  
>  
> We will not drop the reviewer-facing substance—only relocate/compress it.

---

## Checklist สำหรับรอบแก้ถัดไป (เมื่อเริ่มลงมือ)

- [ ] อัปเดต botnet citations  
- [ ] ตัด (S1)–(S3) / novelty ยาวจาก contributions  
- [ ] รวม TrustZone background ที่ซ้ำ  
- [ ] Threat model สั้น + ย้าย platform-config → appendix  
- [ ] ย่อ/ย้าย KDF, hdr, traversals-in-III, continuation invariant  
- [ ] Crypto + FFX → appendix  
- [ ] ตัด boring implementation clause + caption DELETE IT  
- [ ] นิยาม $P$; ย่อ Joia Neto; ปรับ blocking wording  
- [ ] ย่อ footprint + energy; ดัน $T>1$; พิจารณาลบ window-of-exposure  
- [ ] วางแผน/ทำ Option A experiment  

---

*เอกสารนี้สำหรับคุยกับอาจารย์และวางแผนแก้ — ยังไม่ใช่ response letter ถึง IEEE Access reviewers*
