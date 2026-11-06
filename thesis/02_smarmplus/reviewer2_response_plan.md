# SMARM+ IEEE Access — Reviewer 2 (แผนตอบ + แก้ paper)

Reviewer 2 โทนบวก (well written, technically sound = Yes)  
มี **3 ข้อแนะนำ** ก่อน publish — ไม่ใช่ major technical reject แบบ Reviewer 1  
Additional Questions ท้ายฟอร์ม = เรตติ้ง ไม่ต้องตอบแยกเป็นข้อเทคนิค

---

## ภาพรวมสั้นๆ

| ข้อ | ใจความ | ถูกไหม | ทดลองใหม่? | แก้ paper? |
|-----|--------|--------|------------|------------|
| **R2.1** | ใส่ DOI ใน bibliography | ถูก / ทำได้ | ไม่ | ใช่ (bib) |
| **R2.2** | ตัวอักษรในรูปใหญ่ขึ้น / รูป 2 คอลัมน์ | ถูก (พิมพ์ A4 อ่านยาก) | ไม่ | ใช่ (regen รูป) |
| **R2.3** | ระบุขนาด memory ที่ attest + window-of-exposure | ถูกบางส่วน — ตัวเลขอนุมานได้แต่ไม่เขียนชัด | ไม่ | ใช่ (ข้อความ) |

---

## R2.1 — DOI ใน bibliography

### แบบง่ายๆ

**Reviewer พูดอะไร**  
ทุกรายการใน bibliography ควรใส่ DOI (ถ้ามี)

**ใน paper มีอะไรอยู่แล้ว**  
มี references ปกติ แต่จาก Access PDF **ไม่เห็นการเน้น DOI เป็นระบบ** (หลายรายการเป็น URL / proceedings โดยไม่มี DOI field)

**ช่องว่างจริง**  
ยังไม่ได้ใส่ DOI ครบตามที่เค้าขอ

**Reviewer พูดถูกแค่ไหน**  
ถูก — เป็น polish มาตรฐานของ IEEE Access ไม่เกี่ยวกับ soundness

### Response to Reviewers (copy-ready)

**Reviewer’s concern**  
For each entry in the bibliography, provide its DOI (when available).

**Our response**  
We agree. Providing DOIs improves traceability and matches IEEE Access bibliography practice. The submitted manuscript included venue/URL information for many entries but did not systematically list DOIs.

**Action**  
In the revised manuscript we will add a DOI for every bibliography entry for which a DOI is available (IEEE Xplore, ACM DL, Springer, etc.). Entries without a public DOI (e.g., some web pages, datasheets, or early drafts) will retain the best available stable URL and access date. No technical results change.

### Text / สิ่งที่ต้องทำใน paper
- แก้ `citation.bib` / bibliography: เพิ่ม `doi = {...}`  
- ไม่ต้องแก้เนื้อหาบทความนอก bib  
- ไม่ต้องทดลองใหม่

### สรุป
แค่ตอบ + อัปเดต bib — งานบรรณานุกรม ไม่ใช่ science ใหม่

---

## R2.2 — ตัวอักษรในรูปเล็กเกินไป

### แบบง่ายๆ

**Reviewer พูดอะไร**  
ตัวหนังสือในรูปเล็กเกินไป พิมพ์ A4 อ่านแทบไม่ออก  
ให้ขยายฟอนต์ หรือทำรูปกว้างสองคอลัมน์

**ใน paper มีอะไรอยู่แล้ว**  
มีหลายรูป (system model, FAR plots, storage, runtime, …) แบบ single-column — reviewer บอกชัดว่าตอนพิมพ์อ่านยาก

**ช่องว่างจริง**  
เป็นปัญหา presentation ของรูป ไม่ใช่ข้อมูลผิด

**Reviewer พูดถูกแค่ไหน**  
ถูกในทาง usability — ควรทำก่อน resubmit

### Response to Reviewers (copy-ready)

**Reviewer’s concern**  
Make text in all figures bigger (or make each figure two-columns wide) because in the current version figures are nearly unreadable when printed in A4.

**Our response**  
We agree. Several plots and diagrams in the submitted PDF used font sizes that are difficult to read in single-column print. This is a presentation issue and does not affect the underlying data.

**Action**  
In the revised manuscript we will regenerate figures with larger axis labels, tick labels, legends, and in-figure text (target roughly ≥8–9 pt when placed at final column width). Where a figure remains crowded at single-column width (e.g., multi-panel FAR plots), we will either enlarge fonts further or place the figure as a two-column (`figure*`) float. We will re-check a printed/A4 PDF proof before resubmission. No experimental reruns are required; only figure assets and LaTeX figure environments change.

### Text / สิ่งที่ต้องทำใน paper
- Regen รูปจากสคริปต์ (matplotlib: `fontsize`, `figsize`) หรือ draw.io  
- รูปแน่นมาก → พิจารณา `\begin{figure*}`  
- Caption เดิมใช้ได้ ส่วนใหญ่ไม่ต้องเขียน prose ใหม่

### สรุป
ตอบ + ทำรูปใหม่ — **ไม่ทดลองใหม่** แต่ควรทำจริงก่อนส่ง (ไม่งั้น reviewer รอบหน้าจะทักซ้ำ)

---

## R2.3 — ขนาด memory ที่ attest + window-of-exposure

### แบบง่ายๆ

**Reviewer พูดอะไร**  
ตอนรายงานเวลา attestation ทั้งก้อน (ทุกบล็อก) ให้:
1. ระบุว่าวัด memory รวมกี่ขนาด  
2. คอมเมนต์ว่า attestation นานขึ้น = ช่วงที่โจมตีอาจยังไม่ถูกจับนานขึ้น (**window-of-exposure**)

**ใน paper มีอะไรอยู่แล้ว**
- มี Table I / Fig. 10 = overall attestation runtime ตาม `B`  
- บอร์ดระบุ flash 512 KB และแบ่ง Secure/NS  
- จากตัวเลข storage (เช่น ที่ `B=1024`, SMARM storage 576 B สอดคล้อง `n=512` → `|M|=n·B=512` KiB) **อนุมานได้ว่า attested size คงที่ ~512 KiB**  
- แต่ **ข้าง Table I ไม่ได้เขียนชัดว่า `|M|=…`**  
- คำว่า **window-of-exposure / undetected attack window** ใน Access PDF **ไม่มี**

**ช่องว่างจริง**  
ตัวเลข runtime มีแล้ว แต่ขาด (1) ระบุ `|M|` ชัดๆ ข้างผล และ (2) ตีความ security เรื่อง window-of-exposure

**Reviewer พูดถูกแค่ไหน**  
ถูก — เป็น clarification ที่ควรมี  
ไม่ใช่ขอทดลองใหม่

**หมายเหตุสำคัญ**  
ก่อนเขียน `|M|=512` KiB ใน paper ยืนยันจาก firmware จริงอีกครั้ง (R3 ก็ทักเรื่อง flash Secure ครึ่ง vs ขนาดที่วัด — ต้องสอดคล้องกัน)

### Response to Reviewers (copy-ready)

**Reviewer’s concern**  
When presenting the results about the total computation time required for complete attestation (i.e., all memory blocks), please specify the total memory size attested and comment that the longer the attestation time, the wider the time for which an attack goes undetected (i.e., wider window-of-exposure).

**Our response**  
We agree that end-to-end attestation runtime should be reported together with the attested memory size, and that longer attestation latency implies a longer window during which compromise may remain undetected until the current round completes. In our experiments the attested region size is fixed while the block size `B` varies (hence `n = |M|/B` changes). The submitted manuscript reported runtimes in Table I / Figure 10 and described the platform memory map, but did not state `|M|` explicitly next to those results, nor discuss the window-of-exposure interpretation.

**Action**  
In the revised manuscript we will:  
1. State the attested memory size `|M|` explicitly in Section VI (and in the Table I / Figure 10 captions), together with how `n` relates to `B`.  
2. Add a short comment that longer end-to-end attestation time widens the window-of-exposure until the verifier obtains fresh evidence.  
No new experiments are required for this clarification.

### Text to add in the paper

**A) Section VI-B (ใกล้ Table I / Fig. 10)**

```text
Unless otherwise noted, all end-to-end runtime and energy results
correspond to attesting a fixed memory region of size |M| = 512 KiB
(so the number of blocks is n = |M|/B). Changing B therefore changes
the number of blocks and the per-block work, but not the total number
of bytes hashed in one complete attestation round.
```

*(ปรับตัวเลข `|M|` ให้ตรง firmware จริง)*

**B) Caption Table I (ตัวอย่าง)**

```text
Overall attestation runtime for a complete scan of |M| = 512 KiB
under SMARM and SMARM+ (FreeRTOS). The shaded row marks B = 1024 bytes,
the largest evaluated block size that preserves FAR ≈ 1.0.
```

**C) ย่อหน้าสั้นเรื่อง window-of-exposure**

```text
Window of exposure.
End-to-end attestation latency also has a security interpretation:
until a round completes and the verifier accepts fresh evidence,
software compromise on the prover may remain undetected. Thus, designs
or parameter choices that inflate total attestation time (e.g., very
small B under SMARM+PRNG) widen this window-of-exposure, even when
FAR remains close to 1.0. Deployers should therefore jointly consider
real-time interference (FAR), storage, and attestation latency.
```

### สรุป
ตอบ + เติมข้อความ/caption — **ไม่ต้องทดลองใหม่**  
ตรวจ `|M|` ให้ตรงของจริงก่อนใส่ตัวเลข

---

## Additional Questions (ฟอร์มท้าย — ไม่ใช่โจทย์เทคนิคใหม่)

| คำถาม | คำตอบ R2 | ความหมาย |
|-------|----------|----------|
| Reviewed all files? | Yes | ฟอร์มบังคับ |
| Contribute to knowledge? | **Yes** | เห็นว่ามี contribution |
| Technically sound? | **Yes** | ผ่านด้านเทคนิค |
| Comprehensive? | **Yes** | ครอบคลุมพอ |
| Refs applicable? | **Yes** | อ้างอิงโอเค |
| Inappropriate refs? | **No** | ไม่ต้องลบ ref จากข้อนี้ |
| 5a | ว่าง | — |

**ไม่ต้องเขียน Response แยกเป็นข้อ 1)–5a**  
แค่ทำให้ 3 ข้อแนะนำด้านบน — ท้าย response อาจสรุปสั้นๆ ว่า revisions address the presentation suggestions while preserving the positive technical assessment.

ตัวอย่างท้ายไฟล์ response:

> We thank Reviewer 2 for the positive assessment and for the concrete presentation suggestions. We have addressed the DOI, figure readability, and attested-size / window-of-exposure clarifications as detailed above.

---

## Checklist งาน Reviewer 2

- [ ] R2.1: ใส่ DOI ใน bib (ทุกอันที่มี)
- [ ] R2.2: regen รูปฟอนต์ใหญ่ขึ้น / บางรูป `figure*`
- [ ] R2.3: ระบุ `|M|` ชัด + caption Table I/Fig.10 + ย่อหน้า window-of-exposure
- [ ] ยืนยัน `|M|` จาก firmware ให้ตรงข้อความ
- [ ] Response to reviewers ทั้ง 3 ข้อ
- [ ] Highlight จุดที่แก้ใน PDF

---

## สรุปทั้ง Reviewer 2 一句

Reviewer 2 **ผ่านเทคนิคแล้ว** ขอแค่ polish:  
**DOI + รูปอ่านง่าย + บอกขนาด memory / window-of-exposure**  
ไม่มี major crypto/proof แบบ Reviewer 1 — ทำครบได้โดยไม่ต้องรันบอร์ดใหม่

---

*คู่กับ `reviewer1_response_plan.md` สำหรับ Reviewer 1*
