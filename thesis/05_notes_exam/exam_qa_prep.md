# คู่มือเตรียมสอบ / ถาม–ตอบเปเปอร์ SMARM + Atomic Copy + SAU

ใช้ทบทวนก่อนสอบกับอาจารย์ (โดยเฉพาะสาย RA / TrustZone)
พูดสั้น ชัด มีตัวเลข และรู้ limitation

---

## 0) Elevator pitch (30–45 วินาที)

> Classical RA ปิด interrupt ทั้งสแกน ทำให้ RT task พัง  
> SMARM วัดแบบ shuffle เปิด interrupt **ระหว่างบล็อก** เพื่อต้าน *roving malware*  
> แต่พอร์ตมา FreeRTOS/TrustZone-M แล้ว ยังปิด IRQ ตอน **hash แต่ละบล็อก** → FAR ตกเมื่อ \(B\) ใหญ่  
> เราเสนอ 2 วิธี: **Atomic Copy** (IRQ-off แค่ copy) และ **SAU hole** (hash ทับที่เดิม ไม่ต้อง buffer)  
> ทั้งคู่คืน FAR ≈ 1.0 ที่ \(B\le 4096\) และ 10–1000 Hz โดยยังรักษา (S1)–(S3)

---

## 1) พื้นฐาน: ปัญหาและ threat

### Q1. Remote attestation คืออะไร? ทำไม IoT ถึงต้องการ?

**ตอบสั้น:**  
Verifier ส่ง challenge → device วัด memory/state ด้วย key ที่ malware เข้าไม่ถึง → ส่ง report ที่ verify ได้  
เป้าหมายคือตรวจว่า firmware/memory ยังตรงกับที่คาด โดยไม่ต้อง trust software ทั้งหมดบน device

**อย่าพูดเกิน:** ไม่ใช่แค่ “hash ทั้ง flash แล้วส่งกลับ” — ต้องมี key secrecy + atomicity / order secrecy ตามโปรโตคอล

---

### Q2. Roving malware คืออะไร? ต่างจาก malware ธรรมดายังไง?

**ตอบ:**  
Malware ที่ **ย้ายตำแหน่งระหว่างการวัด** เพื่อหลบช่วงที่กำลังถูก hash  
ถ้าสแกนแบบ linear และไม่ atomic ทั้งก้อน malware อาจ “หนี” ไปพื้นที่ที่ยังไม่ถูกวัด / ถูกวัดไปแล้ว

**SMARM รับมือยังไง:**  
ลำดับการวัด \(\sigma\) เป็นความลับ (derive จาก challenge + \(k\)) → malware ที่ไม่รู้ \(\sigma\) เดาไม่ได้ว่าบล็อกไหนจะถูกวัดถัดไป → หนีแบบมีเป้าหมายยาก และทำซ้ำหลายรอบแล้ว escape probability ลดลง

---

### Q3. ทำไม classical RA (SMART/VRASED แนว atomic ทั้งสแกน) ถึงไม่เหมาะกับ real-time?

**ตอบ:**  
ต้องปิด interrupt ตลอดการวัด → บน MCU ต่ำอาจยาวหลายร้อย ms  
RT task ที่ต้องการ period 1 ms (1000 Hz) จะพลาด deadline แน่นอน

**ตัวเลขอ้างอิงในเปเปอร์:** classical window สามารถเกิน ~450 ms ได้ (อ้าง VRASED)

---

### Q4. SMARM อนุญาต interrupt ได้เมื่อไหร่? อะไรยังต้อง atomic?

**ตอบ (สำคัญมาก — อย่าสับสน):**  
- **ระหว่างบล็อก:** เปิด interrupt ได้  
- **ภายในบล็อก (baseline SMARM):** ยังปิด IRQ ตอน hash ทั้งบล็อก  
- สิ่งที่ต้อง “ไม่ให้ malware แทรกครึ่งๆ” คือ **การ capture เนื้อหาบล็อก** ที่จะป้อนเข้า hash ไม่ใช่การคำนวณ crypto เองเสมอไป

นี่คือ root cause ของงานนี้:  
> Hash ไม่จำเป็นต้อง atomic แต่ **bytes ที่ถูก hash** ต้องเป็น snapshot ที่สอดคล้องกัน

---

## 2) Security properties (S1)–(S3) — คาดว่าโดนถามแน่

### Q5. (S1) Controlled Execution คืออะไร?

**ตอบ:**  
1. Attestation service เรียกได้เฉพาะ entry/exit ที่กำหนด (บนเรา = NSC → Secure)  
2. Interrupt เปิดได้เฉพาะ **ระหว่าง** block measurements — ไม่เปิดกลางขั้นตอน capture ที่ต้อง protected

**Atomic Copy รักษา S1 ยังไง:** IRQ ปิดทั้งช่วง `Memcpy` → snapshot ไม่ถูกเขียนครึ่งๆ  
**SAU รักษา S1 ยังไง:** เปิด hole ทำให้บล็อกนั้นเป็น Secure → NS code อ่าน/เขียนบล็อกนั้นไม่ได้ตามกฏ CPU attribution ระหว่าง hash (แม้ IRQ ยังเปิด)

---

### Q6. (S2) Key Secrecy?

**ตอบ:**  
Key \(k\) อยู่ใน Secure RAM / Secure world — NS malware อ่านไม่ได้ → ปลอม report ไม่ได้

ทั้ง Atomic Copy และ SAU **ไม่ย้าย \(k\)** ออกจาก Secure

---

### Q7. (S3) Private Measurement Order?

**ตอบ:**  
Permutation \(\sigma\) อยู่ใน Secure และไม่เขียนลง NS memory  
ถ้า malware รู้ \(\sigma\) ที่เหลือได้ = รู้ “บล็อกถัดไป” = roving แบบมีเป้าหมายได้ → SMARM พัง

**อย่าสับสน:** ความลับของ \(\sigma\) ≠ การปิด interrupt ทั้งสแกน

---

### Q8. Atomic Copy / SAU เปลี่ยน security ของ SMARM หรือไม่?

**ตอบแบบสอบได้คะแนน:**  
ไม่เปลี่ยนโปรโตคอล shuffle และไม่เปลี่ยนที่เก็บ \(k,\sigma\)  
เปลี่ยนแค่ **กลไก capture ต่อบล็อก**  
ดังนั้น (S1)–(S3) ยังอยู่ — แต่รูปแบบ enforcement ของ S1 ต่างกัน (IRQ-off copy vs SAU hole)

---

## 3) Contribution และ novelty

### Q9. Contribution ของคุณคืออะไร? SMARM มีอยู่แล้วไม่ใช่หรือ?

**โครงตอบ 3 จังหวะ:**  
1. **Port + วัดบน production RTOS:** SMARM เดิมอยู่ seL4/HYDRA — เรานำมา FreeRTOS + TrustZone-M และใช้ FAR  
2. **พบ bottleneck ที่เหลือ:** แม้ interruptible ระหว่างบล็อก แต่ per-block IRQ-off hash ยังทำให้ FAR พังเมื่อ \(B\) ใหญ่  
3. **สอง mitigation บน TrustZone-M:** Atomic Copy และ transient SAU hole โดยไม่ทิ้ง shuffled structure

**ประโยคกันโดน “แค่ implement”:**  
> งานไม่ใช่แค่พอร์ต — เรา identify ว่า atomicity ที่จำเป็นอยู่ที่ capture ไม่ใช่ hash แล้วออกแบบสองทางเลือกที่มี trade-off ชัด (RAM vs SAU layout)

---

### Q10. ทำไมต้องมีทั้ง Atomic Copy และ SAU? เหลืออันเดียวไม่พอหรือ?

**ตอบ:**  
เป็น **design alternatives** ตาม constraint ของแพลตฟอร์ม  

| | Atomic Copy | SAU |
|---|---|---|
| FAR | ≈ 1.0 | ≈ 1.0 |
| IRQ-off | สั้น = copy | ไม่ต้อง IRQ-off เพื่อ hash |
| ต้นทุน | Secure buffer ขนาด \(B\) | ต้องมี NS data window แยก + แก้ SAU ชั่วคราว |
| เหมาะเมื่อ | Secure RAM มีพอ / SAU ต้อง lock หลัง boot | Secure RAM แน่น / อนุญาตให้ Secure แก้ SAU ชั่วคราว |

---

## 4) Atomic Copy — คำถามเจาะ

### Q11. Atomic Copy ทำงานยังไง? (วาด/พูดตาม flowchart)

**ลำดับ:**  
1. Derive \(\sigma\) ใน Secure  
2. เลือกบล็อก \(M_{\sigma(i)}\)  
3. **Disable IRQ → Memcpy เข้า Secure buffer → Enable IRQ**  
4. `HMAC.Update` บน buffer (IRQ เปิดได้)  
5. ทำจนครบ → Finalize MAC

---

### Q12. ทำไม copy เร็วกว่า hash ประมาณ 15×?

**ตอบ:**  
บน Cortex-M33 ของเรา memory copy ~17 MB/s  
\(t_{\mathrm{disabled}}^{\mathrm{copy}} \approx B / 17\,\mathrm{MB/s}\)  
เทียบกับ SHA-256/HMAC update ที่ปิด IRQ ทั้งบล็อกใน baseline → จากตาราง \(t_{\mathrm{disabled}}\) ได้ประมาณ 15–16× สำหรับ \(B\ge 128\)

**ตัวเลขจำ:**  
- Baseline \(B=4096\): ~3.75 ms  
- Atomic Copy \(B=4096\): ~0.24 ms  

ที่ 1000 Hz period = 1 ms → baseline ที่ 3.75 ms พลาด deadline ได้ง่าย ส่วน copy 0.24 ms ยังอยู่ภายใต้ period

---

### Q13. ตอน hash เปิด IRQ แล้ว malware แก้ Secure buffer ได้ไหม?

**ตอบ:**  
ไม่ได้ — buffer อยู่ใน **Secure RAM** NS เข้าไม่ถึง  
ที่ต้องกันคือ malware แก้ **ต้นฉบับ NS** ระหว่าง copy → เลยปิด IRQ ตอน copy

---

### Q14. ถ้า malware แก้บล็อกต้นฉบับ *หลัง* copy แล้ว *ก่อน* บล็อกนั้นถูกวัดในรอบอื่น?

**ตอบตาม SMARM:**  
หลัง copy ไปแล้ว รายงานของบล็อกนี้สะท้อน snapshot ตอนนั้น  
การย้าย/แก้หลังถูกวัดแล้วเป็นพฤติกรรม roving ปกติที่ SMARM วิเคราะห์ด้วยความลับของลำดับที่เหลือ + หลายรอบ attestation  
Atomic Copy ไม่ได้ claim ว่า “memory นิ่งทั้งระบบตลอด session” — claim แค่แต่ละบล็อกที่วัดมี consistent snapshot

---

## 5) SAU hole — คำถามเจาะ (ยาก / อาจารย์สาย Arm จะชอบ)

### Q15. SAU คืออะไร? “hole” หมายความว่าอะไร?

**ตอบ:**  
SAU (Security Attribution Unit) กำหนดว่าช่วง address เป็น NS หรือ Secure(+NSC)  
บน Armv8-M: **memory ที่ไม่ถูกคลุมด้วย enabled SAU region = Secure โดยค่าเริ่มต้น**

**Hole ของเรา:**  
Data window ปกติถูกคลุมเป็น NS (region 6/7)  
ตอนวัดบล็อก \(B\): โปรแกรม left/right ให้เป็น NS fragments → **ช่วงบล็อกไม่ถูกคลุม** → กลายเป็น Secure → Secure code hash ทับที่เดิมได้ โดย NS เข้าบล็อกนั้นไม่ได้

---

### Q16. ทำไมห้ามตั้ง SAU region ทับ Secure flash แบบ ENABLE=1, NSC=0?

**ตอบสั้น (พูดสอบ):**  
บน Armv8-M ถ้าเปิด SAU region แล้ว ช่วง address นั้นเป็นได้แค่ 2 แบบเท่านั้น:
1. **Non-Secure (NS)** หรือ
2. **Secure + NSC** (ประตูให้ NS เรียกเข้า Secure ได้)

**ไม่มีโหมด “เปิด region แล้วเป็น Secure ธรรมดา”**  
Secure ธรรมดา = ช่วงที่ **ไม่มี** enabled region คลุมอยู่ (ของเหลือ = Secure โดยค่าเริ่มต้น)

---

**เปรียบเหมือนป้ายบนชั้นหนังสือในห้องสมุด**

สมมติทั้งตึกมีกฎ:
- ชั้นที่ **ติดป้าย** = “ยืมได้ (NS)” หรือ “ยืมได้เฉพาะทางประตูพิเศษ (NSC)”
- ชั้นที่ **ไม่ติดป้าย** = “ห้องพนักงานเท่านั้น (Secure)”

ถ้าอยากให้หนังสือเล่มหนึ่งเป็นของ Secure ชั่วคราว คุณทำถูกคือ:
- **ถอดป้าย NS ออกจากชั้นนั้น** → มันกลายเป็นห้องพนักงานเอง  
  นี่คือ **SAU hole** ของเรา

คุณทำผิดคือ:
- ไปติดป้ายว่า “ยืมได้ (NS)” ทับชั้นที่เป็นโค้ด Secure  
  → malware ฝั่ง NS อาจเข้าถึงโค้ด/ข้อมูล Secure ได้  
  หรือระบบ attribution พัง

**บั๊กที่เคยเจอในโค้ดเก่า:**  
ตั้งใจจะ “mark เป็น Secure” แต่ไปเขียน SAU แบบ `ENABLE=1, NSC=0` ทับช่วง Secure flash  
ในฮาร์ดแวร์แล้ว นั่นแปลว่า **ทำให้ช่วงนั้นเป็น NS** ไม่ใช่ Secure  
เลยพัง map

**เลยในงานนี้ใช้กฎ:**  
> อยากได้ Secure ชั่วคราว → **ไม่คลุมด้วย region** (uncovered = Secure)  
> ไม่ใช่เปิด region แล้วหวังว่า NSC=0 จะแปลว่า Secure

---

**เคลียร์ concept ให้ตรง (คำถามที่มักสับสน):**

คำถาม: “แล้ว SAU ไปแปะป้าย NS ด้วยเหรอ? คิดว่าเป้าหมายคือเปลี่ยนเป็น Secure แป๊บหนึ่งไม่ใช่หรือ?”

คำตอบ: **เป้าหมายถูกแล้ว = ทำให้บล็อกเป็น Secure ชั่วคราว**  
แต่บน Arm **ไม่มีป้ายชื่อว่า Secure** ให้แปะ  
มีแค่ป้าย NS / NSC — ของที่ไม่มีป้าย = Secure อัตโนมัติ

ดังนั้นในโค้ดจริงมี 2 สถานะ:

| สถานะ | เราทำอะไรกับ SAU | ผลกับบล็อกข้อมูล |
|---|---|---|
| **พัก (rest)** | แปะป้าย NS คลุมทั้ง data window (region 6/7) | บล็อกเป็น **NS** — แอป NS ใช้อยู่ได้ตามปกติ |
| **ตอนวัดบล็อก** | **ถอดป้าย** เฉพาะช่วง \(M_{\sigma(i)}\) (เหลือ left/right เป็น NS) | ช่วงนั้น **ไม่มีป้าย** → กลายเป็น **Secure** ชั่วคราว = hole |
| **หลังวัด** | แปะป้าย NS กลับคลุมทั้ง window | กลับเป็น NS |

สรุปหนึ่งประโยค:
> เราไม่ได้ “เขียน Secure=1” — เรา **เอาป้าย NS ออกชั่วคราว** บล็อกเลยกลายเป็น Secure

ส่วนประโยค “ห้ามไปติดป้าย NS ทับ Secure flash” หมายถึง **บั๊กคนละเรื่อง**:
- คนตั้งใจ mark โค้ด Secure ให้เป็น Secure  
- แต่เขียน `ENABLE=1, NSC=0` ซึ่งฮาร์ดแวร์ตีความว่า **แปะป้าย NS**  
- เลยเผลอทำให้ Secure flash กลายเป็น NS — พัง

นั่นไม่ใช่ขั้นตอนปกติของ hole remapping ของเรา

### Q17. ทำไมต้อง DSB/ISB?

**ย่อมาจากอะไร:**
- **DSB** = *Data Synchronization Barrier* DSB — รอให้การเขียน memory/register (เช่น ตั้ง SAU) เสร็จก่อน
- **ISB** = *Instruction Synchronization Barrier* ISB — ให้ pipeline ใช้ค่าใหม่ (ทิ้ง instruction ที่ fetch มาตอน attribute เก่ายังค้าง)

เป็นคำสั่ง barrier ของ ARM ที่บังคับให้ CPU “รอ/ซิงก์” ก่อนทำขั้นต่อไป

**ตอบสั้น:**  
เปลี่ยน SAU แล้ว CPU อาจยัง “จำค่าเก่า” อยู่ชั่วคราว  
`DSB` + `ISB` = สั่งให้ **เขียนค่าใหม่ให้เสร็จ และเริ่มใช้ค่าใหม่ทันที** ก่อนไป hash / restore

---

**เปรียบเหมือนเปลี่ยนกุญแจประตู**

1. คุณเปลี่ยนกุญแจห้อง (โปรแกรม SAU ใหม่ = เปิด hole)
2. ถ้าเดินเข้าไป hash ทันทีโดยไม่รอ → อาจยังใช้กุญแจชุดเก่าในมือ  
   (= CPU ยังเห็นว่าบล็อกนั้นเป็น NS อยู่)
3. `DSB` = “รอให้ช่างใส่กุญแจใหม่เสร็จก่อน” (memory/system ทำงานค้างให้จบ)
4. `ISB` = “ทิ้งแผนที่เดินทางชุดเก่าในหัว แล้วอ่านแผนที่ใหม่” (flush pipeline / instruction side)

**ตัวอย่างในโค้ดเรา (ลำดับที่ถูก):**
```text
เปิด SAU hole รอบบล็อก M[σ(i)]
DSB ; ISB          ← ให้ attribute ใหม่มีผลแล้ว
HMAC.Update(...)   ← ค่อย hash
คืน NS coverage
DSB ; ISB          ← ให้ attribute เดิมกลับมาก่อนจบ/บล็อกถัดไป
```

**ถ้าไม่มี barrier จะเสี่ยงอะไร?**  
เปิด hole แล้วยัง hash ไม่ทัน attribute เปลี่ยน → หรือปิด hole แล้วยังคิดว่า Secure อยู่  
ผลคือ security/behavior ไม่ตรงที่ออกแบบ

**พูด一句ตอนสอบ:**  
> DSB/ISB ทำให้การ remap SAU “มีผลจริง” ก่อนทำ hash หรือคืนค่า — ไม่งั้น CPU อาจทำงานบน security map เก่า

---

### Q18. ตอนเปิด hole แล้ว IRQ ยังเปิด — interrupt handler ที่เป็น NS รันได้ไหม? แตะบล็อกที่กำลัง hash ได้ไหม?

**ตอบแยก 2 ชั้น:**  
1. **NS code/IRQ อื่นรันได้** — เพราะไม่ได้ปิด interrupt ทั้งระบบ (นี่คือจุดขายด้าน RT)  
2. **แตะบล็อกใน hole ไม่ได้ตามกฏ CPU** — access จะ fault / ไม่ lawful เพราะบล็อกเป็น Secure  

**Limitation ที่ต้องพูดเองก่อนโดนตี:**  
SAU/IDAU คุม **CPU attribution** เป็นหลัก — **DMA / bus master** อาจไม่ถูกกันด้วย SAU อย่างเดียว ต้องมี GTZC/MPC ฯลฯ (เปเปอร์บอกชัดว่า leave to extended version)

---

### Q19. ทำไมต้องแยก NS code 64 KB กับ data window 192 KB?

**ตอบ:**  
Hole remapping ต้องไม่ไปคลุม **vector / executable NS code**  
ถ้าเอา code เข้าไปในหน้าต่างที่ถูก remap ชั่วคราว อาจทำให้ NS รันโค้ดไม่ได้หรือ remap อันตราย  
เลย attest เฉพาะ data window ที่ reserve ไว้

**ผลต่อ eval:**  
SAU วัด memory คนละขนาดกับ baseline/Atomic (192 KB vs 512 KB) → **เลยไม่เปรียบเทียบ end-to-end runtime ดิบ** เพราะจะปนขนาดที่วัดกับ overhead จริง ใช้ FAR + \(t_{\mathrm{disabled}}\) + storage แทน

---

### Q20. Production TF-M มัก lock SAU หลัง boot — งานคุณขัดกับแนวนี้ไหม?

**ตอบตรงๆ:**  
ใช่ — prototype **ไม่ lock** SAU เพื่อให้เปิด/ปิด hole ได้  
ดังนั้น SAU approach **ไม่ใช่ drop-in** บนทุก TF-M image  

**Guidance:**  
- ถ้าต้อง lock SAU → ใช้ **Atomic Copy**  
- ถ้าอนุญาต Secure แก้ SAU ช่วงสั้น + มี data window → ใช้ SAU hole  

แสดงว่าคุณเข้าใจ deployment constraint ไม่ใช่แค่ demo board

---

## 6) Evaluation / metrics

### Q21. FAR คืออะไร? ทำไมไม่ใช้แค่ latency?

\[
\mathrm{FAR} = \frac{f_{\mathrm{observed}}}{f_{\mathrm{baseline}}}
\]

**ตอบ:**  
วัดว่า RT task ยังยิงได้ถี่เท่าเดิมระหว่าง attestation หรือไม่  
FAR ≈ 1.0 = แทบไม่รบกวน  
Latency อย่างเดียวไม่บอกว่า task พลาด activation ภายใต้ period จริงแค่ไหน

---

### Q22. ผลหลักที่ต้องจำ?

- **Baseline SMARM:** FAR ดีที่ \(B\le 1024\) ประมาณ; พังชัดที่ \(B\ge 2048\) ที่ 1000 Hz  
- **Atomic Copy & SAU:** FAR ≈ 1.0 ทุก \(B\) ที่ทดสอบถึง 4096  
- **SAU:** 21 configs (\(7\) ขนาด × \(3\) ความถี่) ≈ 1.0  
- **\(t_{\mathrm{disabled}}\):** Atomic ลด ~15–16× vs baseline hash

---

### Q23. ทำไมไม่โชว์ escape probability / runtime vs \(n\) แบบเปเปอร์ SMARM เดิม?

**ตอบ:**  
โฟกัส conference นี้คือ **RT coexistence บน FreeRTOS** หลังพอร์ต  
Security โครงสร้างตาม SMARM เดิม (S1–S3) — ไม่ claim วิเคราะห์ probabilistic ใหม่  
Metric ที่เหมาะกับ claim คือ FAR + interrupt-disabled window + storage/layout trade-off

---

### Q24. Threat model ของคุณครอบคลุมอะไร / ไม่ครอบคลุมอะไร?

**ครอบคลุม (พูดได้):**  
NS malware (รวม roving) ที่ไม่มี \(k,\sigma\); พยายามหลบการวัด / แทรกครึ่งบล็อก

**ไม่ครอบคลุม / limitation:**  
- Secure-world compromise  
- Physical / fault injection  
- DMA ที่ข้าม SAU โดยไม่มี GTZC  
- SAU locked platforms (สำหรับ SAU method)

การยอมรับ limitation ดีกว่าการ overclaim

---

## 7) เปรียบเทียบงานอื่น

### Q25. ต่างจาก SMART / VRASED / HYDRA อย่างไร?

**ตอบ:**  
พวกนั้นเน้น atomic measurement ยาว / trust anchor แข็ง  
เราอยู่บนแนว **SMARM (shuffled, interruptible between blocks)** แล้วแก้ RT bottleneck ของ per-block hash บน FreeRTOS

---

### Q26. ต่างจาก ISC-FLAT / ASAP / IDA?

**ตอบ:**  
หลายงานเน้น control-flow / interruptible attestation คนละมุม  
เราโฟกัส **memory-content RA + RTOS scheduling coexistence**

---

### Q27. ทำไมไม่ใช้ custom hardware (LO-FAT, ATRIUM, …)?

**ตอบ:**  
เป้าหมายคือ **unmodified TrustZone-M MCU** ที่หาได้ทั่วไป ไม่เพิ่ม ASIC/FPGA

---

## 8) Implementation (ถ้าอาจารย์เปิดโค้ด/บอร์ด)

### Q28. ทำไม attestation อยู่ใน Secure world?

**ตอบ:**  
เพื่อ (S1)–(S3): ควบคุม entry ผ่าน NSC, เก็บ \(k,\sigma\), และ (กรณี SAU) เปลี่ยน SAU ได้จาก Secure เท่านั้น

---

### Q29. FreeRTOS อยู่ฝั่งไหน?

**ตอบ:**  
FreeRTOS tasks อยู่ **Non-Secure**  
มี wrapper task เรียก Secure service ทีละบล็อก/ทั้ง session ตาม design  
ระหว่างบล็อก scheduler ยัง preempt RT task ความสำคัญสูงได้

---

### Q30. Block size เลือกยังไง? ใหญ่ดีหรือเล็กดี?

**ตอบ trade-off:**  
- \(B\) เล็ก → แต่ละครั้ง IRQ-off สั้น แต่จำนวนบล็อก \(n\) เยอะ → overhead เข้า–ออก Secure / จัดการมากขึ้น  
- \(B\) ใหญ่ → baseline hash ช้ามาก (FAR พัง); Atomic/SAU ช่วยได้  

งานเราโชว์ว่าที่ \(B\) ใหญ่ซึ่งเคยพัง ตอนนี้ใช้ได้กับ RT 1000 Hz

---

## 9) คำถามจับผิด / pressure questions

### Q31. “เปิด IRQ ตอน hash แล้ว malware ย้ายออกจากบล็อกได้สิ?”

**Atomic Copy:**  
ตอน hash เรา hash **สำเนาใน Secure** แล้ว — ย้ายต้นฉบับไม่กระทบ digest ของบล็อกนี้

**SAU:**  
ระหว่าง hole บล็อกเป็น Secure — NS malware ย้าย/เขียนบล็อกนั้นไม่ได้ตาม attribution  
(ยังมี caveat DMA)

---

### Q32. “แล้วถ้า malware อยู่ใน Secure ล่ะ?”

**ตอบ:**  
นอก threat model — ถ้า Secure พัง ทุก TrustZone RA พังรวมถึง baseline SMARM บน TZ

---

### Q33. “SAU วัดแค่ 192 KB แต่พูดว่าแก้ปัญหา SMARM ทั้งระบบ?”

**ตอบ:**  
Claim คือ **กลไก + RT behavior** ของ hole remapping ไม่ใช่ “เรา attest flash ทั้งก้อนเท่า baseline ในโปรโตไทป์เดียวกัน”  
ขนาด window เป็น engineering choice เพื่อแยกจาก NS code; ขยายได้ถ้ามี region/layout รองรับ  
เปรียบเทียบหลักใช้ FAR ภายใต้ workload เดียวกัน ไม่ใช่ wall-clock รวมที่ขนาด memory คนละค่า

---

### Q34. “อะไรคือ future work?”

จำ 3 ข้อ:  
1. SAU lock policy / production TF-M integration  
2. DMA-aware (GTZC)  
3. Journal: Zephyr, energy, PRF/FPE storage reduction

---

## 10) ศัพท์ลัดที่ควรพูดให้ถูก

| ศัพท์ | ความหมายสั้น |
|---|---|
| NSC | Non-Secure Callable — ประตูเข้า Secure |
| SAU | กำหนด Secure/NS ของช่วง address (CPU) |
| IDAU | Implementation-defined attribution (คู่กับ SAU) |
| GTZC | คุม firewall ฝั่ง bus บน STM32L5 |
| FAR | ความถี่ RT task ระหว่าง attest / ตอนปกติ |
| \(t_{\mathrm{disabled}}\) | ช่วงที่ปิด interrupt |
| Roving malware | malware ย้ายหนีการวัด |
| KFV | known-future-visit — รู้ลำดับข้างหน้า (ทำลาย S3) |

---

## 11) Checklist ก่อนเข้าห้องสอบ

- [ ] วาด timing: Traditional vs SMARM vs Atomic ได้  
- [ ] วาด SAU memory map: R1 code, R6/R7 fragments, hole = Secure  
- [ ] ท่อง (S1)(S2)(S3) + ว่า Atomic/SAU รักษาอย่างไรคนละแบบ  
- [ ] จำตัวเลข \(t_{\mathrm{disabled}}\) ที่ \(B=1024\) และ \(4096\)  
- [ ] พูด limitation: DMA, SAU lock, ขนาด window SAU  
- [ ] ตอบได้ว่าทำไมไม่เทียบ runtime ดิบ SAU vs baseline  

---

## 12) ประโยคปิดท้ายถ้าอาจารย์ถาม “สรุปสั้นๆ อีกครั้ง”

> SMARM ทำให้ attestation interruptible ระหว่างบล็อกเพื่อต้าน roving malware  
> แต่บน FreeRTOS เรายังเจอว่า hash แบบ IRQ-off ต่อบล็อกทำให้ RT 1000 Hz พังเมื่อบล็อกใหญ่  
> Atomic Copy ย่อหน้าต่างปิด IRQ เหลือแค่ copy  
> SAU hole ทำให้บล็อกเป็น Secure ชั่วคราวแล้ว hash ทับที่โดยไม่ต้อง buffer  
> ทั้งคู่คืน FAR ≈ 1 โดยยังรักษา controlled execution และความลับของ \(k\) กับ \(\sigma\)
