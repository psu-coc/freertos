# สรุปเปเปอร์เป็นภาษาไทย (อ่านง่าย)

ไฟล์ต้นฉบับ: `thesis/main.tex`  
ชื่อเต็ม: *Towards Fully Interruptible Shuffled Measurement for Remote Attestation*

ชื่อสั้นในเปเปอร์:
- **SMARM** = baseline เดิม (hash ทีละบล็อก ปิด interrupt ตอน hash)
- **SMARM-Snap** = แบบ copy แล้วค่อย hash
- **SMARM-Guard** = แบบใช้ SAU ทำให้บล็อก “ใช้ไม่ได้ชั่วคราว” แล้ว hash ทับที่เดิม
- **$t_{disabled}$** = ช่วงเวลาที่ปิด interrupt (งาน realtime เข้าไม่ได้)
- **FAR** = ตัวชี้ว่างาน realtime ยังทำงานถี่ตามเป้าแค่ไหน (ใกล้ 1.0 = ดี)

---

## Abstract

Remote attestation (RA) ใช้เช็กว่าซอฟต์แวร์บนอุปกรณ์ IoT ถูกแก้หรือไม่  
แต่ RA แบบดั้งเดิมมักปิด interrupt ทั้งตอน hash หน่วยความจำยาว ๆ ทำให้งาน realtime ค้างได้นานหลายร้อย ms

**SMARM** แก้โดย hash เป็นบล็อก ๆ ตามลำดับสุ่มลับ (shuffled)  
จึงเปิด interrupt **ระหว่างบล็อก** ได้ และยังต้าน *roving malware* (malware ที่ย้ายที่หนีการวัด)

แต่ช่วงปิด interrupt ของ SMARM ยังเท่ากับเวลา hash **หนึ่งบล็อก**  
ถ้าบล็อกใหญ่ ($B$ ใหญ่) ช่วงนี้ยังยาวเกินไปสำหรับงานที่ deadline แน่น

งานนี้เสนอ 2 วิธีย่อ $t_{disabled}$ ต่อ:

1. **SMARM-Snap** — ปิด interrupt แค่ตอน copy บล็อกเข้า Secure RAM แล้วค่อย hash ตอนเปิด interrupt ได้  
2. **SMARM-Guard** — ไม่ต้องมีบัฟเฟอร์ copy; ทำให้บล็อกเป้าหมาย “Non-Secure ใช้ไม่ได้ชั่วคราว” ระหว่าง hash ทับที่เดิม

ทดบน **STM32L552ZE + FreeRTOS** ทั้งสองวิธีย่อช่วงปิด interrupt ต่อบล็อกลงได้ชัด จนถึง $B=4096$ bytes

---

## 1. Introduction

IoT เยอะขึ้น แต่ MCU ถูกและมีทรัพยากรน้อย จึงป้องกัน malware แบบเครื่องใหญ่ยาก  
RA เป็นทางถูก: อุปกรณ์ (prover) hash ซอฟต์แวร์ตัวเอง ส่งผลที่ยืนยันตัวตนได้ให้ verifier เทียบกับค่าที่ถูกต้อง

RA แบบเก่าต้องวัดแบบ atomic (ปิด interrupt ทั้งรอบ)  
กัน malware ย้ายตัวหนีระหว่าง hash  
บน MCU ช่วงปิด interrupt อาจยาวเกิน ~450 ms → งาน realtime พัง deadline ได้

**SMARM** วัดทีละบล็อกตามลำดับสุ่มลับ  
malware ไม่รู้ลำดับ จึงหนีได้ยากแบบ deterministic น้อยลง แต่หนีแบบสุ่มยากขึ้น  
แลกกับการปิด interrupt แค่ต่อบล็อก ไม่ใช่ทั้งภาพ

ปัญหาที่เหลือ: $B$ ใหญ่ → hash ต่อบล็อกช้า → $t_{disabled}$ ยาว  
$B$ เล็ก → $t_{disabled}$ สั้น แต่จำนวนบล็อกเยอะ → attestation ทั้งรอบช้าและกินพลังงานมากขึ้น

### Contributions

1. เสนอ **SMARM-Snap**: ย่อ $t_{disabled}$ จาก “เวลา hash หนึ่งบล็อก” เป็น “เวลา copy หนึ่งบล็อก”  
2. เสนอ **SMARM-Guard**: ไม่ต้องใช้ Secure RAM ขนาด $B$ สำหรับ snapshot; ปิด interrupt แค่ตอนเปลี่ยน security attribute ของบล็อก  
3. implement บน Nucleo-144 (STM32L552ZE, TrustZone-M) + FreeRTOS แล้ววัดจริง

---

## 2. Background

### 2.1 RA Atomicity และ SMARM

RA โดยรวมคือ challenge–response: verifier ส่ง challenge → prover คำนวณ measurement ที่ authenticate ได้ (เช่น MAC) → verifier ตรวจ

RA แบบคลาสสิกปิด interrupt ทั้งการวัด เพื่อกัน roving malware

**SMARM** สร้าง permutation $\sigma$ จาก challenge + key $k$ แล้ววัดบล็อกตาม $\sigma$  
เปิด interrupt ระหว่างบล็อกได้

คุณสมบัติความปลอดภัยที่ต้องการ:

- **[S1] Controlled Execution** — เข้า attestation ได้แค่จุดเข้าที่กำหนด; เปิด interrupt ได้เฉพาะระหว่างบล็อก  
- **[S2] Key Secrecy** — key $k$ อยู่ในที่ malware เข้าไม่ได้  
- **[S3] Private Measurement Order** — ลำดับ $\sigma$ ลับ malware ทายไม่ได้ว่าจะวัดบล็อกไหนต่อ

งานเดิมของ SMARM วางบนสถาปัตยกรรมแบบ HYDRA

### 2.2 ARM TrustZone-M

TrustZone-M แบ่ง MCU เป็น:

- **Secure world** — โค้ด/ความลับที่ไว้ใจได้  
- **Non-Secure world** — RTOS + แอป

**IDAU** (ตายตัวตามชิป) + **SAU** (โปรแกรมได้) บอกว่าช่วง address เป็น Secure / Non-Secure / NSC  
ที่ไม่ได้ทำเครื่องหมายว่า Non-Secure จะถือเป็น **Secure by default**  
Non-Secure เรียกบริการ Secure ได้ผ่านประตู **NSC** เท่านั้น

งานก่อนหน้า (SMARM+ บน FreeRTOS/TrustZone-M) วาง:

- FreeRTOS + แอปใน Non-Secure  
- บริการ SMARM + $k$ + $\sigma$ ใน Secure  
- เรียกผ่าน NSC  

ระหว่างบล็อก RTOS สลับไปรันงาน realtime ได้  
งานนี้ใช้ชุดนี้เป็น **baseline SMARM**

---

## 3. Design

### 3.1 Motivation

ใน baseline: ปิด interrupt ทั้งตอนวัดหนึ่งบล็อก  
ดังนั้น $t_{disabled}$ ≈ เวลา hash หนึ่งบล็อก ขึ้นกับ $B$ และฟังก์ชัน hash

วัดด้วย SHA-256 บน Nucleo-144:

- $B=64$ → $t_{disabled} \approx 0.062$ ms  
- $B=4096$ → $t_{disabled} \approx 3.75$ ms  

งาน 1000 Hz มีคาบ 1 ms → ที่ $B=4096$ การวัดหนึ่งบล็อกกินได้หลายคาบ → miss deadline ได้

จุดสำคัญ: สิ่งที่ต้องการจริง ๆ คือ **temporal consistency** ของข้อมูลที่ป้อนเข้า hash  
ไม่จำเป็นต้องปิด interrupt ทั้งตอน hash ถ้าจับเนื้อหาบล็อกให้สม่ำเสมอด้วยวิธีอื่นได้

จึงเสนอ:

- **SMARM-Snap** — จับ snapshot แล้ว hash จากสำเนา  
- **SMARM-Guard** — ทำให้บล็อกใช้ไม่ได้ชั่วคราว แล้ว hash ทับที่เดิม  

รูป timing เปรียบเทียบช่วงปิด interrupt ของ RA แบบเก่า / SMARM / SMARM-Snap

### 3.2 System and Threat Model

ตามงานก่อนหน้า:

- Prover มี TrustZone-M  
- RTOS + แอปใน Non-Secure; TCB เล็กใน Secure  
- Verifier ไว้ใจได้  

ศัตรู: malware คุม Non-Secure ทั้งก้อน รวม roving malware  
Secure ไว้ใจได้  
โจมตีฮาร์ดแวร์ / แก้ SAU จากฝั่งที่ไม่น่าไว้ใจ → นอกขอบเขตงานนี้

### 3.3 SMARM-Snap

แนวคิด: copy บล็อก $M_{\sigma[i]}$ เข้า Secure RAM แบบ atomic แล้ว hash จากสำเนานั้น

ขั้นตอนต่อบล็อก:

1. ปิด interrupt  
2. `memcpy` เข้า Secure buffer  
3. เปิด interrupt  
4. HMAC update จาก buffer (interrupt เปิดได้)

ข้อดี: $t_{disabled}$ สั้นลงมาก (บน Cortex-M33 copy ~17 MB/s → สั้นกว่า baseline ประมาณ 15×)  
ข้อเสีย: ต้องมี Secure RAM ขนาด $B$

### 3.4 SMARM-Guard

แนวคิด: ไม่ copy  
ก่อน hash ปรับ SAU ให้บล็อกเป้าหมายหลุดจาก Non-Secure coverage → กลายเป็น Secure by default → Non-Secure แตะไม่ได้ชั่วคราว  
แล้ว hash ทับที่เดิมตอน interrupt เปิดได้  
จบแล้ว restore SAU ให้หน้าต่างกลับเป็น Non-Secure

ช่วงปิด interrupt = แค่ตอนโปรแกรม SAU + barrier (DSB/ISB)  
วัดได้ประมาณ **0.80–1.30 µs** (87–143 cycle ที่ 110 MHz) ต่อครั้ง lock/restore

ข้อดี: ไม่ต้องมีบัฟเฟอร์ขนาด $B$; $t_{disabled}$ สั้นมาก  
ข้อควรรู้: ตอนบล็อก “ใช้ไม่ได้ชั่วคราว” ถ้าแอป Non-Secure ไปแตะบล็อกนั้นอาจ fault/stall ได้

### 3.5 Security Guarantees

ทั้ง Snap และ Guard แก้แค่ “วิธีจับแต่ละบล็อก” ไม่เปลี่ยนโปรโตคอล SMARM  
ดังนั้น **[S2]** และ **[S3]** ยังเหมือนเดิม  
ทางเข้ายังผ่าน NSC → ส่วนใหญ่ของ **[S1]** ยังอยู่

ส่วน **[S1]** เรื่อง consistency ของการจับบล็อก:

- **Snap**: จับด้วย copy atomic ใน Secure → แม้ malware แก้ต้นฉบับทีหลังก็ไม่กระทบสำเนา  
- **Guard**: ตอน hash บล็อกถูกกันไม่ให้ Non-Secure แตะ → malware แก้ระหว่าง hash ไม่ได้

ตารางเปรียบเทียบสั้น ๆ ในเปเปอร์:

| แนวทาง | realtime-friendly | Secure RAM เพิ่ม | ขนาด $t_{disabled}$ | RTOS |
|--------|-------------------|------------------|----------------------|------|
| Traditional RA | ไม่ | 0 | ~วินาทีระดับเต็มภาพ | ไม่ |
| SMARM | บางส่วน | 0 | ~0.1–1 ms (hash ต่อบล็อก) | ไม่ (เดิม) |
| SMARM-Snap | ใช่ | ขนาด $B$ | ~0.01–0.1 ms (copy) | ใช่ |
| SMARM-Guard | ใช่ | 0 | ~0.001 ms (SAU) | ใช่ |

---

## 4. Evaluation

### 4.1 Setup

- บอร์ด: STM32 Nucleo-144, Cortex-M33, TrustZone-M, 110 MHz  
- FreeRTOS + แอปใน Non-Secure  
- งาน attested ≈ 512 KiB  
- ขนาดบล็อก $B \in \{64,128,256,512,1024,2048,4096\}$  
- เมตริกหลัก: $t_{disabled}$, Secure RAM เพิ่ม, ความพร้อม realtime (FAR)  
- เมตริกรอง: เวลา attestation ทั้งรอบ (HMAC งานรวมเท่ากับ 512 KiB)

### 4.2 Non-Interruptibility Period ($t_{disabled}$)

รูป log-scale เปรียบเทียบสามแบบ:

- Baseline: ปิด interrupt ตอน hash  
- SMARM-Snap: ปิดแค่ตอน copy → สั้นลงประมาณ **15–16×** (เมื่อ $B \ge 128$)  
- SMARM-Guard: ปิดแค่ตอน SAU lock/restore → รวมประมาณ **0.80–1.30 µs**

### 4.3 Secure Memory Overhead

- SMARM-Snap: ต้องมี Secure buffer ขนาด $B$ (สูงสุด 4 KB ที่ $B=4096$)  
- SMARM-Guard: ไม่ต้องมีบัฟเฟอร์วัดเพิ่ม; $k$ และ $\sigma$ อยู่ใน Secure เหมือน baseline

### 4.4 Real-Time Availability

ที่งาน 1000 Hz:

- Baseline SMARM: พลาด activation เมื่อ $B$ ใหญ่  
- SMARM-Snap และ SMARM-Guard: **FAR ≈ 1.0** ไปจนถึง $B=4096$

สรุป: ย่อช่วงปิด interrupt ต่อบล็อกช่วยให้งาน realtime ยัง “ถี่ตามเป้า” ได้

### 4.5 Attestation Runtime (overhead ทั้งรอบ)

สำคัญ: ย่อ $t_{disabled}$ **ไม่ได้แปลว่า** รอบ attestation ทั้งก้อนสั้นลงตามสัดส่วน  
เพราะงานหนักยังเป็น **HMAC ของ 512 KiB**

ตารางเวลาเฉลี่ย (ms) ไม่มี NormalTask คู่แข่ง, 10 rounds  
SD ของตัวอย่าง &lt; 1 ms (มักเป็น 0):

| $B$ | Baseline | SMARM-Snap | SMARM-Guard |
|----:|---------:|-----------:|------------:|
| 64 | 644 | 654 | 635 |
| 128 | 567 | 592 | 559 |
| 256 | 521 | 550 | 517 |
| 512 | 500 | 529 | 497 |
| 1024 | 490 | 519 | 487 |
| 2048 | 484 | 513 | 482 |
| 4096 | 481 | 511 | 480 |

อ่านผลแบบนี้:

- ทั้งสามแบบอยู่ในย่านเดียวกัน (~480 ms ที่ $B$ ใหญ่ ถึง ~630–650 ms ที่ $B=64$)  
- **SMARM-Snap** ช้ากว่า baseline ประมาณ **~30 ms** เพราะต้อง copy งานที่ hash  
- **SMARM-Guard** ต่างจาก baseline แค่ไม่กี่ ms (SAU เป็นระดับ µs/บล็อก)  
- เปเปอร์ **ไม่เคลมว่า Guard เร็วกว่า** baseline  
- ของที่อยากได้คือ $t_{disabled}$ สั้น ไม่ใช่รอบ attestation สั้นลง

### 4.6 Summary and Design Guidance

ตอนบล็อกถูกทำให้ใช้ไม่ได้ชั่วคราว (Guard) ถ้าแอป Non-Secure ไปแตะบล็อกนั้นอาจ fault/stall ได้  
โปรโตไทป์นี้ยังไม่ทำ recovery อัตโนมัติ → ฝากให้นักออกแบบระบบ (เช่น restore แล้วเริ่ม session ใหม่)

**เลือกแบบไหน:**

- เลือก **SMARM-Snap** เมื่อ: มี Secure RAM พอสำหรับขนาด $B$, หรือ SAU ต้องล็อกหลัง boot, หรือแยกหน้าต่างข้อมูลออกจากโค้ดยาก  
- เลือก **SMARM-Guard** เมื่อ: Secure RAM แน่น, จองหน้าต่าง Non-Secure สำหรับข้อมูลได้, และอนุญาตให้ Secure ปรับ SAU สั้น ๆ ระหว่าง attestation  

หมายเหตุ: หลายแพลตฟอร์ม production ล็อก SAU หลัง boot → ใช้ Guard ไม่ได้ถ้าไม่เปลี่ยนนโยบายนั้น  
DMA เข้า $|M|$ ยังต้องคุมแยก (เช่น GTZC)

---

## 5. Related Work

- RA แบบ atomic ยาว: SMART, VRASED, HYDRA  
- RA แบบ shuffled: SMARM, PRoM, HAtt  
- งาน companion: SMARM บน FreeRTOS/TrustZone-M เป็น baseline ของการทดลองนี้  
- Attestation ที่เกี่ยวกับ interrupt อื่น ๆ (ISC-FLAT, ASAP, IDA) — คนละโฟกัส  
- งาน timekeeping ของ RTOS ตอน Secure ยาว — คนละมุม (เขาแก้นาฬิกา; เราทำให้ช่วงปิด interrupt สั้นลง)  
- TrustZone-M/SAU มักตั้งแบบ static; งานนี้ใช้ SAU แบบสั้น ๆ ระหว่างวัด  
- ฮาร์ดแวร์พิเศษ (LO-FAT, LiteHAX, ATRIUM) — เราใช้ MCU TrustZone-M ธรรมดา

---

## 6. Conclusion

SMARM ทำให้เปิด interrupt ระหว่างบล็อกได้ แต่ยังปิด interrupt ตอน hash ต่อบล็อก → $t_{disabled}$ ยังยาวเมื่อ $B$ ใหญ่

- **SMARM-Snap** ย่อช่วงนั้นเหลือตอน copy (แลก Secure buffer ขนาด $B$ และรอบเต็มช้าขึ้นประมาณ ~30 ms)  
- **SMARM-Guard** ย่อเหลือตอนอัปเดต SAU สั้น ๆ โดยไม่ต้องมีบัฟเฟอร์ แล้ว hash ตอนเปิด interrupt ได้  

ทั้งคู่รักษา (S1)–(S3)  
เลือก Snap เมื่อมี Secure RAM หรือ SAU ต้องคงที่หลัง boot  
เลือก Guard เมื่อจองหน้าต่างข้อมูล Non-Secure ได้และอนุญาตปรับ SAU ชั่วคราว

---

## โน้ตสั้น ๆ สำหรับคุยกับอาจารย์

1. จุดขายหลักของงานคือ **$t_{disabled}$ / FAR** ไม่ใช่ “รอบ attestation เร็วขึ้น”  
2. ตาราง runtime โชว์ว่า HMAC ครองเวลา; Snap จ่ายค่า copy; Guard ใกล้ baseline  
3. บางช่อง Guard อาจน้อยกว่า baseline นิด → ข้อความในเปเปอร์บอกแล้วว่า **ไม่เคลมว่าเร็วกว่า**  
4. SD จาก 10 rounds เล็กมาก (&lt; 1 ms)
