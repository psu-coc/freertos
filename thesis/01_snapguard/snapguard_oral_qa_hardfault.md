# Snap/Guard — Oral Q&A (อย่าใส่ใน paper)

เก็บไว้ตอบตอน present / reviewer ถามปากเปล่าเท่านั้น  
**ใน manuscript:** พูดแค่ harness ใช้ $|M|{=}128$~KiB dedicated data window — **ไม่** เล่า HardFault / ไม่เล่าว่าเคยขยายเป็น full NS 256 KiB

---

## ทำไมอาจารย์ไม่ให้ใส่ HardFault ใน paper

1. เรา**ไม่ได้ handle** HardFault เป็นส่วนของ protocol — แค่สังเกตว่า fault เกิดเมื่อ NS แตะบล็อกที่ lock  
2. ถ้าไปออกแบบ fault handler ที่ “จัดการแล้วไปต่อ” มันกลายเป็น **observable signal** ว่าบล็อกไหนถูก lock อยู่ → เสี่ยงต่อ **[S3]** (order leakage) มากกว่าเดิม  
3. พอเขียนใน paper ว่า “ลอง 256 KiB แล้ว HardFault” คนอ่านจะกระโดดไปคำถาม: **แล้ว Guard ใช้จริงกับ firmware ทั้งก้อนไม่ได้ใช่ไหม?** — เปิดช่องโจมตี narrative โดยไม่จำเป็น  
4. สารที่อยากสื่อจริงๆ คือ: Guard เหมาะกับ **dedicated Non-Secure data window**; ถ้าต้องวัด region ที่ NS ยัง execute/load อยู่ระหว่างวัด ให้ใช้ **Snap / baseline**

---

## Q: ทำไมไม่ attest ทั้ง Non-Secure bank (256 KiB)?

**คำตอบที่แนะนำ (สั้น, สมเหตุสมผล, ไม่เปิดแผล):**

> We use a fixed 128 KiB Non-Secure **data** window so all three designs run on the **same harness** and the comparison isolates only the capture mechanism (IRQ-off hash vs copy vs SAU lock). Guard is intended for a dedicated attested data region that applications do not need to fetch from during the measurement. If the deployment must attest a region that stays live/executable for Non-Secure software throughout the round, Snap or baseline SMARM is the appropriate choice.

**อย่าเปิดด้วย:** “เพราะลอง 256 แล้ว HardFault”

---

## Q: แล้วถ้าบังคับถาม — เกิดอะไรขึ้นถ้า lock ทับโค้ดที่กำลัง XIP?

**คำตอบสำรอง (เมื่อถูกกดเท่านั้น):**

> On TrustZone-M, making a currently fetched Non-Secure block Secure-by-default means a concurrent Non-Secure fetch/load to that block faults. We do **not** add application-level fault handling as part of Guard, because a handler that distinguishes “which block faulted / when” would itself be an order oracle and weaken [S3]. So the deployment rule is **layout and mechanism choice**, not fault recovery: put Guard on a data-only window, or use Snap/baseline when the live image must stay accessible.

---

## Q: แบบนี้ Guard ใช้ไม่ได้จริง?

**อย่าตอบว่า “ใช้ไม่ได้”** — ตอบเป็น **deployment niche**:

> Guard is real and useful when the attested object is a **Non-Secure data window** (e.g., staged update image, sensor/log buffer, inactive bank) and the goal is microsecond-scale $t_{\mathrm{disabled}}$ with no Secure copy buffer. It is not positioned as a drop-in for “hash the entire live XIP RTOS image while that image keeps running.” That use case is what Snap/baseline cover.

---

## Checklist ก่อน present

- [ ] Paper ไม่มีคำว่า HardFault / ไม่เล่า experiment 256 KiB fault  
- [ ] Setup พูดแค่ 128 KiB same-harness dedicated data window  
- [ ] ถ้าถูกถาม 256 → ใช้คำตอบ “fair comparison + dedicated data window” ก่อน  
- [ ] HardFault / S3 handler leakage เก็บเป็นชั้นสอง ถ้าถูกกดลึก  
