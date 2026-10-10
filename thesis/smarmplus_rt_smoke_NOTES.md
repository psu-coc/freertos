# SMARM+ RT smoke — โน้ตก่อนสลับ branch

**วันที่:** 2026-10-10  
**Branch งานนี้:** `smarmplus-rt-smoke` (ยังมี local changes ยังไม่ commit)  
**เป้าหมายตอนกลับมา:** ทำ Option A หลัก (PWM จำลอง + parse 10 ms) เพื่อตอบ reviewer — ของจริงเป็น demo

---

## อย่าสับสนสองชั้น

| ชั้น | คืออะไร | ใช้ทำอะไร |
|------|---------|-----------|
| **Option A (หลักเปเปอร์)** | TIM3 PWM พัลส์จำลอง + task อ่านทุก 10 ms | คุมความถี่ได้ (200/1k/5k Hz), วัด miss/latency ภายใต้ attestation |
| **Demo ฮาร์ดแวร์** | Ultrasonic + จานรู optical | โชว์ระบบรถหลาย task — **ไม่แทน** ชุดตัวเลข Option A |

อาจารย์ล็อก Option A แล้ว: *simulate sensor + periodic parse*  
Accelerometer / Hall = ทางเลือกทีหลัง (Hall ซ้ำบทบาทจานรู)

---

## สิ่งที่เทสผ่านแล้วบน `smarmplus-rt-smoke`

1. PWM loopback D12↔D11 → `edges_per_s≈1000` (หลังตั้ง EXTI7 เป็น Non-Secure)
2. Ultrasonic Grove → SIG **D11 (PA7)**, VCC 3V3, GND, ขาว NC ไม่เสียบ → `distance_cm` เปลี่ยนตามมือ
3. Optical Robotlinking + จานรู 20 ช่อง → OUT **D12 (PA6)** → `[wheel] edges/s` ตามการหมุน
4. **สอง FreeRTOS task พร้อมกัน** (`PULSE_SMOKE_MODE 3`): `[sonar]` + `[wheel]`
5. แชร์ไฟผ่าน breadboard: Nucleo 3V3/GND → แถบ +/− → แยกไปทั้งสองเซ็นเซอร์ (แรงดันไม่แบ่งครึ่ง)

---

## โค้ด / TrustZone ที่แก้ไว้ (ยังไม่ commit)

ไฟล์ที่แก้บน branch นี้:

- `NonSecure/Src/main.c` — `PULSE_SMOKE_TEST`, `PULSE_SMOKE_MODE` (0..3)
- `NonSecure/Src/stm32l5xx_it.c` / `Inc/stm32l5xx_it.h` — `EXTI6` + `EXTI7`
- `Secure/Inc/partition_stm32l552xx.h` — `NVIC_INIT_ITNS0_VAL = 0x00060000` (EXTI6+EXTI7 → NS)

โหมดใน `main.c`:

- `0` = PWM loopback  
- `1` = HW speed อย่างเดียว  
- `2` = ultrasonic อย่างเดียว  
- `3` = ultrasonic + wheel (**ค่าปัจจุบันตอนหยุดงาน**)

ตอนกลับมาทำ Option A หลัก: ตั้งโหมด PWM/parse (และอาจถอดล้อจาก D12 ชั่วคราวถ้าขาชนกับ PWM)

**Flash:** เปลี่ยน partition แล้วต้อง flash **Secure ก่อน** แล้วค่อย NonSecure

---

## สายที่จำไว้

| อุปกรณ์ | สัญญาณ | ไฟ |
|---------|--------|-----|
| Ultrasonic | SIG → D11 | 3V3 + GND (ขาว NC ไม่เสียบ) |
| Optical wheel | OUT → D12 | 3V3 + GND (สกรีนเขียน 5V แต่ใช้ 3V3 ได้) |
| UART log | PuTTY | USART3 VCP 115200 |

---

## ตอนกลับจาก branch อื่น

1. `git checkout smarmplus-rt-smoke` (หรือ merge ตามที่ตกลง)  
2. ดูว่า changes ด้านบนยังอยู่หรือต้อง commit ก่อนสลับครั้งหน้า  
3. งานถัดไปเพื่อ reviewer: **Option A = PWM + parse 10 ms** (+ ทีหลัง attestation)  
4. ของจริง sonar/wheel เก็บเป็น demo / เล่า story รถ ได้ ไม่บังคับใส่ทุกกราฟ

---

## Snap/Guard (อย่าปน)

Camera-ready Snap/Guard คนละเส้น — ถ้าทำ paper ICSEC ใช้ branch/ไฟล์ tex คนละชุด อย่าสับสนกับ `smarmplus-rt-smoke`
