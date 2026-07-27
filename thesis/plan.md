# แผนทดลอง + รูป/ตาราง สำหรับ `main.tex` (Conference paper)

เอกสารนี้สรุป **(1) การทดลองที่ต้องรันบนบอร์ด** เพื่อได้ตัวเลขใส่ paper, **(2) ใส่ตรงไหนใน `main.tex`**, **(3) รูป/flowchart ใช้เดิมหรือต้องทำใหม่**  
ทำทีละข้อตามลำดับด้านล่าง — ติ๊ก `[ ]` เมื่อเสร็จ

**บอร์ด:** STM32L552ZE, 110 MHz, FreeRTOS, TrustZone-M  
**Firmware หลัก:** `Secure/Src/secure_nsc.c`, `NonSecure/Src/main.c`

---

## 0. สลับโหมดใน firmware (ก่อนรันทุกครั้ง)

| โหมด | `USE_SAU_APPROACH` (`secure_nsc.c`) | `BLOCK_SIZE` / `NS_HMAC_BLOCK_SIZE` | `TARGET_FREQ_HZ` (`main.c`) | Task ที่รัน |
|------|-------------------------------------|-------------------------------------|----------------------------|-------------|
| **Baseline SMARM** | `0` | `BLOCK_SIZE` ใน Secure | `1000` / `100` / `10` | `SMARM_Experiment_Task` |
| **Memory locking** | `1` (+ IRQ รอบ SAU open/close แล้ว) | เหมือนกัน | เหมือนกัน | `SMARM_Experiment_Task` |
| **Atomic Copy (\(t_{\mathrm{disabled}}\) copy)** | ไม่ใช้ Secure path นี้ | `NS_HMAC_BLOCK_SIZE`, memory 512 KB | — | **`NS_HashBenchmark_Task`** (ตอนนี้ยังไม่ถูกสร้างใน `main()` — ต้องเปิดใช้ชั่วคราว) |

**หมายเหตุ layout (ใช้บนบอร์ดตอนนี้):** NS app **64 KiB** `@0x08040000`–`0x0804FFFF`; **guard gap** `@0x08050000`–`0x0805FFFF` (ไม่ attest); **Memory locking \|M\|** **128 KiB** `@0x08060000`–`0x0807FFFF` (`ATTEST_DATA_BASE`, `TOTAL_SIZE=0x20000` ใน `secure_nsc.c`). อย่า remap SAU ทับ NS code ขณะ FreeRTOS รัน — จะค้าง. Baseline / Atomic อาจใช้ \|M\| ใหญ่กว่าในงานอื่น — เปรียบ FAR / \(t_{\mathrm{disabled}}\) ภายในโหมดเดียวกันเท่านั้น

---

## 1. การทดลองที่ต้องได้ตัวเลข (เรียงลำดับแนะนำ)

### E1 — Memory locking: FAR หลัง firmware IRQ fix (สำคัญสุด)

- **เป้า:** ยืนยันข้อความใน Evaluation ว่า FAR = **1.000** ทุก config หลัง IRQ-off รอบ SAU lock/unlock (+ DSB/ISB), hash ระหว่าง lock เปิด IRQ
- **รัน:** `USE_SAU=1`, sweep  
  - **B** ∈ {64, 128, 256, 512, 1024, 2048, 4096} (`BLOCK_SIZE` ใน `secure_nsc.c`)  
  - **Hz** ∈ {1000, 100, 10} (`TARGET_FREQ_HZ` ใน `main.c`) → **21 configurations**
- **ต่อ run:** 10 rounds (ตาม task), เก็บจาก UART:
  - `FAR mean/min/max`
  - `SAU_config_avg_cycles`
  - runtime mean/min/max (optional)
- **ใส่ใน paper:** §Real-Time Availability (`main.tex`), อ้าง 21 configs; raw → `thesis/sau_far_raw_data.md`

**ความคืบหน้า E1 (128 KiB window):** **21 / 21 — ปิด E1 (FAR)** — ยืนยันบนบอร์ด 2026-07-27

| สถานะ | Config (Hz × B) | FAR summary | หมายเหตุ |
|-------|-----------------|-------------|----------|
| `[x]` | ทั้ง **21** (3 Hz × 7 B) | **1.0000** ทุก config (เช็คแล้ว) | UART ละเอียดเก็บไว้ **1000×4096**; ที่เหลือสรุปใน `sau_far_raw_data.md` §E1 aggregate |
| `[x]` | **1000 × 4096** | 1.0000 | UART 10 รอบครบใน raw md |

- **สถานะ milestone:** `[x]` NSC/CMSE + SAU path — **2026-07-26**  
- **สถานะ E1 (FAR):** `[x]` raw archive **21/21** (1000 confirm + 100/10 Hz × 7 B) — **2026-07-27**  
- **ยังทำ (ไม่ใช่ E1):** `[ ]` sync `main.tex` ให้ \|M\|=128 KiB ตรง firmware (ถ้ายังเขียน 192 KB) · `[ ]` **E2** เก็บ `SAU_config_avg_cycles` ต่อ B จาก log ที่มี

**ถ้ารerun:** แก้ `BLOCK_SIZE` / `TARGET_FREQ_HZ` → Clean Build **Secure + NonSecure** → flash ทั้งคู่ → (optional) copy SUMMARY ลง raw md

### ถ้า UART ค้างหลัง `Calling SECURE_...` (debug)

| ตรวจใน IDE / บอร์ด | ค่าที่ถูก |
|---------------------|----------|
| Memory `0x08040000` | MSP NS (ไม่ใช่ `0xFFFFFFFF`) |
| Memory `0x08040004` | Reset **Thumb** เช่น `0x080405CD` |
| Memory `0x08060000` | ข้อมูล attest (มัก `0xFF` ถ้ายังไม่เขียน — hash ได้) |
| Secure `secure_nsc.c` | `ATTEST_DATA_BASE=0x08060000`, `TOTAL_SIZE=0x20000`, `USE_SAU=1` |
| `BLOCK_SIZE` @ 128 KiB | 4096 → **32** blocks; 2048 → **64**; 1024 → **128**; 512 → **256**; … |
| Serial | **ST-Link VCP (USART3)** 115200 |
| Run หลัง flash | **Secure + NonSecure**; `portALLOCATE_SECURE_CONTEXT` ก่อน NSC; debug จาก **NonSecure launch** |

UART รอบทดลอง: `SECURE_LEDToggle OK` → `Calling...` → `[NS] attest block …` / จุด `.` = NS ยังรัน → `NS: returned...` = จบรอบ | **LED1 (PC7) กระพริบ** = เข้า Secure แล้ว

---

### E2 — Memory locking: \(t_{\mathrm{disabled}}\) (DWT → Table `tab:tdis_lock`)

- **เป้า:** ตัวเลข **cycles ต่อ block** (open + restore รวม) แยกตาม **B**
- **รัน:** โหมด E1 เดียวกัน — `SAU_config_avg_cycles` จาก UART (DWT ใน `secure_nsc.c`)
- **ใส่ใน paper:** Table `\ref{tab:tdis_lock}` — **รอแก้ `main.tex` ทีเดียวหลังทดลองครบ**
- **Raw @ 1000 Hz, 7 ค่า B:** `[x]` ครบใน `sau_far_raw_data.md` §E2 (2026-07-27)
- **TIM2 ใน Secure:** ยังไม่มี → ตอนแก้ tex เลือก **B:** prose ระบุ DWT อย่างเดียว

---

### E3 — Baseline SMARM: FAR (Motivation + Fig. 8 เส้น baseline)

- **เป้า:** เส้น baseline FAR ตกเมื่อ B ≥ 2048 @ 1000 Hz (~0.27 @ 4096)
- **รัน (ถ้าต้องการ):** `USE_SAU=0`, 1000 Hz — branch **`SAU-SMARM`**
- **ใส่ใน paper:** `far_1000.pdf`, `far_three_1000hz.pdf` (`gen_far_fig8.py`)
- **สถานะ:** `[x]` **ไม่ rerun** — 2048/4096 ตกแล้ว ตรงกรaph; ใช้ค่าใน `gen_far_fig8.py`

---

### E4 — Table `tab:tdisdisabled` — Baseline vs Atomic Copy (TIM2)

- **เป้า:** Mean/std **ms** ต่อ block update, 7 ค่า B
- **Branch:** **`SAU-ATOMICCOPY`** จาก `sau-far` — ยก logic แบบเดิมไป **ฟังก์ชัน NS** (UART/ TIM2 วัดช่วง `t_disabled` / memcpy ได้)
- **Firmware:** `NS_APP_MODE_E4_ATOMIC=1` ใน `NonSecure/Src/main.c` → `NS_HashBenchmark_Task`
- **รันบอร์ด:** เปลี่ยน `NS_HMAC_BLOCK_SIZE` → rebuild **NonSecure** → flash **Secure + NonSecure** → 7 ครั้ง (64…4096)
- **Raw:** `thesis/e4_atomic_raw_data.md`
- **สถานะ:** `[x]` Atomic **7/7 B** → `e4_atomic_raw_data.md` (2026-07-27); baseline column TBD / legacy tex

---

### E5 — Atomic Copy + Memory locking: FAR @ 1000 Hz (Fig. 8)

- **เป้า:** สามเส้นใน `far_three_1000hz.pdf` — baseline (E3), atomic, memory locking (E1 @ 1000 Hz)
- **Atomic FAR:** ต้องชัดว่างานเดิมวัด FAR ระหว่าง **Secure attestation แบบ copy** หรือ proxy จาก NS — **ถ้ายังไม่มี Secure atomic FAR sweep ต้องทำหรือยืนยันกับอาจารย์**
- **Regen กราฟ:** `python gen_far_fig8.py` (ใน `thesis/`) หลังได้ CSV/ตัวเลขครบ
- **สถานะ:** `[ ]` ตัวเลขครบ `[ ]` PDF อัปเดต

---

### E6 — (ไม่บังคับใน paper นี้)

- End-to-end attestation time เปรียบ 512 KB vs 192 KB — **ไม่ใส่** (paper omit)
- Zephyr — **ไม่ทดลอง** (ไป journal / SMARM+ arXiv)
- Fig. 9 heatmap SAU — **ถอดแล้ว** ใช้ prose + E1 แทน

---

## 2. ตารางใน paper — ข้อมูลจากไหน

| Label | เนื้อหา | แหล่งตัวเลข | ทดลอง |
|-------|---------|-------------|--------|
| `tab:ra_comparison` | เปรียบเทียบแนวคิด | ไม่ต้องรัน — แก้มือ | — |
| `tab:tdisabled` | Baseline vs Atomic \(t_{\mathrm{disabled}}\) ms | TIM2 | **E4** |
| `tab:tdis_lock` | Memory locking cycles/block | DWT (`SAU_config_avg_cycles`) | **E2** |
| `tab:storage` | RAM/layout | จาก design (B, 192 KB window) | ไม่ต้องรัน |
| `tab:metric_summary` | สรุป FAR / IRQ-off / RAM | สรุปจาก E1,E3,E4 | หลัง E1–E5 |

---

## 3. รูปและ Flowchart — ใช้เดิม / อัปเดต / ทำใหม่

| ไฟล์ | Fig. label | ประเภท | แนะนำ |
|------|------------|--------|--------|
| `system-threat-model.pdf` | `fig:system_model` |  architecture | **ใช้เดิม** (`gen_figs.py`) — แก้ caption ใน tex ถ้าอาจารย์ขอ |
| `smarm_workflow.pdf` | `fig:workflow` | workflow | **ใช้เดิม** |
| `timing_diagram.png` | `fig:timing_diagram` | timing | **ใช้เดิม** — อาจเพิ่มเส้น “memory locking = lock สั้น” ในอนาคต (optional) |
| `far_1000.pdf` | `fig:setup_and_baseline_far` | plot | **อัปเดต** ถ้า E3 rerun; **แก้ caption** เอา Zephyr ออก → FreeRTOS only |
| `atomic_workflow.pdf` | `fig:atomic_copy_workflow` | flowchart | **ใช้เดิม** หรือ **วาดใหม่** ถ้าอาจารย์/คุณมี Fig 5 ใหม่ (เคยวางแผนแทน) |
| `sau_workflow.pdf` | `fig:sau_workflow` | flowchart | **ควรอัปเดต** ให้ dashed = IRQ-off **lock/unlock + DSB/ISB**, hash = IRQ on (`gen_figs.py` / วาดใหม่ Fig 7) |
| `sau_memory_map.pdf` | `fig:sau_memory_map` | memory map | **ใช้เดิม** — caption เปลี่ยนเป็น “memory locking via SAU” ใน tex แล้ว |
| `far_three_1000hz.pdf` | `fig:far_three` | plot | **อัปเดตหลัง E5** — `gen_far_fig8.py` |
| `tz_isolation.pdf` | — | (ไม่ใน `main.tex` ฉบับ advisor) | ไม่ต้องใส่ unless อาจารย์เพิ่ม Background |

**สคริปต์ gen:** `thesis/gen_figs.py`, `thesis/gen_far_fig8.py`

---

## 4. จุดใน `main.tex` ที่ผูกกับตัวเลข (ไล่ตาม section)

| Section | ต้องมีตัวเลขจาก |
|---------|------------------|
| Abstract | สรุป FAR ≈ 1.0, B≤4096, 10–1000 Hz — จาก E1,E5 |
| Motivation | `far_1000.pdf`, root cause | **E3** |
| Evaluation Setup | อธิบาย TIM2/DWT/FAR | — |
| §Non-Interruptibility | `tab:tdisabled`, `tab:tdis_lock` | **E2, E4** |
| §Real-Time Availability | `fig:far_three`, 21 configs | **E1, E5** |
| §Storage / Summary tables | ส่วนใหญ่ fixed | E1 สำหรับข้อความ FAR |

---

## 5. แผนทำทีละข้อ (ทำด้วยกัน)

| ลำดับ | งาน | ผลลัพธ์ |
|------|-----|---------|
| **1** | Flash firmware memory locking + IRQ; รัน E1 อย่างน้อย B=4096 @ 1000 Hz | `[x]` FAR=1.0 (2026-07-26) |
| **2** | ครบ E1 ทั้ง 21 config → อัป `sau_far_raw_data.md` + ข้อความใน tex | `[x]` FAR 21/21 (2026-07-27); tex sync 128 KiB ถ้ายังไม่ตรง |
| **3** | E2: สรุป DWT cycles → อัป `tab:tdis_lock` | raw 7×B @1000Hz `[x]`; **รอแก้ main.tex** |
| **4** | E3 + E5: baseline FAR + regen `far_three_1000hz.pdf` | Fig. 8 |
| **5** | E4: ยืนยัน/ rerun `tab:tdisabled` (+ เปิด NS benchmark task ถ้าต้อง) | Table III |
| **6** | รูป: อัป `sau_workflow.pdf` (lock/unlock IRQ); atomic ถ้ามี Fig ใหม่ | flowchart ตรง story |
| **7** | Polish: caption `far_1000` (ไม่มี Zephyr); sync Overleaf | ส่งอาจารย์ |

---

## 6. เก็บ log แบบมาตรฐาน (copy-paste ต่อ run)

```text
Date:
Git/firmware note:
MODE: baseline | memory_locking | atomic_copy_timing
USE_SAU_APPROACH:
BLOCK_SIZE:
TARGET_FREQ_HZ:
FAR mean / min / max:
SAU_config_avg_cycles: (locking only)
TIM2 per-update ms: (atomic/baseline if applicable)
Notes:
```

---

## 7. อ้างอิงไฟล์ใน repo

- Paper: `thesis/main.tex`
- Backup แก้เก่า: `thesis/main1.tex`, `thesis/main_backup.tex`
- Raw FAR SAU: `thesis/sau_far_raw_data.md`
- Bib SMARM+ (arXiv): `thesis/citation.bib` → `smarmplus2026` (ใส่ eprint เมื่อลง arXiv)

---

*อัปเดตล่าสุด: 2026-07-27 — E1+E2 raw ครบ; Zephyr ไม่ใส่ paper นี้*

---

## 8. Git branches (จาก `sau-far` = **base**)

| Branch | ใช้เมื่อ | Firmware โดยสรุป |
|--------|----------|-------------------|
| **`sau-far` (base)** | Memory locking + E1/E2 ครบ, 128 KiB @ `0x08060000` | `USE_SAU=1`, `SMARM_Experiment_Task` |
| **`SAU-SMARM`** (แตกจาก base) | Baseline FAR / `USE_SAU=0` ถ้าต้อง rerun หรือ debug | `#else` ใน `secure_nsc.c`: IRQ-off ตลอด HMAC update |
| **`SAU-ATOMICCOPY`** (แตกจาก base) | **E4 rerun** — `t_disabled` (TIM2), NS mirror ของ crypto + **memcpy** | ยก logic แบบเดิมไปฟังก์ชัน NS (พิมพ์ UART ได้); เปิด `NS_HashBenchmark_Task` ใน `main()` ชั่วคราว |

**อย่า merge สับสน layout:** base ใช้ 128 KiB data window; Atomic/baseline ใน paper เดิมอาจอ้าง 512 KB — เปรียบ `t_disabled` / FAR **ภายใน branch เดียวกัน**; ตอนแก้ `main.tex` อธิบายให้ตรง branch ที่วัด.

### สถานะทดลอง (ตัดสิน 2026-07-27)

| ID | รันบอร์ดอีก? | หมายเหตุ |
|----|:------------:|----------|
| **E1** | ไม่ | raw 21/21 ใน `sau_far_raw_data.md` |
| **E2** | ไม่ | DWT 7×B @ 1000 Hz ใน raw §E2 |
| **E3** | **ไม่** | Baseline FAR 2048/4096 ตกแล้ว ตรง `gen_far_fig8.py` / กรaph เดิม — ไม่ rerun sweep |
| **E4** | **Atomic done** | **`SAU-ATOMICCOPY`**: NS mirror 7×B archived; baseline optional |
| **E5** | ไม่ (กรaph) | อัป `gen_far_fig8.py` หลังตกลงตัวเลข baseline (เก่า) + E1 SAU + atomic |
| **Zephyr** | — | **ไม่ใส่** paper ฉบับนี้ |

**`main.tex`:** รอของครบ (อย่างน้อย E4 raw) แล้วแก้ทีเดียว — ยังไม่แตะตอนนี้.
