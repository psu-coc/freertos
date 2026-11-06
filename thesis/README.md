# thesis/ — จัดหมวดแล้ว (2026-10-13)

โฟลเดอร์นี้เป็นเอกสาร/เปเปอร์ ไม่ใช่ firmware  
ตอน compile LaTeX ให้รันจากโฟลเดอร์ย่อยที่มี `.tex` (เช่น `01_snapguard/`) เพราะ path รูปอยู่ข้างๆ ไฟล์ tex

## หมวด

| โฟลเดอร์ | ของอะไร |
|----------|---------|
| `01_snapguard/` | Snap/Guard ICSEC — tex, notes, รูป `tdisabled_*`, timing/threat figs |
| `02_smarmplus/` | SMARM+ / IEEE Access — tex (`SMARMplus_ver2*`, main*), oak/reviewer notes, FAR/storage figs, bib |
| `03_rt_experiment/` | สเปกทดลอง RT, HANDOFF, โน้ต smoke เซ็นเซอร์ |
| `04_scripts/` | สคริปต์ช่วย (cite/DOI/FF1 host/uart smoke) |
| `05_notes_exam/` | สรุปไทย / เตรียมสอบ |
| `06_archive/` | ไฟล์ดึง PDF / scratch / `__pycache__` |
| `07_refs_pdf/` | PDF อ้างอิงภายนอก / สำเนาเปเปอร์ |

## สถานะงานสั้นๆ

### Snap/Guard
ส่งแล้ว รอรีวิว — พักเส้นนี้ก่อน

### SMARM+ (กลับมาทำต่อ) — อยู่ตรงไหน
**Branch ปัจจุบันที่เตรียมไว้:** `smarmplus-prep`  
**แผนที่ branch โค้ด:** `../BRANCH_MAP.local.md` (gitignore)

| ชิ้น | สถานะ |
|------|--------|
| รู้ชื่อใน paper | SMARM+PRNG ≈ `bitarray*`, SMARM+FPE ≈ `ff1*` |
| ทดลองเซ็นเซอร์ NS (sonar+wheel) | บน `smarmplus-rt-smoke` commit แล้ว |
| รวมโหมด SMARMPLUS เดียว | ยังไม่ทำ — ตกลงแนวแล้ว |
| Option A: PWM + parse 10 ms | **ยังค้าง** (ตัวหลักตอบ RT / วัดซ้ำได้) |
| ของจริง ≥ 2 task | มี demo แล้ว; มอเตอร์/รถบังคับเสริม |
| ตัวที่ 3 (accel) | ทำเมื่อ 3 ไม่ยากกว่า 2 มาก |

**ถัดไปที่สมเหตุสมผล:** Option A บนโค้ด + ค่อยรวม branch โหมด PRNG/FPE

## ไฟล์สำคัญเร็วๆ

- Snap/Guard ล่าสุด: `01_snapguard/snapguard_revision.tex` (เทียบ `snapguard_overleaf_10062026.tex`)
- รูป t_disabled (ตัด B=64 แล้ว): `01_snapguard/tdisabled_three_log_1.pdf` — regen ด้วย `gen_tdisabled_fig.py` ในโฟลเดอร์เดียวกัน
- SMARM+ submitted: `02_smarmplus/SMARMplus_ver2_submitted.tex`
- OAK comments: `02_smarmplus/smarmplus_oak_comment_responses.md`
- RT spec: `03_rt_experiment/rt_experiment_spec.md`
