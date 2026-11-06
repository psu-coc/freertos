# Checklist ก่อนส่งอาจารย์รีวิว (Atomic Copy / `main.tex`)

**เป้าหมายรอบนี้:** ส่ง PDF + Overleaf ที่เล่าเรื่องเดียวกัน — baseline อ้าง SMARM+ / `smarmplus2026`, สอง enhancement, FAR / `t_disabled`, memory locking — **ไม่มี** รูปสลับ / Fig. 9 ค้าง / caption เก่า

**อัปเดต:** 2026-07-30

---

## A. รูปและ LaTeX (ทำก่อน compile)

| ลำดับ | งาน | ทำไม | สถานะ |
|------|-----|------|--------|
| **A1** | Fig. 6/7 (SAU): `sau_memory_map.pdf` ก่อน `sau_workflow.pdf`; caption memory locking (ไม่ใช่ hole-remapping) | PDF (8) สลับ caption/รูป | `[ ]` |
| **A2** | Regen `sau_workflow.pdf` (`gen_figs.py`): hole → memory locking; dashed = IRQ-off lock/unlock + DSB/ISB; hash IRQ on | ตรง Algorithm 2 | `[ ]` |
| **A3** | ลบ Fig. 9 — เอา `far_sau_heatmap.pdf` + float ออก; prose 21 config (E1) + ชี้ Fig. 8 | plan: ไม่ heatmap | `[ ]` |
| **A4** | Fig. 8: `python gen_far_fig8.py` → อัป `far_three_1000hz.pdf` | E5 | `[ ]` |
| **A5** | Fig. 4: caption FreeRTOS only (เอา Zephyr ออก) | tex ~316 | `[ ]` |
| **A6** | ตรวจ fig อื่นหลัง compile (system/workflow/timing/atomic) | รูปย้อนกลับ | `[ ]` |

---

## B. ข้อความ

| ลำดับ | งาน | สถานะ |
|------|-----|--------|
| **B1** | Intro: แก้ `\cite{}` ว่าง, Nucleo + cite; สอดคล้อง `smarmplus2026` (ไม่ claim port ใหม่ถ้า OAK ไม่ให้) | `[ ]` |
| **B2** | Merge `\ben{...}` เข้าเนื้อหา; ลบ `\oak{...}` ที่ close แล้ว; คง oak เฉพาะที่ยังรอคำตอบ | `[ ]` |
| **B3** | § Real-Time Availability: หลังลบ Fig. 9 — แก้ `\ref{fig:far_sau_all}` | `[ ]` |
| **B4** | `citation.bib` — `smarmplus2026` arXiv ID จริง หรือ note pending | `[ ]` |

---

## C. ตาราง / ตัวเลข (ถ้ามีเวลา)

| ลำดับ | งาน | plan |
|------|-----|------|
| **C1** | `tab:tdis_lock` (E2 cycles/block) — raw ใน `sau_far_raw_data.md` §E2 | tex รอ |
| **C2** | `tab:tdisabled` — baseline ตรง E3 / `gen_far_fig8.py`; Atomic E4 | |
| **C3** | ข้อความ 21/21 FAR — 128 KiB @ `0x08060000`; nuance 100/10 Hz ใน raw md | E1 done |

---

## D. Sync ก่อนส่ง

- [ ] Overleaf = repo `thesis/main.tex` + PDF รูปชุดเดียวกัน
- [ ] Compile → ดู Fig. 1–8 + caption
- [ ] บอกอาจารย์ว่า **ไม่มี Fig. 9** แล้ว

---

## ไม่จำเป็นรอบนี้

- arXiv Replace IEEEtran
- ลบ `\oak`/`\ben` แบบ camera-ready
- Zephyr / end-to-end time / heatmap ใน `gen_figs.py`

---

## ลำดับทำแนะนำ

```
A1 → A2 → A3 → A4 → A5 → compile → A6
     ↓
B1 → B3 → B2
     ↓
C1 (optional)
     ↓
D → ส่ง PDF + Overleaf
```

**สรุป:** ทดลองครบ — รอบนี้โฟกัส รูป SAU + ลบ Fig. 9 + Fig. 8, intro/caption, merge OAK/BEN, sync Overleaf.

---

ดูรายละเอียดทดลอง: `plan.md`, `sau_far_raw_data.md`.
