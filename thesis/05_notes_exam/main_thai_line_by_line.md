# แปลบรรทัดต่อบรรทัด (EN → TH)

จาก `thesis/main.tex` — แปลเฉพาะเนื้อหาที่พิมพ์จริง (ข้าม `\usepackage`, comment `%`, โค้ด LaTeX จัดหน้า)  
ชื่อเฉพาะคงทับศัพท์: SMARM, SMARM-Snap, SMARM-Guard, TrustZone-M, SAU, FAR, FreeRTOS, HMAC

รูปแบบ:
- **EN:** ต้นฉบับ
- **TH:** คำแปลภาษากลาง

---

## Title

**EN:** Towards Fully Interruptible Shuffled Measurement for Remote Attestation  
**TH:** มุ่งสู่การวัดแบบ shuffled ที่เปิด interrupt ได้มากขึ้นสำหรับ Remote Attestation

---

## Abstract

**EN:** Remote attestation (RA) detects software compromise on IoT devices, but classical RA keeps interrupts disabled for a full memory (hash) measurement and can block real-time work for hundreds of milliseconds.  
**TH:** Remote attestation (RA) ใช้ตรวจว่าซอฟต์แวร์บนอุปกรณ์ IoT ถูกแก้หรือไม่ แต่ RA แบบเดิมมักปิด interrupt ทั้งตอน hash หน่วยความจำทั้งก้อน ทำให้งาน realtime ค้างได้นานหลายร้อยมิลลิวินาที

**EN:** SMARM instead measures memory in a secret shuffled order, so attestation can be interrupted between blocks while still resisting roving malware.  
**TH:** SMARM วัดหน่วยความจำตามลำดับสุ่มลับ (shuffled) จึงถูก interrupt ระหว่างบล็อกได้ และยังต้าน roving malware ได้

**EN:** As a result, SMARM still reduces the uninterruptability period from a full memory measurement to a single-block measurement.  
**TH:** ดังนั้น SMARM ย่อช่วงที่ปิด interrupt จาก “วัดทั้งภาพ” เหลือ “วัดทีละบล็อก”

**EN:** Despite a big improvement, this period can be non-negligible for a large block size, making SMARM not ideal for deadline driven settings in this scenario.  
**TH:** แม้จะดีขึ้นมาก แต่ถ้าขนาดบล็อกใหญ่ ช่วงนี้ยังยาวไม่น้อย ทำให้ยังไม่เหมาะกับงานที่มี deadline แน่น

**EN:** In this work, we propose two independent enhancements to SMARM that reduce this uninterruptability period even further.  
**TH:** งานนี้เสนอการปรับปรุง SMARM สองแบบแยกกัน เพื่อย่อช่วงปิด interrupt ให้สั้นลงอีก

**EN:** The first, called SMARM-Snap, disables interrupts only around a short memory copy in secure memory, then performs a block memory hashing with interrupts enabled.  
**TH:** แบบแรกชื่อ SMARM-Snap ปิด interrupt แค่ตอน copy หน่วยความจำสั้น ๆ เข้า secure memory แล้วค่อย hash บล็อกตอนเปิด interrupt ได้

**EN:** The second, SMARM-Guard, instead makes the target block ephemerally unavailable to Non-Secure software during an in-place hash, with no copy buffer.  
**TH:** แบบที่สอง SMARM-Guard ทำให้บล็อกเป้าหมาย “Non-Secure ใช้ไม่ได้ชั่วคราว” ระหว่าง hash ทับที่เดิม โดยไม่ต้องมีบัฟเฟอร์สำหรับ copy

**EN:** On STM32L552ZE with FreeRTOS, both substantially reduce the per-block interrupt-disabled window for block sizes up to B=4096 bytes.  
**TH:** บน STM32L552ZE กับ FreeRTOS ทั้งสองวิธีย่อช่วงปิด interrupt ต่อบล็อกได้ชัด สำหรับขนาดบล็อกถึง B=4096 bytes

**EN (keywords):** Remote attestation, IoT security, interrupt disablement, TrustZone-M, RTOS, SMARM, interrupt latency  
**TH:** Remote attestation, ความปลอดภัย IoT, การปิด interrupt, TrustZone-M, RTOS, SMARM, latency ของ interrupt

---

## 1. Introduction

**EN:** Modern societies increasingly depend on Internet-of-Things (IoT) devices across domains ranging from smart factories, where industrial sensors support monitoring and automation, to smart homes, where devices such as smart lights and robotic vacuum cleaners improve convenience.  
**TH:** สังคมสมัยนี้พึ่งอุปกรณ์ IoT มากขึ้น ทั้งโรงงานอัจฉริยะ (เซ็นเซอร์คุม/อัตโนมัติ) และบ้านอัจฉริยะ (ไฟ หุ่นดูดฝุ่น ฯลฯ)

**EN:** Since many IoT devices are designed to be inexpensive and energy-efficient, they typically operate on resource-constrained microcontrollers (MCUs) with limited RAM, flash storage, and energy budgets.  
**TH:** อุปกรณ์ IoT จำนวนมากออกแบบให้ถูกและประหยัดไฟ จึงมักรันบน MCU ที่ RAM / flash / พลังงานจำกัด

**EN:** These constraints often preclude sophisticated monolithic operating systems and the malware defenses available on higher-end devices.  
**TH:** ข้อจำกัดนี้ทำให้ใช้ OS ใหญ่ ๆ และระบบกัน malware แบบเครื่องแรง ๆ ได้ยาก

**EN:** Remote attestation (RA) has emerged as an inexpensive technique for detecting malware compromise on resource-constrained IoT devices.  
**TH:** Remote attestation (RA) จึงเป็นวิธีราคาถูกสำหรับตรวจว่า IoT แบบทรัพยากรน้อยถูก malware หรือไม่

**EN:** In RA, an IoT device, called the prover, executes a measurement routine that hashes its software image (i.e., the code stored in flash memory) and returns the measurement result in an authenticated response, e.g., using a message authentication code (MAC).  
**TH:** ใน RA อุปกรณ์ที่เรียกว่า prover จะรัน routine วัดผลด้วยการ hash ซอฟต์แวร์ใน flash แล้วส่งผลกลับแบบยืนยันตัวตนได้ เช่น ด้วย MAC

**EN:** An external trusted party, called the verifier, authenticates this response and then compares the measurement against that of the expected software image to determine whether the prover's software has been modified and hence be able to deduce whether a malware compromise happens on the prover.  
**TH:** ฝั่งที่ไว้ใจได้เรียกว่า verifier จะตรวจความถูกต้องของคำตอบ แล้วเทียบค่าวัดกับซอฟต์แวร์ที่คาดหวัง เพื่อดูว่าซอฟต์แวร์ถูกแก้หรือมี malware หรือไม่

**EN:** Traditional RA requires the measurement routine to execute atomically with interrupts disabled.  
**TH:** RA แบบเดิมต้องวัดแบบ atomic โดยปิด interrupt ทั้งช่วง

**EN:** This atomic execution is necessary to prevent roving malware from evading detection by relocating itself to a region that has already been measured.  
**TH:** จำเป็นเพื่อกัน roving malware หนีการตรวจด้วยการย้ายตัวไปโซนที่วัดไปแล้ว

**EN:** On low-end MCUs, however, the resulting uninterruptability period can exceed 450 ms.  
**TH:** แต่บน MCU ระดับล่าง ช่วงปิด interrupt อาจยาวเกิน 450 ms

**EN:** Such prolonged interrupt disablement can severely interfere with real-time applications whose correctness depends on completing time-critical tasks within their deadlines.  
**TH:** การปิด interrupt นานขนาดนี้รบกวนงาน realtime ที่ต้องทัน deadline ได้มาก

**EN:** To relax the atomicity requirement, SMARM introduces a new technique, called shuffled measurement, which permits interrupts between the measurements of individual memory blocks.  
**TH:** เพื่อผ่อนข้อบังคับ atomic SMARM เสนอ shuffled measurement ที่เปิด interrupt ระหว่างการวัดแต่ละบล็อกได้

**EN:** To defend against roving malware, SMARM measures the blocks comprising the software image in a random and secret order.  
**TH:** เพื่อต้าน roving malware SMARM วัดบล็อกของซอฟต์แวร์ตามลำดับสุ่มและลับ

**EN:** Since malware does not know which blocks have already been measured, it cannot reliably relocate itself to an inspected block to evade detection.  
**TH:** เพราะ malware ไม่รู้ว่าบล็อกไหนวัดไปแล้ว จึงย้ายหนีไปบล็อกที่วัดแล้วแบบชัวร์ ๆ ไม่ได้

**EN:** Although this approach does not provide the deterministic protection of fully atomic measurement, it makes evasion probabilistically difficult while reducing the uninterruptability period from the measurement of the entire software image to that of a single block.  
**TH:** แม้จะไม่แข็งเท่าการวัด atomic ทั้งก้อน แต่ทำให้การหนียากขึ้นแบบความน่าจะเป็น และย่อช่วงปิด interrupt จากทั้งภาพเหลือหนึ่งบล็อก

**EN:** Despite this improvement, SMARM's maximum uninterruptability period remains determined by its block size.  
**TH:** แม้จะดีขึ้น ช่วงปิด interrupt สูงสุดของ SMARM ยังขึ้นกับขนาดบล็อก

**EN:** Prior work shows that using larger blocks can reduce the total attestation time by lowering per-block processing overhead.  
**TH:** งานก่อนหน้าบอกว่าใช้บล็อกใหญ่ช่วยลดเวลา attestation รวม เพราะ overhead ต่อบล็อกลดลง

**EN:** However, a larger block also takes longer to measure, thereby extending each uninterruptible period and potentially delaying time-critical tasks beyond their deadlines.  
**TH:** แต่บล็อกใหญ่ก็วัดนานขึ้น ทำให้ช่วงปิด interrupt แต่ละครั้งยาวขึ้น และอาจทำให้งานสำคัญพลาด deadline

**EN:** Conversely, smaller blocks shorten these intervals but increase the number of blocks and the associated computational overhead, resulting in longer attestation times and greater energy consumption.  
**TH:** กลับกัน บล็อกเล็กทำให้ช่วงสั้นลง แต่จำนวนบล็อกและ overhead เพิ่ม → attestation ทั้งรอบช้าลงและกินพลังงานมากขึ้น

**EN:** SMARM therefore exposes an inherent tradeoff between real-time responsiveness and energy efficiency, which is particularly problematic for battery-powered IoT devices.  
**TH:** ดังนั้น SMARM มี tradeoff โดยธรรมชาติระหว่างความตอบสนอง realtime กับประสิทธิภาพพลังงาน ซึ่งหนักสำหรับ IoT ที่ใช้แบต

**EN:** Motivated by this observation, we propose two independent enhancements to shuffled measurement that further shorten its uninterruptability periods, allowing the technique to coexist with a broader range of time-sensitive applications.  
**TH:** จากจุดนี้ เราเสนอการปรับ shuffled measurement สองแบบแยกกัน เพื่อย่อช่วงปิด interrupt ให้สั้นลงอีก และใช้ร่วมกับงาน time-sensitive ได้กว้างขึ้น

### Contributions

**EN:** We propose SMARM-Snap, which reduces each uninterruptability period from the time required to hash one memory block to the substantially shorter time required to take a snapshot of (i.e., perform a memory copy) that block.  
**TH:** เราเสนอ SMARM-Snap ที่ย่อช่วงปิด interrupt จากเวลา hash หนึ่งบล็อก เหลือเวลา copy (snapshot) หนึ่งบล็อกซึ่งสั้นกว่ามาก

**EN:** SMARM-Snap incurs additional secure memory to copy a memory into. To relax this constraint, we propose SMARM-Guard, which uses interrupt disablement only for temporarily changing the security attribute of the memory block without copying its memory.  
**TH:** SMARM-Snap ต้องใช้ Secure memory เพิ่มสำหรับ copy เพื่อผ่อนข้อนี้ เราเสนอ SMARM-Guard ที่ปิด interrupt แค่ตอนเปลี่ยน security attribute ของบล็อกชั่วคราว โดยไม่ copy เนื้อหา

**EN:** As an additional benefit, SMARM-Guard further reduces the uninterruptability period, at the cost of introducing a probability of interference with concurrent application memory accesses.  
**TH:** ข้อดีเพิ่มของ SMARM-Guard คือย่อช่วงปิด interrupt ได้สั้นลงอีก แต่แลกกับโอกาสไปชนกับการเข้าถึงหน่วยความจำของแอปที่รันพร้อมกัน

**EN:** We implement SMARM-Snap and SMARM-Guard on an STM32 Nucleo-144 board (STM32L552ZE, Cortex-M33 with TrustZone-M) running FreeRTOS and evaluate them on real hardware. Our results demonstrate that both mechanisms substantially reduce SMARM's non-interruptible periods, thereby improving their compatibility with time-sensitive embedded applications.  
**TH:** เราทำ SMARM-Snap และ SMARM-Guard บน Nucleo-144 (STM32L552ZE, Cortex-M33 + TrustZone-M) กับ FreeRTOS แล้ววัดบนฮาร์ดแวร์จริง ผลแสดงว่าทั้งสองวิธีย่อช่วงปิด interrupt ของ SMARM ได้ชัด และเข้ากับงาน embedded ที่ time-sensitive ได้ดีขึ้น

---

## 2. Background

### 2.1 RA Atomicity and SMARM

**EN:** At a high level, remote attestation (RA) is a challenge–response protocol between a prover (i.e., a resource-constrained IoT device in this work) and a trusted verifier.  
**TH:** โดยรวม RA เป็นโปรโตคอล challenge–response ระหว่าง prover (IoT ทรัพยากรน้อยในงานนี้) กับ verifier ที่ไว้ใจได้

**EN:** The verifier sends a fresh challenge (e.g., a random nonce); the prover computes an authenticated integrity measurement, typically using a message authentication code (MAC), over its software image together with that challenge and a pre-shared attestation key k and returns it as attestation evidence.  
**TH:** verifier ส่ง challenge ใหม่ (เช่น nonce) → prover คำนวณค่าวัด integrity ที่ authenticate ได้ (มักเป็น MAC) จากซอฟต์แวร์ + challenge + key k ที่แชร์ล่วงหน้า แล้วส่งเป็นหลักฐาน

**EN:** The verifier then checks the evidence using the verification key against a reference value for the expected software state.  
**TH:** verifier ตรวจหลักฐานด้วย verification key เทียบกับค่าอ้างอิงของซอฟต์แวร์ที่ควรเป็น

**EN:** Classical RA techniques often disables interrupts for the whole measurement so that roving malware cannot move itself while memory is being hashed.  
**TH:** RA แบบคลาสสิกมักปิด interrupt ทั้งการวัด เพื่อกัน roving malware ย้ายตัวระหว่าง hash

**EN:** This makes those techniques undesirable for time-sensitive or deadline-driven settings.  
**TH:** จึงไม่เหมาะกับงานที่ไวต่อเวลาหรือมี deadline

**EN:** To address this, SMARM instead derives a one-time permutation σ from the verifier challenge and an attestation key k, then measures blocks M_σ(i) one by one, re-enabling interrupts between blocks.  
**TH:** SMARM จึงสร้าง permutation σ แบบใช้ครั้งเดียวจาก challenge + key k แล้ววัดบล็อก M_σ(i) ทีละบล็อก พร้อมเปิด interrupt ระหว่างบล็อก

**EN:** The security claim is not “malware cannot interrupt”; it is that malware without knowledge of the remaining secret order, malware cannot reliably relocate to escape detection, and repeated independent sessions drive the escape probability down.  
**TH:** ข้ออ้างด้านความปลอดภัยไม่ใช่ “malware interrupt ไม่ได้” แต่คือ malware ที่ไม่รู้ลำดับลับที่เหลือ จะย้ายหนีแบบชัวร์ไม่ได้ และการรันหลาย session อิสระจะลดโอกาสหนีลง

**EN:** SMARM requires following security properties:  
**TH:** SMARM ต้องการคุณสมบัติด้านความปลอดภัยดังนี้:

**EN [S1] Controlled Execution:** The attestation function (i.e., the authenticated integrity and measurement routines) is reachable only through defined entry/exit points. Interrupts may be re-enabled only between block measurements so malware cannot relocate itself during a block measurement.  
**TH [S1] Controlled Execution:** ฟังก์ชัน attestation เข้าได้แค่จุดเข้า/ออกที่กำหนด เปิด interrupt ได้เฉพาะระหว่างบล็อก เพื่อกัน malware ย้ายตัวระหว่างวัดบล็อก

**EN [S2] Key Secrecy:** The attestation key k is stored in memory inaccessible to untrusted software, so malware cannot obtain it to directly forge a valid report without triggering the attestation routine.  
**TH [S2] Key Secrecy:** key k อยู่ในหน่วยความจำที่ซอฟต์แวร์ไม่ไว้ใจเข้าไม่ได้ จึงปลอมรายงานที่ถูกต้องโดยไม่ผ่าน attestation ไม่ได้ง่าย ๆ

**EN [S3] Private Measurement Order:** The permutation σ is stored in memory inaccessible to malware such that the malware cannot predict which remaining blocks will be measured next and relocate before/after each block measurement.  
**TH [S3] Private Measurement Order:** permutation σ อยู่ในที่ malware เข้าไม่ได้ จึงทายไม่ได้ว่าจะวัดบล็อกไหนต่อเพื่อย้ายหนีก่อน/หลังแต่ละบล็อก

**EN:** The original SMARM's design builds atop HYDRA attestation architecture to satisfy these properties.  
**TH:** ดีไซน์เดิมของ SMARM สร้างบนสถาปัตยกรรม attestation แบบ HYDRA เพื่อให้ได้คุณสมบัติเหล่านี้

### 2.2 ARM TrustZone-M

**EN:** ARM TrustZone-M for Cortex-M partitions the MCU into a Secure world (trusted code and secrets) and a Non-Secure world (RTOS and applications).  
**TH:** ARM TrustZone-M บน Cortex-M แบ่ง MCU เป็น Secure world (โค้ด/ความลับที่ไว้ใจได้) กับ Non-Secure world (RTOS และแอป)

**EN:** The IDAU (fixed) and programmable SAU attribute each address range as Secure, Non-Secure, or Non-Secure Callable (NSC); memory not explicitly marked Non-Secure is Secure by default, and Non-Secure software may reach Secure services only through NSC entry points.  
**TH:** IDAU (ตายตัว) และ SAU (โปรแกรมได้) กำหนดว่าช่วง address เป็น Secure / Non-Secure / NSC; ที่ไม่ได้ทำเครื่องหมายว่า Non-Secure จะเป็น Secure by default; Non-Secure เรียกบริการ Secure ได้ผ่านประตู NSC เท่านั้น

**EN:** Our target devices provide eight programmable SAU regions.  
**TH:** อุปกรณ์เป้าหมายของเรามี SAU region ที่โปรแกรมได้ 8 อัน

**EN:** Recent work adapts SMARM to coexist with an RTOS (e.g., FreeRTOS) while leveraging TrustZone-M to preserve SMARM's security guarantees.  
**TH:** งานล่าสุดปรับ SMARM ให้ใช้ร่วมกับ RTOS (เช่น FreeRTOS) โดยอาศัย TrustZone-M เพื่อคงการันตีความปลอดภัยของ SMARM

**EN:** In this design, FreeRTOS and application tasks execute in the Non-Secure world, while the SMARM service, attestation key k, and secret permutation σ reside in the Secure world.  
**TH:** ในดีไซน์นี้ FreeRTOS และงานแอปอยู่ Non-Secure ส่วนบริการ SMARM, key k และ permutation σ อยู่ Secure

**EN:** A Non-Secure wrapper (as a real-time task) may invoke the SMARM service through an NSC entry point, as illustrated in Fig. (system model).  
**TH:** wrapper ฝั่ง Non-Secure (เป็น realtime task) เรียกบริการ SMARM ผ่านประตู NSC ตามรูป system model

**EN:** During attestation, interrupts are enabled between consecutive block measurements, allowing the RTOS to schedule other tasks, including high-priority real-time workloads.  
**TH:** ระหว่าง attestation เปิด interrupt ระหว่างบล็อกได้ ทำให้ RTOS สลับไปรันงานอื่น รวมงาน realtime สำคัญได้

**EN:** We use this TrustZone-M-based SMARM design as the baseline for this work.  
**TH:** เราใช้ดีไซน์ SMARM บน TrustZone-M นี้เป็น baseline ของงานนี้

**EN (Fig. caption):** SMARM on FreeRTOS with TrustZone-M, as proposed in [smarmplus2026].  
**TH:** SMARM บน FreeRTOS กับ TrustZone-M ตามที่เสนอในงาน companion

---

## 3. Design

### 3.1 Motivation

**EN:** In the SMARM baseline, interrupts must be disabled throughout the measurement of each memory block.  
**TH:** ใน baseline SMARM ต้องปิด interrupt ทั้งตอนวัดแต่ละบล็อก

**EN:** Consequently, the non-interruptibility period (t_disabled), during which real-time tasks cannot execute, is determined by the time required to measure a single block.  
**TH:** ดังนั้นช่วงปิด interrupt (t_disabled) ที่งาน realtime รันไม่ได้ ถูกกำหนดด้วยเวลาวัดหนึ่งบล็อก

**EN:** This duration depends primarily on two factors: the block size (B) and the measurement function.  
**TH:** เวลานี้ขึ้นกับสองอย่างหลัก: ขนาดบล็อก B และฟังก์ชันที่ใช้วัด

**EN:** Following prior work, we investigated this issue using SHA-256 as the underlying measurement function.  
**TH:** ตามงานก่อนหน้า เราดูปัญหานี้โดยใช้ SHA-256 เป็นฟังก์ชันวัด

**EN:** On an STM32 Nucleo-144 board, we observe that t_disabled increases with B, from 0.062 ms at B=64 bytes to 3.75 ms at B=4096 bytes.  
**TH:** บน Nucleo-144 พบว่า t_disabled โตตาม B จาก 0.062 ms ที่ B=64 ถึง 3.75 ms ที่ B=4096

**EN:** For comparison, a high-frequency real-time task running at 1000 Hz has a period of only 1 ms.  
**TH:** เทียบแล้ว งาน realtime 1000 Hz มีคาบแค่ 1 ms

**EN:** Thus, with B=4096 bytes, a single SMARM block measurement can span several task periods and cause the real-time task to miss multiple deadlines.  
**TH:** ดังนั้นที่ B=4096 การวัดหนึ่งบล็อกของ SMARM อาจกินหลายคาบงาน และทำให้งาน realtime พลาด deadline หลายครั้งได้

**EN:** This makes the baseline SMARM unsuitable for such time-sensitive workloads (as also observed in prior FreeRTOS/TrustZone-M SMARM work).  
**TH:** ทำให้ baseline SMARM ไม่เหมาะกับงานที่ time-sensitive แบบนี้ (งาน SMARM บน FreeRTOS/TrustZone-M ก่อนหน้าก็เห็นเช่นกัน)

**EN:** In this work, we observe that t_disabled can be reduced without compromising SMARM's functionality or security guarantees.  
**TH:** ในงานนี้เราสังเกตว่า t_disabled ย่อได้โดยไม่ทำลายฟังก์ชันหรือการันตีความปลอดภัยของ SMARM

**EN:** In particular, we notice that the security of each block measurement (in [S1]) inherently requires a temporal consistency property in which the measurement result must faithfully correspond to a consistent state of the measured block at some point during the measurement.  
**TH:** โดยเฉพาะ ความปลอดภัยของการวัดแต่ละบล็อกใน [S1] ต้องการ temporal consistency คือผลวัดต้องตรงกับสถานะของบล็อกที่สม่ำเสมอ ณ จุดใดจุดหนึ่งระหว่างการวัด

**EN:** The SMARM baseline achieves this property by measuring each block atomically with interrupts disabled, thereby preventing untrusted software from modifying the block while its contents are being hashed.  
**TH:** baseline SMARM ได้คุณสมบัตินี้ด้วยการวัดบล็อกแบบ atomic ปิด interrupt กันซอฟต์แวร์ไม่ไว้ใจแก้บล็อกระหว่าง hash

**EN:** However, disabling interrupts throughout the entire measurement is not the only way to fulfill this property.  
**TH:** แต่การปิด interrupt ทั้งตอนวัดไม่ใช่ทางเดียวที่จะได้คุณสมบัตินี้

**EN:** The key requirement is instead to ensure a consistent capture of the block contents that serve as input to the hash.  
**TH:** สิ่งที่ต้องได้จริงคือ “จับ” เนื้อหาบล็อกให้สม่ำเสมอเป็นอินพุตของ hash

**EN:** Based on this observation, we propose two alternative mechanisms, SMARM-Snap and SMARM-Guard, that preserve such consistency while substantially shortening the non-interruptibility period and allowing the hash computation itself to remain interruptible.  
**TH:** จากจุดนี้เราเสนอ SMARM-Snap และ SMARM-Guard ที่คง consistency นี้ไว้ แต่ย่อช่วงปิด interrupt ได้มาก และให้ขั้นตอน hash เองเปิด interrupt ได้

**EN (Fig. caption):** Interrupt-disabled windows. Traditional RA covers the full scan. SMARM limits disability to one block hash. SMARM-Snap further limits it to one block copy.  
**TH:** ช่วงปิด interrupt — RA แบบเดิมครอบทั้งสแกน; SMARM จำกัดเหลือ hash หนึ่งบล็อก; SMARM-Snap จำกัดเหลือ copy หนึ่งบล็อก

### 3.2 System and Threat Model

**EN:** We follow the system and threat models of prior work.  
**TH:** เราใช้ system/threat model ตามงานก่อนหน้า

**EN:** The prover is a resource-constrained IoT device equipped with a TrustZone-M TEE.  
**TH:** prover คือ IoT ทรัพยากรน้อยที่มี TrustZone-M TEE

**EN:** It runs a commodity RTOS and application tasks in the Non-Secure world, while a small Trusted Computing Base (TCB), including the attestation service and its security-critical state, resides in the Secure world.  
**TH:** รัน RTOS ทั่วไปและงานแอปใน Non-Secure ส่วน TCB เล็ก (บริการ attestation และ state สำคัญ) อยู่ใน Secure

**EN:** The remote verifier is trusted, aiming to determine the integrity of the prover's software while minimizing interference with real-time tasks scheduled to run intermittently on the prover device.  
**TH:** verifier ไว้ใจได้ ต้องการรู้ integrity ของซอฟต์แวร์ prover โดยรบกวนงาน realtime บนอุปกรณ์ให้น้อยที่สุด

**EN:** We consider a software-level adversary (i.e., malware) with full control over the Non-Secure world, including the RTOS and applications.  
**TH:** ศัตรูระดับซอฟต์แวร์ (malware) คุม Non-Secure ทั้งก้อน รวม RTOS และแอป

**EN:** In particular, the adversary may employ roving malware that relocates its malicious code or data during an ongoing attestation in an attempt to evade measurement.  
**TH:** โดยเฉพาะอาจใช้ roving malware ที่ย้ายโค้ด/ข้อมูลระหว่าง attestation เพื่อหนีการวัด

**EN:** The Secure world and its TCB are trusted; thus, the adversary cannot compromise or modify code, data, or secrets residing in Secure memory.  
**TH:** Secure world และ TCB ไว้ใจได้ ดังนั้นศัตรูแก้โค้ด/ข้อมูล/ความลับใน Secure ไม่ได้

**EN:** We consider hardware attacks, including attacks that directly manipulate the hardware-enforced security attribution provided by the SAU/IDAU, to be out of scope.  
**TH:** โจมตีฮาร์ดแวร์ รวมการเล่นกับ SAU/IDAU โดยตรง อยู่นอกขอบเขตงานนี้

**EN (Fig. caption):** Shared SMARM loop (left) with three per-block measurement options (right). Red dashed regions mark interrupt-disabled steps. Baseline SMARM hashes under interrupts disabled; SMARM-Snap copies under interrupts disabled then hashes with interrupts enabled; SMARM-Guard makes the selected block unavailable to Non-Secure software via the SAU, hashes in place with interrupts enabled, then restores Non-Secure availability.  
**TH:** ลูป SMARM ร่วมกัน (ซ้าย) กับ 3 ทางเลือกต่อบล็อก (ขวา) กรอบแดงประ = ช่วงปิด interrupt — Baseline hash ตอนปิด interrupt; SMARM-Snap copy ตอนปิดแล้ว hash ตอนเปิด; SMARM-Guard ทำให้บล็อกใช้ไม่ได้ชั่วคราวด้วย SAU แล้ว hash ทับที่เดิมตอนเปิด จากนั้นคืน Non-Secure

### 3.3 SMARM-Snap

**EN:** Our first enhancement achieves temporal consistency by taking an atomic snapshot of the shuffled memory block to be measured (M_σ[i]) and storing the snapshot in Secure-world RAM, where it is inaccessible to untrusted Non-Secure software.  
**TH:** แบบแรกได้ temporal consistency ด้วยการ snapshot บล็อกที่จะวัด (M_σ[i]) แบบ atomic แล้วเก็บใน Secure RAM ที่ Non-Secure ไม่ไว้ใจเข้าไม่ได้

**EN:** The subsequent measurement is then performed over this protected copy. We refer to this enhancement as SMARM-Snap.  
**TH:** จากนั้นวัดจากสำเนานี้ เราเรียกวิธีนี้ว่า SMARM-Snap

**EN:** As illustrated in Fig. (combined workflow), SMARM-Snap follows the same overall workflow as the SMARM baseline and differs only in the per-block measurement operation.  
**TH:** ตามรูป workflow รวม SMARM-Snap ใช้ workflow รวมแบบเดียวกับ baseline ต่างแค่ขั้นตอนวัดต่อบล็อก

**EN:** Rather than disabling interrupts for the entire measurement of M_σ[i], SMARM-Snap disables interrupts only while copying M_σ[i] into a Secure-world RAM.  
**TH:** แทนที่จะปิด interrupt ทั้งตอนวัด M_σ[i] SMARM-Snap ปิดแค่ตอน copy M_σ[i] เข้า Secure RAM

**EN:** Since the copy is performed atomically, the resulting snapshot corresponds to a consistent state of M_σ[i].  
**TH:** เพราะ copy แบบ atomic สำเนาที่ได้จึงตรงกับสถานะของ M_σ[i] ที่สม่ำเสมอ

**EN:** Once the snapshot is complete, interrupts can be re-enabled and the measurement function can operate on the secure copy without interference from untrusted software.  
**TH:** เมื่อ snapshot เสร็จ เปิด interrupt ได้ และฟังก์ชันวัดทำงานบนสำเนาใน Secure โดยไม่ถูกซอฟต์แวร์ไม่ไว้ใจรบกวน

**EN:** The direct benefit of SMARM-Snap is that it substantially shortens the non-interruptibility period, since t_disabled is reduced from the time required to hash a block to the time required only to copy it.  
**TH:** ประโยชน์ตรง ๆ คือย่อช่วงปิด interrupt ได้มาก เพราะ t_disabled เหลือแค่เวลา copy ไม่ใช่เวลา hash ทั้งบล็อก

**EN:** On our Cortex-M33 target, the memory-copy throughput is around 17 MB/s, giving t_disabled^copy ≈ 17 MB/s, which is 15× shorter than the corresponding t_disabled of the SMARM baseline.  
**TH:** บน Cortex-M33 ของเรา ความเร็ว copy ประมาณ 17 MB/s ทำให้ t_disabled ของ copy สั้นกว่า baseline ประมาณ 15 เท่า  
*(หมายเหตุ: ใน tex ต้นฉบับสูตรเขียนคล้าย throughput; ความหมายที่ตั้งใจคือเวลา copy สั้นกว่า hash ประมาณ 15×)*

**EN:** The main drawback of SMARM-Snap is its additional memory requirement since it requires a Secure-world storage of size B to hold the snapshot.  
**TH:** ข้อเสียหลักคือต้องมี Secure storage ขนาด B สำหรับ snapshot

**EN:** Our second enhancement, presented next, eliminates this additional storage.  
**TH:** แบบที่สองในหัวข้อถัดไปตัด storage เพิ่มนี้ออก

### 3.4 SMARM-Guard

**EN:** While SMARM-Snap achieves temporal consistency by copying each block into a Secure-world buffer, this requires an additional B bytes of Secure RAM.  
**TH:** SMARM-Snap ได้ consistency จาก copy แต่ต้องใช้ Secure RAM เพิ่ม B bytes

**EN:** Our second enhancement, SMARM-Guard, avoids this storage overhead by instead guarding the memory block being measured, M_σ[i], by making it ephemerally unavailable to untrusted software.  
**TH:** SMARM-Guard เลี่ยง overhead นี้ด้วยการ “กัน” บล็อกที่กำลังวัด M_σ[i] ให้ซอฟต์แวร์ไม่ไว้ใจใช้ไม่ได้ชั่วคราว

**EN:** This allows the SMARM Service to measure the block in place while interrupts remain enabled.  
**TH:** ทำให้บริการ SMARM วัดทับที่เดิมได้ขณะเปิด interrupt ได้

**EN:** In our target system, we leverage the SAU to realize this mechanism.  
**TH:** ในระบบเป้าหมายเราใช้ SAU ทำให้กลไกนี้เกิดขึ้นจริง

**EN:** As shown in Fig. (combined workflow), before measuring M_σ[i], SMARM-Guard reconfigures the SAU regions covering the attested memory so that M_σ[i] is excluded from Non-Secure coverage.  
**TH:** ตามรูป ก่อนวัด M_σ[i] SMARM-Guard ปรับ SAU ที่ครอบหน่วยความจำ attested ให้ M_σ[i] หลุดจาก Non-Secure coverage

**EN:** The SAU configurations before and during block measurement are shown in Figure (sau memory map).  
**TH:** การตั้ง SAU ก่อนและระหว่างวัดบล็อกดูได้จากรูป sau memory map

**EN:** Since memory not explicitly attributed as Non-Secure is considered to be Secure by default in TrustZone-M, this effectively makes M_σ[i] accessible only from the Secure world.  
**TH:** เพราะที่ไม่ได้ทำเครื่องหมาย Non-Secure จะเป็น Secure by default ใน TrustZone-M จึงทำให้ M_σ[i] เข้าได้จาก Secure เท่านั้น

**EN:** The SAU update is followed by data and instruction synchronization barriers (DSB and ISB) to ensure that the new memory-security attribution takes effect immediately.  
**TH:** หลังอัปเดต SAU มี barrier DSB และ ISB เพื่อให้ attribution ใหม่มีผลทันที

**EN:** To preserve temporal consistency, the SAU reconfiguration and the associated barriers must execute atomically with interrupts disabled.  
**TH:** เพื่อคง temporal consistency การปรับ SAU และ barrier ต้องรันแบบ atomic โดยปิด interrupt

**EN:** However, this atomic operation is substantially shorter than either hashing or copying the entire block.  
**TH:** แต่ขั้นตอน atomic นี้สั้นกว่าการ hash หรือ copy ทั้งบล็อกมาก

**EN:** In our measurements, each SAU attribution update takes only 0.80-1.30 µs (87-143 cycles at 110 MHz).  
**TH:** จากที่วัด แต่ละครั้งอัปเดต SAU ใช้เวลาแค่ประมาณ 0.80–1.30 µs (87–143 cycle ที่ 110 MHz)

**EN:** Once the block has been guarded, interrupts are re-enabled and the SMARM service measures M_σ[i] directly.  
**TH:** เมื่อกันบล็อกแล้ว เปิด interrupt และบริการ SMARM วัด M_σ[i] โดยตรง

**EN:** During this period, Non-Secure software may continue executing but cannot access or modify the protected block.  
**TH:** ช่วงนี้ Non-Secure ยังรันต่อได้ แต่เข้าถึงหรือแก้บล็อกที่ถูกกันไม่ได้

**EN:** After the measurement completes, the SMARM Service briefly disables interrupts again to restore the original SAU configuration to make M_σ[i] Non-Secure, and then proceeds to the next shuffled block M_σ[i+1].  
**TH:** หลังวัดเสร็จ ปิด interrupt สั้น ๆ อีกครั้งเพื่อคืน SAU เดิมให้ M_σ[i] เป็น Non-Secure แล้วไปบล็อกถัดไปตาม shuffle

**EN:** Thus, SMARM-Guard reduces each non-interruptibility period to a short SAU reconfiguration operation while keeping the substantially longer measurement computation interruptible.  
**TH:** ดังนั้น SMARM-Guard ย่อช่วงปิด interrupt แต่ละครั้งเหลือแค่ปรับ SAU สั้น ๆ ส่วนงานวัดที่ยาวกว่าเปิด interrupt ได้

**EN:** Unlike SMARM-Snap, it also requires no additional Secure-world buffer of size B.  
**TH:** ไม่เหมือน SMARM-Snap คือไม่ต้องมี Secure buffer ขนาด B เพิ่ม

**EN (Fig. caption):** Non-Secure attribution of the attested flash before and during measurement. At rest, |M| is fully Non-Secure; while M_σ(i) is measured, two dynamic SAU regions leave the B-byte block uncovered (Secure by default). Interrupts are off only while the SAU is reprogrammed.  
**TH:** การตั้ง Non-Secure ของ flash ที่ attested ก่อน/ระหว่างวัด — พัก ๆ |M| เป็น Non-Secure ทั้งก้อน; ตอนวัด M_σ(i) มี SAU สอง region แบบ dynamic ทิ้งบล็อก B bytes ให้ไม่ถูกครอบ (Secure by default); ปิด interrupt แค่ตอนโปรแกรม SAU

### 3.5 Security Guarantees

**EN:** Both SMARM-Snap and SMARM-Guard modify only how each shuffled memory block is measured; they do not alter the underlying SMARM protocol.  
**TH:** ทั้ง SMARM-Snap และ SMARM-Guard แก้แค่วิธีวัดแต่ละบล็อก ไม่เปลี่ยนโปรโตคอล SMARM หลัก

**EN:** In particular, the attestation key k and secret permutation σ are generated, used, and stored exactly as in the SMARM baseline. Therefore, [S2] Key Secrecy and [S3] Private Measurement Order continue to hold.  
**TH:** โดยเฉพาะ key k และ permutation σ สร้าง/ใช้/เก็บเหมือน baseline ดังนั้น [S2] และ [S3] ยังอยู่

**EN:** Likewise, both designs preserve SMARM's controlled execution interface, where the attestation functionality remains within the Secure-world SMARM Service and can be invoked from the Non-Secure world only through its designated NSC entry point.  
**TH:** เช่นกัน ทั้งสองแบบคงอินเทอร์เฟซ controlled execution: ฟังก์ชัน attestation อยู่ใน Secure SMARM Service และเรียกจาก Non-Secure ได้แค่ผ่านประตู NSC ที่กำหนด

**EN:** The main change concerns [S1] Controlled Execution.  
**TH:** จุดที่เปลี่ยนหลักเกี่ยวกับ [S1] Controlled Execution

**EN:** In the SMARM baseline, interrupts are disabled throughout the measurement of each block, preventing malware from modifying or relocating the block while it is being hashed.  
**TH:** ใน baseline ปิด interrupt ทั้งตอนวัดแต่ละบล็อก กัน malware แก้หรือย้ายบล็อกระหว่าง hash

**EN:** Our designs shorten this non-interruptability period while preserving the same requirement as follows:  
**TH:** ดีไซน์ของเราย่อช่วงปิด interrupt นี้ แต่ยังรักษาความต้องการเดิมดังนี้:

**EN (Snap):** SMARM-Snap achieves this by disabling interrupts only while M_σ[i] is copied into a Secure-world RAM. Since this snapshot is taken atomically, it represents a consistent state of the block. Once the copy completes, interrupts can safely be re-enabled. This is because even if malware subsequently modifies or relocates the original Non-Secure contents, such changes cannot affect the snapshot already stored in Secure RAM. The measurement therefore operates on a consistent and protected representation of M_σ[i].  
**TH (Snap):** SMARM-Snap ปิด interrupt แค่ตอน copy M_σ[i] เข้า Secure RAM snapshot แบบ atomic จึงเป็นสถานะสม่ำเสมอของบล็อก พอ copy เสร็จเปิด interrupt ได้ปลอดภัย เพราะแม้ malware แก้/ย้ายต้นฉบับ Non-Secure ทีหลัง ก็ไม่กระทบสำเนาใน Secure แล้ว การวัดจึงทำบนภาพของ M_σ[i] ที่สม่ำเสมอและถูกป้องกัน

**EN (Guard):** SMARM-Guard achieves the same property without creating a snapshot. Before measuring M_σ[i], the SMARM service atomically reconfigures the SAU so that the block becomes inaccessible to Non-Secure software. Interrupts can then be re-enabled while the block is measured in place, since malware cannot access or modify the protected block during this period. After the measurement completes, the SMARM service atomically restores the original SAU configuration before proceeding to the next block.  
**TH (Guard):** SMARM-Guard ได้คุณสมบัติเดียวกันโดยไม่สร้าง snapshot ก่อนวัด M_σ[i] บริการปรับ SAU แบบ atomic ให้บล็อก Non-Secure เข้าไม่ได้ จากนั้นเปิด interrupt ได้ขณะวัดทับที่เดิม เพราะ malware แตะบล็อกที่ถูกกันไม่ได้ หลังวัดเสร็จ คืน SAU เดิมแบบ atomic ก่อนไปบล็อกถัดไป

**EN:** Thus, SMARM-Snap and SMARM-Guard preserve [S1] through two different mechanisms. As such, with the unchanged guarantees of [S2] and [S3], both enhancements retain the security guarantees of the SMARM baseline while substantially reducing its non-interruptibility period.  
**TH:** ดังนั้นทั้งสองแบบรักษา [S1] ด้วยกลไกคนละแบบ พร้อม [S2]/[S3] ที่ไม่เปลี่ยน จึงคงการันตีความปลอดภัยของ baseline ไว้ ขณะที่ย่อช่วงปิด interrupt ได้มาก

**EN (Table caption):** Comparison of attestation approaches for real-time IoT devices.  
**TH:** เปรียบเทียบแนวทาง attestation สำหรับ IoT แบบ realtime

| Approach | RT safe | Extra Secure RAM | t_disabled ขนาดคร่าว ๆ | RTOS |
|----------|---------|------------------|-------------------------|------|
| Traditional RA | × | 0 | ~10³ ms (วัดทั้งก้อน) | × |
| SMARM | Partial | 0 | ~10⁻¹–10⁰ ms (hash ต่อบล็อก) | × |
| SMARM-Snap | ✓ | B | ~10⁻²–10⁻¹ ms (copy ต่อบล็อก) | ✓ |
| SMARM-Guard | ✓ | 0 | ~10⁻³ ms (ปรับ SAU) | ✓ |

---

## 4. Evaluation

### 4.1 Setup

**EN:** We evaluate the SMARM baseline, SMARM-Snap, and SMARM-Guard on an STM32 Nucleo-144 development board equipped with a Cortex-M33 MCU and TrustZone-M.  
**TH:** เราประเมิน baseline SMARM, SMARM-Snap และ SMARM-Guard บน Nucleo-144 ที่เป็น Cortex-M33 + TrustZone-M

**EN:** The MCU runs at 110 MHz and executes FreeRTOS and real-time applications in the Non-Secure world.  
**TH:** MCU รันที่ 110 MHz และรัน FreeRTOS กับแอป realtime ใน Non-Secure

**EN:** We configure 512 KiB of Non-Secure flash as the attested memory region, yielding the total attested memory size of |M|=512 KiB.  
**TH:** เราตั้ง Non-Secure flash 512 KiB เป็นโซนที่ attested ทำให้ |M|=512 KiB

**EN:** We evaluate all three designs across seven block sizes, B ∈ {64,128,256,512,1024,2048,4096} bytes.  
**TH:** ทดสอบทั้งสามแบบกับขนาดบล็อกเจ็ดค่า B ∈ {64,128,256,512,1024,2048,4096} bytes

**EN:** The primary metrics are the non-interruptibility period (t_disabled), additional Secure-world RAM, and real-time availability.  
**TH:** เมตริกหลักคือช่วงปิด interrupt (t_disabled), Secure RAM ที่เพิ่ม และความพร้อม realtime

**EN:** As a secondary cost, we also report end-to-end attestation runtime for an equalized 512 KiB hashed workload.  
**TH:** เป็นต้นทุนรอง เรายังรายงานเวลา attestation ทั้งรอบสำหรับงาน hash ที่เทียบเท่า 512 KiB

### 4.2 Non-Interruptibility Period

**EN:** Fig. (tdisabled) compares the per-block t_disabled of all three designs on a logarithmic scale.  
**TH:** รูป t_disabled เทียบ t_disabled ต่อบล็อกของทั้งสามแบบบนสเกล log

**EN:** SMARM-Snap reduces t_disabled by about 15–16× for B≥128 bytes by limiting interrupt disablement to the block copy.  
**TH:** SMARM-Snap ย่อ t_disabled ประมาณ 15–16 เท่า เมื่อ B≥128 โดยปิด interrupt แค่ตอน copy

**EN:** SMARM-Guard keeps HMAC interruptible and disables interrupts only for the SAU lock and restore sequences, whose combined cost is 0.80–1.30 µs across the evaluated block sizes.  
**TH:** SMARM-Guard ให้ HMAC เปิด interrupt ได้ และปิด interrupt แค่ตอน SAU lock/restore รวมประมาณ 0.80–1.30 µs ในทุกขนาดบล็อกที่ทดสอบ

**EN (Fig. caption):** Per-block interrupt-disabled time versus block size (logarithmic y-axis). Baseline SMARM disables interrupts during hashing, SMARM-Snap only during the block copy, and SMARM-Guard only during the SAU lock and restore sequences.  
**TH:** เวลาปิด interrupt ต่อบล็อกเทียบขนาดบล็อก (แกน y เป็น log) — Baseline ปิดตอน hash; SMARM-Snap ปิดแค่ตอน copy; SMARM-Guard ปิดแค่ตอน SAU lock/restore

### 4.3 Secure Memory Overhead

**EN:** In terms of Secure-world RAM overhead, SMARM-Snap needs one Secure buffer of size B (up to 4 KB at B=4096).  
**TH:** ด้าน Secure RAM เพิ่ม SMARM-Snap ต้องมี Secure buffer ขนาด B (สูงสุด 4 KB ที่ B=4096)

**EN:** SMARM-Guard needs no extra measurement buffer and keeps k and σ in Secure RAM as in baseline SMARM.  
**TH:** SMARM-Guard ไม่ต้องมีบัฟเฟอร์วัดเพิ่ม และเก็บ k กับ σ ใน Secure เหมือน baseline

### 4.4 Real-Time Availability

**EN:** Under the 1000 Hz workload, baseline SMARM loses activations for large blocks, whereas SMARM-Snap and SMARM-Guard maintain FAR≈1.0 through B=4096 bytes.  
**TH:** ที่งาน 1000 Hz baseline SMARM พลาด activation เมื่อบล็อกใหญ่ ส่วน SMARM-Snap และ SMARM-Guard คง FAR≈1.0 ได้ถึง B=4096

**EN:** This result confirms that shortening the per-block IRQ-off window preserves the task's frequency-level availability.  
**TH:** ผลนี้ยืนยันว่าการย่อช่วงปิด interrupt ต่อบล็อกช่วยรักษาความพร้อมระดับความถี่ของงานไว้ได้

### 4.5 Attestation Runtime

**EN:** Reducing t_disabled does not proportionally shorten a full attestation round: the HMAC over |M|=512 KiB still dominates wall-clock time.  
**TH:** การย่อ t_disabled ไม่ได้ทำให้รอบ attestation ทั้งก้อนสั้นลงตามสัดส่วน เพราะ HMAC ของ |M|=512 KiB ยังครองเวลา wall-clock

**EN:** Table (attest_runtime) reports the mean duration of one Secure shuffled HMAC call with no concurrent real-time task (ten rounds per configuration).  
**TH:** ตาราง attest_runtime รายงานเวลาเฉลี่ยของการเรียก Secure shuffled HMAC หนึ่งครั้ง โดยไม่มีงาน realtime คู่แข่ง (10 rounds ต่อค่าคอนฟิก)

**EN:** Across these sizes the three designs remain in the same ballpark, from about 480 ms at B=4096 to about 630–650 ms at B=64.  
**TH:** ในทุกขนาดทั้งสามแบบอยู่ในย่านเดียวกัน จากประมาณ 480 ms ที่ B=4096 ถึงประมาณ 630–650 ms ที่ B=64

**EN:** Sample standard deviation over the ten rounds is below 1 ms for every cell (often exactly 0).  
**TH:** ส่วนเบี่ยงเบนมาตรฐานจากตัวอย่าง 10 rounds ต่ำกว่า 1 ms ทุกช่อง (มักเป็น 0 พอดี)

**EN:** SMARM-Snap is consistently about 30 ms slower than the baseline because it copies the hashed bytes before HMAC (IRQ on).  
**TH:** SMARM-Snap ช้ากว่า baseline ประมาณ 30 ms สม่ำเสมอ เพราะต้อง copy ไบต์ที่จะ hash ก่อน HMAC (ตอนเปิด interrupt)

**EN:** SMARM-Guard adds only a short SAU update per block (IRQ off), then hashes with interrupts enabled; the SAU work is microseconds per block and does not show up as a large end-to-end gap.  
**TH:** SMARM-Guard เพิ่มแค่การอัปเดต SAU สั้น ๆ ต่อบล็อก (ปิด interrupt) แล้ว hash ตอนเปิด interrupt; งาน SAU อยู่ระดับไมโครวินาทีต่อบล็อก จึงไม่โผล่เป็นช่องว่างใหญ่ในเวลาทั้งรอบ

**EN:** Baseline SMARM and SMARM-Guard therefore differ by only a few milliseconds; this paper does not claim that SMARM-Guard is faster.  
**TH:** ดังนั้น baseline กับ SMARM-Guard ต่างกันแค่ไม่กี่มิลลิวินาที; เปเปอร์นี้ไม่เคลมว่า SMARM-Guard เร็วกว่า

**EN:** The intended tradeoff is a shorter t_disabled, not a shorter attestation round.  
**TH:** tradeoff ที่ตั้งใจคือ t_disabled สั้นลง ไม่ใช่รอบ attestation สั้นลง

**EN (Table caption):** Mean end-to-end attestation time (ms) for a 512 KiB hashed workload with no concurrent real-time task (ten rounds; sample SD < 1 ms).  
**TH:** เวลา attestation ทั้งรอบเฉลี่ย (ms) สำหรับงาน hash 512 KiB โดยไม่มีงาน realtime คู่แข่ง (10 rounds; SD ของตัวอย่าง < 1 ms)

| B | Baseline | SMARM-Snap | SMARM-Guard |
|--:|---------:|-----------:|------------:|
| 64 | 644 | 654 | 635 |
| 128 | 567 | 592 | 559 |
| 256 | 521 | 550 | 517 |
| 512 | 500 | 529 | 497 |
| 1024 | 490 | 519 | 487 |
| 2048 | 484 | 513 | 482 |
| 4096 | 481 | 511 | 480 |

### 4.6 Summary and Design Guidance

**EN:** While a block is ephemerally unavailable (Secure-by-default), a concurrent benign Non-Secure access can fault or stall even though interrupts remain enabled for scheduling.  
**TH:** ตอนบล็อกใช้ไม่ได้ชั่วคราว (Secure-by-default) การเข้าถึงจาก Non-Secure แบบไม่ประสงค์ร้ายที่ชนช่วงเดียวกันอาจ fault หรือ stall ได้ แม้ interrupt สำหรับ scheduling ยังเปิดอยู่

**EN:** Our prototype leaves recovery to the designer (e.g., restore Non-Secure coverage and restart the shuffled session); we report t_disabled under contention-free measurement of the reserved data window.  
**TH:** โปรโตไทป์ของเราฝาก recovery ให้นักออกแบบ (เช่น คืน Non-Secure coverage แล้วเริ่ม session ใหม่); เรารายงาน t_disabled ภายใต้การวัดหน้าต่างข้อมูลที่จองไว้แบบไม่มี contention

**EN:** When to prefer each enhancement.  
**TH:** เมื่อไหร่ควรเลือกแบบไหน

**EN:** Platform guidance typically configures SAU once at boot as part of the static Secure/Non-Secure map, and production TF-M-style images often lock SAU/GTZC after that setup.  
**TH:** คู่มือแพลตฟอร์มมักตั้ง SAU ครั้งเดียวตอน boot เป็นแผนที่ Secure/Non-Secure แบบตายตัว และอิมเมจแบบ TF-M ใน production มักล็อก SAU/GTZC หลังตั้งเสร็จ

**EN:** Our per-block SAU updates are not a replacement for that static partition: they briefly reprogram only |M| during attestation.  
**TH:** การอัปเดต SAU ต่อบล็อกของเราไม่ได้แทนที่การแบ่งแบบ static นั้น: มันแค่โปรแกรมใหม่สั้น ๆ เฉพาะ |M| ระหว่าง attestation

**EN:** Prefer SMARM-Snap when Secure RAM of size B is available, SAU must remain locked after boot, free SAU regions are scarce, or the attested image cannot be isolated from Non-Secure code/vectors.  
**TH:** เลือก SMARM-Snap เมื่อมี Secure RAM ขนาด B, หรือ SAU ต้องล็อกหลัง boot, หรือ region SAU ว่างน้อย, หรือแยกภาพ attested ออกจากโค้ด/vector ของ Non-Secure ไม่ได้

**EN:** Prefer SMARM-Guard when Secure RAM is tight, a dedicated Non-Secure data window can be reserved, and Secure software may reprogram SAU briefly.  
**TH:** เลือก SMARM-Guard เมื่อ Secure RAM แน่น จองหน้าต่างข้อมูล Non-Secure ไว้ได้ และอนุญาตให้ซอฟต์แวร์ Secure ปรับ SAU สั้น ๆ ได้

**EN:** Our prototype leaves SAU unlocked so ephemeral unavailability can run; platforms that lock SAU after boot therefore cannot use SMARM-Guard without changing that policy, and DMA into |M| still needs controls such as GTZC.  
**TH:** โปรโตไทป์ของเราปลดล็อก SAU ไว้เพื่อให้ “ใช้ไม่ได้ชั่วคราว” ทำงานได้; แพลตฟอร์มที่ล็อก SAU หลัง boot จึงใช้ SMARM-Guard ไม่ได้ถ้าไม่เปลี่ยนนโยบายนั้น และ DMA เข้า |M| ยังต้องคุมแยก เช่น ด้วย GTZC

---

## 5. Related Work

**EN:** Classical and shuffled RA. SMART, VRASED, and HYDRA use long atomic measurements.  
**TH:** RA แบบคลาสสิกและแบบ shuffled — SMART, VRASED และ HYDRA ใช้การวัด atomic ยาว

**EN:** SMARM, PRoM, and HAtt use shuffled measurement to resist roving malware while allowing interrupts between blocks.  
**TH:** SMARM, PRoM และ HAtt ใช้ shuffled measurement เพื่อต้าน roving malware และเปิด interrupt ระหว่างบล็อกได้

**EN:** SMARM's original prototype targeted a seL4/HYDRA stack rather than commodity RTOSes.  
**TH:** โปรโตไทป์เดิมของ SMARM อยู่บนสแตก seL4/HYDRA ไม่ใช่ RTOS ทั่วไป

**EN:** A companion report describes a FreeRTOS/TrustZone-M instantiation that we use as the experimental baseline; here we evaluate two per-block capture mechanisms on that stack.  
**TH:** รายงาน companion อธิบายการทำบน FreeRTOS/TrustZone-M ที่เราใช้เป็น baseline การทดลอง; ที่นี่เราประเมินกลไกจับต่อบล็อกสองแบบบนสแตกนั้น

**EN:** Interruptible attestation. ISC-FLAT, ASAP, and IDA handle interrupts in control-flow or hybrid attestation. They are complementary; we target memory-content RA under FreeRTOS.  
**TH:** Attestation ที่เกี่ยวกับ interrupt — ISC-FLAT, ASAP และ IDA จัดการ interrupt ใน control-flow หรือ hybrid attestation เป็นงานเสริมกับเรา; เราโฟกัส RA เนื้อหาหน่วยความจำบน FreeRTOS

**EN:** RTOS timekeeping under Secure services. Joia Neto et al. characterize SysTick loss when Non-Secure RTOS timekeeping is blocked during atomic Secure-world services on TrustZone-M and propose Secure-world tick reconstruction.  
**TH:** การจับเวลาของ RTOS ตอนบริการ Secure — Joia Neto et al. วิเคราะห์การเสีย SysTick เมื่อ timekeeping ฝั่ง Non-Secure ถูกบล็อกระหว่างบริการ Secure แบบ atomic บน TrustZone-M และเสนอการสร้าง tick ใหม่ฝั่ง Secure

**EN:** That work is orthogonal to ours: we shorten the per-block interrupt-disabled window during shuffled measurement rather than compensating the RTOS clock after long Secure calls.  
**TH:** งานนั้นคนละมุมกับเรา: เราย่อช่วงปิด interrupt ต่อบล็อกระหว่าง shuffled measurement ไม่ได้ชดเชยนาฬิกา RTOS หลังเรียก Secure ยาว ๆ

**EN:** TrustZone-M and SAU. TrustZone-M and SAU define CPU security attribution. TF-M and vendor TFM examples usually set a static SAU map early and may lock it until reset.  
**TH:** TrustZone-M และ SAU กำหนด attribution ความปลอดภัยของ CPU — TF-M และตัวอย่างผู้ขายมักตั้งแผนที่ SAU แบบ static ตั้งแต่ต้น และอาจล็อกไว้จนรีเซ็ต

**EN:** We instead apply brief dynamic SAU reconfiguration during SMARM measurement as an alternative to copying a block into Secure RAM, and discuss when that choice is appropriate in Section (guidance).  
**TH:** เราใช้การปรับ SAU แบบสั้น ๆ ระหว่างวัดของ SMARM เป็นทางเลือกแทนการ copy บล็อกเข้า Secure RAM และอภิปรายว่าเมื่อไหร่เหมาะในหัวข้อ guidance

**EN:** Custom hardware. LO-FAT, LiteHAX, and ATRIUM remove non-interruptible windows with dedicated hardware. We stay on unmodified TrustZone-M MCUs.  
**TH:** ฮาร์ดแวร์พิเศษ — LO-FAT, LiteHAX และ ATRIUM ตัดช่วงปิด interrupt ด้วยฮาร์ดแวร์เฉพาะทาง; เราอยู่บน MCU TrustZone-M ที่ไม่ดัดแปลง

---

## 6. Conclusion

**EN:** SMARM improves RA interactivity by allowing interrupts between blocks, but its per-block interrupt-disabled hash still yields a long t_disabled for large B on FreeRTOS/TrustZone-M.  
**TH:** SMARM ทำให้ RA โต้ตอบได้ดีขึ้นด้วยการเปิด interrupt ระหว่างบล็อก แต่ hash ต่อบล็อกที่ปิด interrupt ยังทำให้ t_disabled ยาวเมื่อ B ใหญ่ บน FreeRTOS/TrustZone-M

**EN:** SMARM-Snap shortens that window to a Secure-world memory copy at the cost of a Secure buffer of size B and about 30 ms extra round time from the copy.  
**TH:** SMARM-Snap ย่อช่วงนั้นเหลือตอน copy ใน Secure แลกกับ Secure buffer ขนาด B และเวลาทั้งรอบช้าขึ้นประมาณ 30 ms จาก copy

**EN:** SMARM-Guard achieves a still shorter t_disabled without that buffer by making each block ephemerally unavailable via brief SAU updates, then hashing in place with interrupts enabled.  
**TH:** SMARM-Guard ได้ t_disabled สั้นกว่าอีกโดยไม่ต้องมีบัฟเฟอร์ ด้วยการทำให้แต่ละบล็อกใช้ไม่ได้ชั่วคราวผ่านการอัปเดต SAU สั้น ๆ แล้ว hash ทับที่เดิมตอนเปิด interrupt ได้

**EN:** Both preserve (S1)–(S3). Prefer SMARM-Snap when Secure RAM of size B is available or the SAU must remain fixed after boot, and SMARM-Guard when a dedicated Non-Secure data window |M| can absorb brief SAU attribution changes.  
**TH:** ทั้งคู่รักษา (S1)–(S3) เลือก SMARM-Snap เมื่อมี Secure RAM ขนาด B หรือ SAU ต้องคงที่หลัง boot และเลือก SMARM-Guard เมื่อมีหน้าต่างข้อมูล Non-Secure |M| ที่รับการเปลี่ยน attribution ของ SAU สั้น ๆ ได้

---

*จบไฟล์แปลบรรทัดต่อบรรทัด — คู่กับสรุปสั้นใน `main_thai_summary.md`*
