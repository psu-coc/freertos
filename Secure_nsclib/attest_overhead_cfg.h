/**
 * Snap/Guard paper harness (branch: snapguard-paper-reviewer).
 * Same firmware image; switch algorithm with the two flags below only.
 *
 *   Baseline SMARM:  USE_SAU_APPROACH 0, USE_SNAP 0
 *   SMARM-Snap:      USE_SAU_APPROACH 0, USE_SNAP 1
 *   SMARM-Guard:     USE_SAU_APPROACH 1, USE_SNAP 0
 *
 * Then rebuild Secure + NonSecure and flash both ELF images.
 *
 * Default window: |M|=128 KiB @ 0x08060000 (no live NS XIP).
 * ATTEST_FAULT_DEMO=1: Guard only — |M|=256 KiB @ 0x08040000 (full NS flash,
 * including live FreeRTOS XIP). Expect SecureFault; not a production path.
 */
#ifndef ATTEST_OVERHEAD_CFG_H
#define ATTEST_OVERHEAD_CFG_H

#define USE_SAU_APPROACH  0   /* normal Baseline (IRQ-off HMAC) */
#define USE_SNAP          0
#if USE_SAU_APPROACH && USE_SNAP
#error USE_SAU_APPROACH and USE_SNAP are mutually exclusive
#endif

#define ATTEST_BLOCK_SIZE  128

/* 1 = one-shot reviewer Q3 demo (Guard + full NS flash). Keep 0 for measurements. */
#define ATTEST_FAULT_DEMO  0
#if ATTEST_FAULT_DEMO && !USE_SAU_APPROACH
#error ATTEST_FAULT_DEMO requires USE_SAU_APPROACH 1 (Guard)
#endif

/*
 * EVALUATION-ONLY FAR reference (NOT a secure attestation design).
 * When 1 on the Baseline path: HMAC without __disable_irq/__enable_irq.
 * Keep 0 for all paper/security builds.
 */
#define FAR_REF_NO_IRQ_DISABLE  0
#if FAR_REF_NO_IRQ_DISABLE && (USE_SAU_APPROACH || USE_SNAP)
#error FAR_REF_NO_IRQ_DISABLE is Baseline-path only (USE_SAU=0, USE_SNAP=0)
#endif

/*
 * Per-round summary (avg/min/max/std) is enough for the paper table.
 * Raw L/U probe logs saved under thesis/01_snapguard/logs/ — keep OFF by default.
 */
#define TDIS_RAW_DUMP  0
#define TDIS_RAW_CAP   2048U

#endif /* ATTEST_OVERHEAD_CFG_H */
