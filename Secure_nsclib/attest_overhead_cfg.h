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

#define USE_SAU_APPROACH  1   /* 1 = Guard: SAU lock / HMAC IRQ on / unlock */
#define USE_SNAP          0   /* 1 = Snap; requires USE_SAU_APPROACH 0 */
#if USE_SAU_APPROACH && USE_SNAP
#error USE_SAU_APPROACH and USE_SNAP are mutually exclusive
#endif

#define ATTEST_BLOCK_SIZE  1024  /* 64,128,256,512,1024,2048,4096 */

/* 1 = one-shot reviewer Q3 demo (Guard + full NS flash). Keep 0 for measurements. */
#define ATTEST_FAULT_DEMO  0
#if ATTEST_FAULT_DEMO && !USE_SAU_APPROACH
#error ATTEST_FAULT_DEMO requires USE_SAU_APPROACH 1 (Guard)
#endif

#endif /* ATTEST_OVERHEAD_CFG_H */
