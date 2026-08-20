/**
 * Same firmware, three algorithms. Do not switch git branches to measure Guard.
 *
 * Baseline (current): USE_SAU_APPROACH 0, USE_SNAP 0
 * Snap:               USE_SAU_APPROACH 0, USE_SNAP 1
 * Guard/SAU:          USE_SAU_APPROACH 1, USE_SNAP 0
 *
 * Change only this file, then rebuild Secure + NonSecure and flash both.
 * Expected wall-clock: Baseline fastest; Snap more expensive; Guard >= Baseline.
 */
#ifndef ATTEST_OVERHEAD_CFG_H
#define ATTEST_OVERHEAD_CFG_H

#define USE_SAU_APPROACH  1   /* 1 = Guard: SAU lock / HMAC IRQ on / unlock */
#define USE_SNAP          0   /* 1 = Snap; requires USE_SAU_APPROACH 0 */
#if USE_SAU_APPROACH && USE_SNAP
#error USE_SAU_APPROACH and USE_SNAP are mutually exclusive
#endif

#define ATTEST_BLOCK_SIZE  512  /* 64,128,256,512,1024,2048,4096 */

#endif /* ATTEST_OVERHEAD_CFG_H */
