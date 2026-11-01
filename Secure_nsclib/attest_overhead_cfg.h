/**
 * IEEE Access SMARM+ / shared Secure_nsclib flags.
 *
 * Shuffle modes (ATTEST_SHUFFLE_MODE):
 *   0 ORIGINAL  — AES-CTR Fisher–Yates (measured Original SMARM path)
 *   1 PRNG      — used-bits / k-th zero (SMARM+PRNG / bitarray)
 *   2 FPE       — FF1-Speck permute (SMARM+FPE; bijection-fixed on smoke)
 *
 * Snap / Guard (orthogonal; keep 0 for PRNG/FPE RT campaign):
 *   Original SMARM:  USE_SAU_APPROACH 0, USE_SNAP 0  (HMAC with IRQ masked)
 *   SMARM-Snap:      USE_SAU_APPROACH 0, USE_SNAP 1
 *   SMARM-Guard:     USE_SAU_APPROACH 1, USE_SNAP 0
 *
 * Primary RT compare: IRQ masked around each block HMAC update in all three
 * shuffle modes (Policy A). Historical ff1 tip had mask commented out — see
 * thesis/03_rt_experiment/PRNG_FPE_SOURCE_DEPARTURES.md.
 *
 * Rebuild Secure + NonSecure and flash both after any change.
 * Default window: |M|=128 KiB @ 0x08060000.
 */
#ifndef ATTEST_OVERHEAD_CFG_H
#define ATTEST_OVERHEAD_CFG_H

#define ATTEST_SHUFFLE_ORIGINAL  0
#define ATTEST_SHUFFLE_PRNG      1
#define ATTEST_SHUFFLE_FPE       2

#ifndef ATTEST_SHUFFLE_MODE
#define ATTEST_SHUFFLE_MODE  0
#endif

#define USE_SAU_APPROACH  0   /* Phase3: keep 0 for Original/PRNG/FPE RT */
#define USE_SNAP          0
#if USE_SAU_APPROACH && USE_SNAP
#error USE_SAU_APPROACH and USE_SNAP are mutually exclusive
#endif

#if (ATTEST_SHUFFLE_MODE != ATTEST_SHUFFLE_ORIGINAL) && \
    (USE_SAU_APPROACH || USE_SNAP)
#error PRNG/FPE RT campaign requires USE_SAU_APPROACH=0 and USE_SNAP=0
#endif

#if (ATTEST_SHUFFLE_MODE < ATTEST_SHUFFLE_ORIGINAL) || \
    (ATTEST_SHUFFLE_MODE > ATTEST_SHUFFLE_FPE)
#error ATTEST_SHUFFLE_MODE must be ORIGINAL (0), PRNG (1), or FPE (2)
#endif

/* 1 = mask IRQ around block HMAC (Policy A — required for fair RT compare). */
#define ATTEST_IRQ_HMAC_MASK  1
#if !ATTEST_IRQ_HMAC_MASK
#error Stage-1 primary compare requires ATTEST_IRQ_HMAC_MASK 1
#endif

/*
 * EVALUATION-ONLY FAR reference (NOT a secure attestation design).
 * When 1 on Baseline path (USE_SAU=0, USE_SNAP=0): HMAC without IRQ mask.
 * Keep 0 for all paper/security and FAR-interference pilots.
 */
#ifndef FAR_REF_NO_IRQ_DISABLE
#define FAR_REF_NO_IRQ_DISABLE  0
#endif
#if FAR_REF_NO_IRQ_DISABLE && (USE_SAU_APPROACH || USE_SNAP)
#error FAR_REF_NO_IRQ_DISABLE is Baseline-path only (USE_SAU=0, USE_SNAP=0)
#endif
#if FAR_REF_NO_IRQ_DISABLE && (ATTEST_SHUFFLE_MODE != ATTEST_SHUFFLE_ORIGINAL)
#error FAR_REF_NO_IRQ_DISABLE supported for ORIGINAL shuffle only in this gate
#endif

#define ATTEST_BLOCK_SIZE  1024  /* Stage-1 default; matrix: 64..4096 */

/*
 * Access Table I / historical 512 KiB energy profile (default 0).
 * When 1: |M|=512 KiB @ 0x08000000..0x0807FFFF, ATTEST_PASSES=1.
 * Baseline HMAC only — never combine with Guard/Snap (would lock live NS XIP).
 * Frozen pre-KDF / KDF-512 evidence remains under Energy512_* tags; new primary
 * campaigns use ATTEST_NS_256K_REGION instead.
 */
#ifndef ATTEST_TABLE_I_REGION
#define ATTEST_TABLE_I_REGION  0
#endif
#if ATTEST_TABLE_I_REGION && (USE_SAU_APPROACH || USE_SNAP)
#error ATTEST_TABLE_I_REGION requires USE_SAU_APPROACH=0 and USE_SNAP=0
#endif

/*
 * Primary post-reviewer Non-Secure attestation window (default 0).
 * When 1: |M|=256 KiB @ 0x08040000..0x0807FFFF (entire NS flash bank).
 * Baseline HMAC only — never combine with Guard/Snap (live NS XIP).
 */
#ifndef ATTEST_NS_256K_REGION
#define ATTEST_NS_256K_REGION  0
#endif
#if ATTEST_NS_256K_REGION && (USE_SAU_APPROACH || USE_SNAP)
#error ATTEST_NS_256K_REGION requires USE_SAU_APPROACH=0 and USE_SNAP=0
#endif
#if ATTEST_NS_256K_REGION && ATTEST_TABLE_I_REGION
#error ATTEST_NS_256K_REGION and ATTEST_TABLE_I_REGION are mutually exclusive
#endif

/* 1 = one-shot reviewer Q3 demo (Guard + full NS flash). Keep 0 for measurements. */
#define ATTEST_FAULT_DEMO  0
#if ATTEST_FAULT_DEMO && !USE_SAU_APPROACH
#error ATTEST_FAULT_DEMO requires USE_SAU_APPROACH 1 (Guard)
#endif
#if ATTEST_FAULT_DEMO && ATTEST_TABLE_I_REGION
#error ATTEST_FAULT_DEMO and ATTEST_TABLE_I_REGION are mutually exclusive
#endif
#if ATTEST_FAULT_DEMO && ATTEST_NS_256K_REGION
#error ATTEST_FAULT_DEMO and ATTEST_NS_256K_REGION are mutually exclusive
#endif

#if ATTEST_SHUFFLE_MODE == ATTEST_SHUFFLE_ORIGINAL
#define ATTEST_SHUFFLE_MODE_NAME  "original"
#elif ATTEST_SHUFFLE_MODE == ATTEST_SHUFFLE_PRNG
#define ATTEST_SHUFFLE_MODE_NAME  "prng"
#else
#define ATTEST_SHUFFLE_MODE_NAME  "fpe"
#endif

#endif /* ATTEST_OVERHEAD_CFG_H */
