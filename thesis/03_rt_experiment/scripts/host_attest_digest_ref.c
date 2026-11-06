/*
 * Host deterministic attestation digest reference (correctness, not RT).
 *
 * Fixed challenge + synthetic |M| (128 KiB patterned). Independently checks
 * Original / PRNG / FPE digests using firmware AES-CTR, HMAC-SHA256, and
 * (for FPE) fixed FF1-Speck. Performance smoke on-target is separate.
 *
 * Build via test_attest_digest_ref.py (ziglang cc).
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "aes.h"
#include "hmac-sha256.h"
#include "ff1_speck.h"

#define TOTAL_SIZE   0x20000u
#define BLOCK_SIZE  1024u
#define BLOCKS      ((int)(TOTAL_SIZE / BLOCK_SIZE))
#define ATTEST_PASSES 1u

static const uint8_t SECRET[] = "MySecureKey123"; /* master k — tip string */

/* Fixed challenge (16 B) — deterministic correctness vector. */
static const uint8_t CHALLENGE[16] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10
};

#define KDF_LABEL_PRP ((uint8_t)0x00)
#define KDF_LABEL_MAC ((uint8_t)0x01)

typedef struct {
    struct AES_ctx ctx;
    uint8_t buf[16];
    int idx;
} ctr_prng_t;

/* KDF(label, k) := HMAC-SHA256(k, label) — matches Secure/Src/secure_nsc.c */
static void kdf_derive_32(uint8_t out32[32], uint8_t label)
{
    hmac_sha256 hmac;
    uint8_t lab = label;
    hmac_sha256_initialize(&hmac, SECRET, (unsigned int)strlen((const char *)SECRET));
    hmac_sha256_update(&hmac, &lab, 1);
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out32, hmac.digest, 32);
}

static void derive_aes_key_iv(uint8_t key16[16], uint8_t iv16[16],
                              const uint8_t *k_prp32,
                              const uint8_t *challenge, size_t clen)
{
    hmac_sha256 hmac;
    hmac_sha256_initialize(&hmac, k_prp32, 32);
    hmac_sha256_update(&hmac, challenge, clen);
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(key16, hmac.digest, 16);
    memcpy(iv16, hmac.digest + 16, 16);
}

static void prng_init(ctr_prng_t *p, const uint8_t key16[16], const uint8_t iv16[16])
{
    AES_init_ctx_iv(&p->ctx, key16, iv16);
    memset(p->buf, 0, sizeof(p->buf));
    p->idx = 16;
}

static void prng_refill(ctr_prng_t *p)
{
    uint8_t zero[16] = {0};
    memcpy(p->buf, zero, 16);
    AES_CTR_xcrypt_buffer(&p->ctx, p->buf, 16);
    p->idx = 0;
}

static uint32_t prng_next_u32(ctr_prng_t *p)
{
    if (p->idx > 12) prng_refill(p);
    uint32_t v;
    memcpy(&v, &p->buf[p->idx], 4);
    p->idx += 4;
    return v;
}

static int prng_uniform(ctr_prng_t *p, int n)
{
    const uint32_t lim = 0xFFFFFFFFu - (0xFFFFFFFFu % (uint32_t)n);
    for (;;) {
        uint32_t r = prng_next_u32(p);
        if (r < lim) return (int)(r % (uint32_t)n);
    }
}

static void shuffle_fy(int *arr, int n, const uint8_t key16[16], const uint8_t iv16[16])
{
    ctr_prng_t prng;
    prng_init(&prng, key16, iv16);
    for (int i = n - 1; i > 0; i--) {
        int j = prng_uniform(&prng, i + 1);
        int tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
    }
}

/* PRNG bitarray helpers */
static uint8_t used_bits[(BLOCKS + 7) / 8];

static int get_bit(int i) { return (used_bits[i / 8] >> (i % 8)) & 1; }
static void set_bit(int i) { used_bits[i / 8] |= (uint8_t)(1u << (i % 8)); }
static void clear_bits(int n) { memset(used_bits, 0, (size_t)((n + 7) / 8)); }

static int find_kth_zero(int n, int k)
{
    int count = 0, result = 0;
    for (int i = 0; i < n; i++) {
        if (get_bit(i) == 0) {
            if (count == k) result = i;
            count++;
        }
    }
    return result;
}

static void fill_memory(uint8_t *mem)
{
    /* Known patterned contents — independent of on-flash image.
     * Must differ across 1024 B blocks (avoid period-1024 patterns). */
    for (uint32_t i = 0; i < TOTAL_SIZE; i++) {
        uint32_t blk = i / BLOCK_SIZE;
        mem[i] = (uint8_t)((blk * 17u) ^ (i * 131u + 17u));
    }
}

static void digest_hex(const uint8_t d[32], char out[65])
{
    static const char *hex = "0123456789abcdef";
    for (int i = 0; i < 32; i++) {
        out[2 * i] = hex[d[i] >> 4];
        out[2 * i + 1] = hex[d[i] & 0xf];
    }
    out[64] = 0;
}

enum { MODE_ORIGINAL = 0, MODE_PRNG = 1, MODE_FPE = 2 };

static void compute_digest(int mode, const uint8_t *mem, uint8_t out[32],
                           const uint8_t challenge[16])
{
    uint8_t k_prp[32], k_mac[32];
    uint8_t key16[16], iv16[16];
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    kdf_derive_32(k_mac, KDF_LABEL_MAC);
    derive_aes_key_iv(key16, iv16, k_prp, challenge, 16);

    hmac_sha256 hmac;
    hmac_sha256_initialize(&hmac, k_mac, 32);

    for (uint32_t pass = 0; pass < ATTEST_PASSES; pass++) {
        int indices[BLOCKS];
        ctr_prng_t prng;
        FF1_Key_Speck ff1_key;
        uint32_t ff1_tweak = 0;

        if (mode == MODE_ORIGINAL) {
            for (int i = 0; i < BLOCKS; i++) indices[i] = i;
            uint8_t iv_pass[16];
            memcpy(iv_pass, iv16, 16);
            iv_pass[0] ^= (uint8_t)pass;
            shuffle_fy(indices, BLOCKS, key16, iv_pass);
        } else if (mode == MODE_PRNG) {
            uint8_t iv_pass[16];
            memcpy(iv_pass, iv16, 16);
            iv_pass[0] ^= (uint8_t)pass;
            prng_init(&prng, key16, iv_pass);
            clear_bits(BLOCKS);
        } else {
            ff1_tweak = ((uint32_t)challenge[0] << 24) |
                        ((uint32_t)challenge[1] << 16) |
                        ((uint32_t)challenge[2] << 8) |
                        (uint32_t)challenge[3];
            ff1_tweak ^= (uint32_t)pass;
            FF1_SetKey_Speck(&ff1_key, key16);
        }

        for (int i = 0; i < BLOCKS; i++) {
            int idx;
            if (mode == MODE_ORIGINAL) {
                idx = indices[i];
            } else if (mode == MODE_PRNG) {
                int remaining = BLOCKS - i;
                int k = prng_uniform(&prng, remaining);
                idx = find_kth_zero(BLOCKS, k);
                set_bit(idx);
            } else {
                uint32_t p = FF1Permute_Speck((uint32_t)i, (uint32_t)BLOCKS,
                                              &ff1_key, ff1_tweak);
                if (p >= (uint32_t)BLOCKS) {
                    fprintf(stderr, "FPE idx OOB\n");
                    exit(2);
                }
                idx = (int)p;
            }
            hmac_sha256_update(&hmac, &mem[(size_t)idx * BLOCK_SIZE], BLOCK_SIZE);
        }
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out, hmac.digest, 32);
}

/* Collect visit counts for one Original shuffle (bijection check). */
static int check_original_covers_all(const uint8_t challenge[16])
{
    uint8_t k_prp[32], key16[16], iv16[16];
    int indices[BLOCKS];
    int seen[BLOCKS];
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    derive_aes_key_iv(key16, iv16, k_prp, challenge, 16);
    for (int i = 0; i < BLOCKS; i++) {
        indices[i] = i;
        seen[i] = 0;
    }
    shuffle_fy(indices, BLOCKS, key16, iv16);
    for (int i = 0; i < BLOCKS; i++) {
        if (indices[i] < 0 || indices[i] >= BLOCKS) return 0;
        seen[indices[i]]++;
    }
    for (int i = 0; i < BLOCKS; i++) {
        if (seen[i] != 1) return 0;
    }
    return 1;
}

static int check_prng_covers_all(const uint8_t challenge[16])
{
    uint8_t k_prp[32], key16[16], iv16[16];
    ctr_prng_t prng;
    int seen[BLOCKS];
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    derive_aes_key_iv(key16, iv16, k_prp, challenge, 16);
    prng_init(&prng, key16, iv16);
    clear_bits(BLOCKS);
    memset(seen, 0, sizeof(seen));
    for (int i = 0; i < BLOCKS; i++) {
        int remaining = BLOCKS - i;
        int k = prng_uniform(&prng, remaining);
        int idx = find_kth_zero(BLOCKS, k);
        set_bit(idx);
        if (idx < 0 || idx >= BLOCKS) return 0;
        seen[idx]++;
    }
    for (int i = 0; i < BLOCKS; i++) {
        if (seen[i] != 1) return 0;
    }
    return 1;
}

static int check_fpe_covers_all(const uint8_t challenge[16])
{
    uint8_t k_prp[32], key16[16], iv16[16];
    FF1_Key_Speck ff1_key;
    int seen[BLOCKS];
    uint32_t tweak;
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    derive_aes_key_iv(key16, iv16, k_prp, challenge, 16);
    (void)iv16;
    tweak = ((uint32_t)challenge[0] << 24) | ((uint32_t)challenge[1] << 16) |
            ((uint32_t)challenge[2] << 8) | (uint32_t)challenge[3];
    FF1_SetKey_Speck(&ff1_key, key16);
    memset(seen, 0, sizeof(seen));
    for (int i = 0; i < BLOCKS; i++) {
        uint32_t p = FF1Permute_Speck((uint32_t)i, (uint32_t)BLOCKS, &ff1_key, tweak);
        if (p >= (uint32_t)BLOCKS) return 0;
        seen[p]++;
    }
    for (int i = 0; i < BLOCKS; i++) {
        if (seen[i] != 1) return 0;
    }
    return 1;
}

int main(void)
{
    uint8_t *mem = (uint8_t *)malloc(TOTAL_SIZE);
    if (!mem) return 1;
    fill_memory(mem);

    uint8_t k_prp[32], k_mac[32];
    char hex_prp[65], hex_mac[65];
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    kdf_derive_32(k_mac, KDF_LABEL_MAC);
    digest_hex(k_prp, hex_prp);
    digest_hex(k_mac, hex_mac);
    printf("k_prp %s\n", hex_prp);
    printf("k_mac %s\n", hex_mac);
    if (memcmp(k_prp, k_mac, 32) == 0) {
        fprintf(stderr, "FAIL: k_prp == k_mac\n");
        free(mem);
        return 1;
    }
    printf("PASS k_prp_ne_k_mac\n");

    /* Deterministic KDF */
    uint8_t k_prp2[32];
    kdf_derive_32(k_prp2, KDF_LABEL_PRP);
    if (memcmp(k_prp, k_prp2, 32) != 0) {
        fprintf(stderr, "FAIL: KDF not deterministic\n");
        free(mem);
        return 1;
    }
    printf("PASS kdf_deterministic\n");

    const char *names[] = {"original", "prng", "fpe"};
    uint8_t digests[3][32];
    char hex[3][65];

    for (int m = 0; m < 3; m++) {
        compute_digest(m, mem, digests[m], CHALLENGE);
        digest_hex(digests[m], hex[m]);
        printf("%s %s\n", names[m], hex[m]);
    }

    if (memcmp(digests[0], digests[1], 32) == 0 ||
        memcmp(digests[0], digests[2], 32) == 0 ||
        memcmp(digests[1], digests[2], 32) == 0) {
        fprintf(stderr, "FAIL: digests not all distinct\n");
        free(mem);
        return 1;
    }
    printf("PASS digests_distinct\n");

    uint8_t again[32];
    compute_digest(MODE_ORIGINAL, mem, again, CHALLENGE);
    if (memcmp(again, digests[0], 32) != 0) {
        fprintf(stderr, "FAIL: Original not deterministic\n");
        free(mem);
        return 1;
    }
    printf("PASS digests_deterministic\n");

    /* Different challenge -> different Original digest (order changes). */
    uint8_t chal2[16];
    memcpy(chal2, CHALLENGE, 16);
    chal2[15] ^= 0x01;
    uint8_t dig_alt[32];
    compute_digest(MODE_ORIGINAL, mem, dig_alt, chal2);
    if (memcmp(dig_alt, digests[0], 32) == 0) {
        fprintf(stderr, "FAIL: challenge did not change Original digest\n");
        free(mem);
        return 1;
    }
    printf("PASS challenge_changes_order_digest\n");

    if (!check_original_covers_all(CHALLENGE) ||
        !check_prng_covers_all(CHALLENGE) ||
        !check_fpe_covers_all(CHALLENGE)) {
        fprintf(stderr, "FAIL: permutation does not cover all blocks once\n");
        free(mem);
        return 1;
    }
    printf("PASS all_blocks_visited_once\n");

    printf("PASS all_kdf_host_vectors\n");
    free(mem);
    return 0;
}
