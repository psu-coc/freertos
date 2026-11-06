/*
 * Host digest over a raw |M| dump (correctness). Modes: original|prng|fpe.
 * Usage:
 *   host_digest_from_mem.exe <mode> <mem.bin> [challenge_hex32] [block_size] [total_size]
 * Defaults: challenge=0123...3210, block_size=1024, total_size=0x40000 (256 KiB NS)
 * Also prints VISIT_ONCE=1|0 and n_blocks after digest line on stderr summary:
 *   digest hex on stdout; "VISIT_ONCE=1 n=<n>" on stderr.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "aes.h"
#include "hmac-sha256.h"
#include "ff1_speck.h"

static const uint8_t SECRET[] = "MySecureKey123";
static uint8_t CHALLENGE[16] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10
};

#define KDF_LABEL_PRP ((uint8_t)0x00)
#define KDF_LABEL_MAC ((uint8_t)0x01)

typedef struct { struct AES_ctx ctx; uint8_t buf[16]; int idx; } ctr_prng_t;

static void kdf_derive_32(uint8_t out32[32], uint8_t label)
{
    hmac_sha256 hmac;
    uint8_t lab = label;
    hmac_sha256_initialize(&hmac, SECRET, (unsigned int)strlen((const char *)SECRET));
    hmac_sha256_update(&hmac, &lab, 1);
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out32, hmac.digest, 32);
}

static void derive_aes_key_iv(uint8_t key16[16], uint8_t iv16[16], const uint8_t *k_prp32)
{
    hmac_sha256 hmac;
    hmac_sha256_initialize(&hmac, k_prp32, 32);
    hmac_sha256_update(&hmac, CHALLENGE, 16);
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(key16, hmac.digest, 16);
    memcpy(iv16, hmac.digest + 16, 16);
}

static void prng_init(ctr_prng_t *p, const uint8_t *k, const uint8_t *iv)
{
    AES_init_ctx_iv(&p->ctx, k, iv);
    memset(p->buf, 0, 16);
    p->idx = 16;
}
static void prng_refill(ctr_prng_t *p)
{
    uint8_t z[16] = {0};
    memcpy(p->buf, z, 16);
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
static void shuffle_fy(int *arr, int n, const uint8_t *k, const uint8_t *iv)
{
    ctr_prng_t prng;
    prng_init(&prng, k, iv);
    for (int i = n - 1; i > 0; i--) {
        int j = prng_uniform(&prng, i + 1);
        int tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
    }
}

static uint8_t *g_bits;
static int g_bits_bytes;

static void clear_bits(int n)
{
    g_bits_bytes = (n + 7) / 8;
    memset(g_bits, 0, (size_t)g_bits_bytes);
}
static int get_bit(int i) { return (g_bits[i / 8] >> (i % 8)) & 1; }
static void set_bit(int i) { g_bits[i / 8] |= (uint8_t)(1U << (i % 8)); }
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

enum { MODE_ORIGINAL = 0, MODE_PRNG = 1, MODE_FPE = 2 };

static int parse_mode(const char *s)
{
    if (!strcmp(s, "original")) return MODE_ORIGINAL;
    if (!strcmp(s, "prng")) return MODE_PRNG;
    if (!strcmp(s, "fpe")) return MODE_FPE;
    return -1;
}

static int parse_hex32(const char *s, uint8_t out[16])
{
    if (strlen(s) != 32) return -1;
    for (int i = 0; i < 16; i++) {
        unsigned v;
        if (sscanf(s + 2 * i, "%2x", &v) != 1) return -1;
        out[i] = (uint8_t)v;
    }
    return 0;
}

static int compute(int mode, const uint8_t *mem, uint32_t block_size, int blocks,
                   uint8_t out[32])
{
    uint8_t k_prp[32], k_mac[32];
    uint8_t key16[16], iv16[16];
    kdf_derive_32(k_prp, KDF_LABEL_PRP);
    kdf_derive_32(k_mac, KDF_LABEL_MAC);
    derive_aes_key_iv(key16, iv16, k_prp);
    hmac_sha256 hmac;
    hmac_sha256_initialize(&hmac, k_mac, 32);

    int *indices = (int *)malloc((size_t)blocks * sizeof(int));
    uint8_t *seen = (uint8_t *)calloc((size_t)blocks, 1);
    g_bits = (uint8_t *)malloc((size_t)((blocks + 7) / 8));
    if (!indices || !seen || !g_bits) {
        free(indices); free(seen); free(g_bits);
        return -1;
    }

    ctr_prng_t prng;
    FF1_Key_Speck ff1_key;
    uint32_t ff1_tweak = 0;

    if (mode == MODE_ORIGINAL) {
        for (int i = 0; i < blocks; i++) indices[i] = i;
        shuffle_fy(indices, blocks, key16, iv16);
    } else if (mode == MODE_PRNG) {
        prng_init(&prng, key16, iv16);
        clear_bits(blocks);
    } else {
        ff1_tweak = ((uint32_t)CHALLENGE[0] << 24) | ((uint32_t)CHALLENGE[1] << 16) |
                    ((uint32_t)CHALLENGE[2] << 8) | (uint32_t)CHALLENGE[3];
        FF1_SetKey_Speck(&ff1_key, key16);
    }

    int visit_ok = 1;
    for (int i = 0; i < blocks; i++) {
        int idx;
        if (mode == MODE_ORIGINAL) {
            idx = indices[i];
        } else if (mode == MODE_PRNG) {
            int k = prng_uniform(&prng, blocks - i);
            idx = find_kth_zero(blocks, k);
            set_bit(idx);
        } else {
            uint32_t p = FF1Permute_Speck((uint32_t)i, (uint32_t)blocks, &ff1_key, ff1_tweak);
            if (p >= (uint32_t)blocks) {
                fprintf(stderr, "FPE OOB\n");
                free(indices); free(seen); free(g_bits);
                return -2;
            }
            idx = (int)p;
        }
        if (idx < 0 || idx >= blocks || seen[idx]) visit_ok = 0;
        else seen[idx] = 1;
        hmac_sha256_update(&hmac, &mem[(size_t)idx * block_size], (int)block_size);
    }
    for (int i = 0; i < blocks; i++) {
        if (!seen[i]) visit_ok = 0;
    }
    hmac_sha256_finalize(&hmac, NULL, 0);
    memcpy(out, hmac.digest, 32);
    fprintf(stderr, "VISIT_ONCE=%d n=%d\n", visit_ok, blocks);
    free(indices);
    free(seen);
    free(g_bits);
    return visit_ok ? 0 : 3;
}

int main(int argc, char **argv)
{
    uint32_t block_size = 1024u;
    uint32_t total_size = 0x40000u; /* default: 256 KiB NS */

    if (argc < 3) {
        fprintf(stderr,
                "usage: %s original|prng|fpe mem.bin [challenge_hex32] [block_size] [total_size]\n",
                argv[0]);
        return 1;
    }
    int mode = parse_mode(argv[1]);
    if (mode < 0) {
        fprintf(stderr, "bad mode\n");
        return 1;
    }
    if (argc >= 4 && parse_hex32(argv[3], CHALLENGE) != 0) {
        fprintf(stderr, "bad challenge hex\n");
        return 1;
    }
    if (argc >= 5) block_size = (uint32_t)strtoul(argv[4], NULL, 0);
    if (argc >= 6) total_size = (uint32_t)strtoul(argv[5], NULL, 0);
    if (block_size == 0 || (total_size % block_size) != 0) {
        fprintf(stderr, "bad block_size/total_size\n");
        return 1;
    }
    int blocks = (int)(total_size / block_size);

    FILE *f = fopen(argv[2], "rb");
    if (!f) {
        perror(argv[2]);
        return 1;
    }
    uint8_t *mem = (uint8_t *)malloc(total_size);
    if (!mem) return 1;
    size_t n = fread(mem, 1, total_size, f);
    fclose(f);
    if (n != total_size) {
        fprintf(stderr, "need %u bytes, got %zu\n", total_size, n);
        free(mem);
        return 1;
    }

    uint8_t dig[32];
    int rc = compute(mode, mem, block_size, blocks, dig);
    for (int i = 0; i < 32; i++) printf("%02x", dig[i]);
    printf("\n");
    free(mem);
    return (rc == 0) ? 0 : rc;
}
