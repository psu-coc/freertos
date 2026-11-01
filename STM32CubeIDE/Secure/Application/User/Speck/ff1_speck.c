/*
 * ff1_speck.c — FF1-style radix-2 Feistel over Speck128/128 (SMARM+FPE).
 *
 * Provenance: ff1 @ 7377001 STM32CubeIDE/Secure/Application/User/Speck/ff1_speck.c
 *
 * Departures from historical tip (documented in
 * thesis/03_rt_experiment/PRNG_FPE_SOURCE_DEPARTURES.md):
 *  1) Feistel uses separate integer halves A/B (fixes odd-n bitbuffer bug).
 *  2) Round width m = u on even rounds, v on odd (SP 800-38G); tip had this swapped.
 *  3) Cycle-walk re-encrypts under the *same* tweak (tip used tweak+1).
 *
 * PRF (Speck CBC-MAC-style over P||Q) is unchanged in cost structure (10 rounds).
 */
#include "ff1_speck.h"
#include <string.h>

void FF1_SetKey_Speck(FF1_Key_Speck *k, const uint8_t raw_key[16])
{
    Speck_Init(&k->cipher, cfg_128_128, MODE_ECB, (void *)raw_key, NULL, NULL);
}

static void Speck128_Encrypt_Block(const uint8_t in[16],
                                   uint8_t out[16],
                                   const FF1_Key_Speck *k)
{
    uint8_t buf[16];
    memcpy(buf, in, 16);
    Speck_Encrypt_128(
        k->cipher.round_limit,
        k->cipher.key_schedule,
        buf,
        buf
    );
    memcpy(out, buf, 16);
}

static void FF1_PRF_Speck(const FF1_Key_Speck *key,
                          const uint8_t *M, size_t Mlen,
                          uint8_t Y[16])
{
    uint8_t X[16] = {0};
    size_t blocks = (Mlen + 15) / 16;

    for (size_t j = 0; j < blocks; j++) {
        uint8_t tmp[16] = {0};
        const uint8_t *Mj = M + 16 * j;
        size_t remain = (Mlen - 16 * j);
        size_t len = remain < 16 ? remain : 16;

        for (size_t i = 0; i < len; i++) {
            tmp[i] = X[i] ^ Mj[i];
        }
        Speck128_Encrypt_Block(tmp, X, key);
    }
    memcpy(Y, X, 16);
}

static uint32_t ceillog2_Speck(uint32_t x)
{
    uint32_t n = 0, v = x - 1;
    while (v > 0) {
        v >>= 1;
        n++;
    }
    return n;
}

/* One Feistel encryption of an n-bit value (n <= 32). */
static uint32_t FF1EncryptBits_Speck(const FF1_Key_Speck *K,
                                     uint32_t x,
                                     uint32_t n,
                                     const uint8_t *T,
                                     size_t t)
{
    const uint32_t u = n / 2U;
    const uint32_t v = n - u;
    uint32_t A = (v < 32U) ? (x >> v) : 0U;
    uint32_t B = (v < 32U) ? (x & ((1U << v) - 1U)) : x;

    uint8_t P[16] = {0};
    P[0]  = 0x01;
    P[1]  = 0x00;
    P[2]  = 0x00;
    P[3]  = 0x02;
    P[4]  = 0x0A;
    P[5]  = (uint8_t)((n >> 24) & 0xFF);
    P[6]  = (uint8_t)((n >> 16) & 0xFF);
    P[7]  = (uint8_t)((n >> 8)  & 0xFF);
    P[8]  = (uint8_t)(n & 0xFF);
    P[9]  = 0x00;
    P[10] = 0x00;
    P[11] = (uint8_t)t;
    P[12] = P[13] = P[14] = P[15] = 0x00;

    uint32_t d = 4U * ((v + 15U) / 16U);
    if (d < 4U) {
        d = 4U;
    }

    size_t b = ((t + 15U) / 16U) * 16U;
    if (b == 0U) {
        b = 16U;
    }

    uint8_t Q[64];
    uint8_t PQ[16 + 64];

    for (int i = 0; i < 10; i++) {
        /* SP 800-38G: m = u if i even; m = v if i odd */
        const uint32_t m = ((i % 2) == 0) ? u : v;
        const uint32_t numB = B;

        memset(Q, 0, sizeof(Q));
        if (t > 0U) {
            memcpy(Q, T, t);
        }
        Q[b - 1U] = (uint8_t)i;

        {
            uint32_t tmp = numB;
            for (uint32_t k = 0; k < d; k++) {
                Q[b + d - 1U - k] = (uint8_t)(tmp & 0xFFU);
                tmp >>= 8;
            }
        }

        {
            size_t Qlen = b + d;
            size_t PQlen = 16U + Qlen;
            memcpy(PQ, P, 16);
            memcpy(PQ + 16, Q, Qlen);
            if (PQlen < 16U) {
                memset(PQ + PQlen, 0, 16U - PQlen);
                PQlen = 16U;
            }

            uint8_t Y[16];
            FF1_PRF_Speck(K, PQ, PQlen, Y);

            uint32_t y = 0;
            for (uint32_t k = 0; k < d; k++) {
                y = (y << 8) | Y[k];
            }

            const uint32_t mod =
                (m >= 32U) ? 0xFFFFFFFFu : ((1u << m) - 1u);
            const uint32_t c = (A + y) & mod;
            A = B;
            B = c;
        }
    }

    if (v >= 32U) {
        return B;
    }
    return (A << v) | B;
}

uint32_t FF1Permute_Speck(uint32_t index,
                          uint32_t maxblocks,
                          const FF1_Key_Speck *key,
                          uint32_t tweak)
{
    if (index >= maxblocks) {
        return index;
    }

    uint32_t n = ceillog2_Speck(maxblocks);
    if (n == 0U) {
        return index;
    }
    if (n > 32U) {
        n = 32U;
    }

    uint8_t T[4];
    T[0] = (uint8_t)((tweak >> 24) & 0xFF);
    T[1] = (uint8_t)((tweak >> 16) & 0xFF);
    T[2] = (uint8_t)((tweak >> 8) & 0xFF);
    T[3] = (uint8_t)(tweak & 0xFF);

    uint32_t out = FF1EncryptBits_Speck(key, index, n, T, sizeof(T));
    /* Same-tweak cycle-walk (historical tip incorrectly used tweak+1). */
    while (out >= maxblocks) {
        out = FF1EncryptBits_Speck(key, out, n, T, sizeof(T));
    }
    return out;
}

