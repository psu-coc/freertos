// ff1_speck.c
#include "ff1_speck.h"
#include <string.h>

// 1) Set key

void FF1_SetKey_Speck(FF1_Key_Speck *k, const uint8_t raw_key[16])
{
    Speck_Init(&k->cipher, cfg_128_128, MODE_ECB, (void *)raw_key, NULL, NULL);
}

// 2) Speck block encrypt (16 bytes)

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

// 3) PRF ใช้ Speck (แทน AES เดิม)

static void FF1_PRF_Speck(const FF1_Key_Speck *key,
                          const uint8_t *M, size_t Mlen,
                          uint8_t Y[16])
{
    uint8_t X[16] = {0};
    size_t blocks = (Mlen + 15) / 16;

    for (size_t j = 0; j < blocks; j++) {
        uint8_t tmp[16] = {0};
        const uint8_t *Mj = M + 16*j;
        size_t remain = (Mlen - 16*j);
        size_t len = remain < 16 ? remain : 16;

        for (size_t i = 0; i < len; i++) {
            tmp[i] = X[i] ^ Mj[i];
        }
        Speck128_Encrypt_Block(tmp, X, key);
    }
    memcpy(Y, X, 16);
}

// 4) helper ของ FF1 (copy จาก FF1 เดิม แต่เปลี่ยนชื่อกันชน)

static uint32_t NUM2_Speck(const uint8_t *X, size_t m)
{
    uint32_t x = 0;
    for (size_t i = 0; i < m; i++) {
        x = (x << 1) | (X[i] & 1);
    }
    return x;
}

static void STR2_Speck(uint32_t x, uint8_t *X, size_t m)
{
    for (size_t i = 0; i < m; i++) {
        X[m - 1 - i] = (uint8_t)(x & 1);
        x >>= 1;
    }
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

// 5) FF1 core (binary, radix 2) เวอร์ชัน Speck

static void FF1EncryptBinary_Speck(const FF1_Key_Speck *K,
                                   uint8_t *X, size_t n,
                                   const uint8_t *T, size_t t)
{
    size_t u = n / 2;
    size_t v = n - u;
    uint8_t *A = X;
    uint8_t *B = X + u;

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

    uint32_t d = 4 * ((v + 15) / 16);
    if (d < 4) d = 4;

    size_t b = ((t + 15) / 16) * 16;
    if (b == 0) b = 16;

    uint8_t Q[64];
    uint8_t PQ[16 + 64];

    for (int i = 0; i < 10; i++) {
        size_t m = (i % 2 == 0) ? v : u;
        size_t mlen = m;

        uint32_t numB = (i % 2 == 0) ? NUM2_Speck(B, v) : NUM2_Speck(A, u);

        memset(Q, 0, sizeof(Q));
        if (t > 0) memcpy(Q, T, t);
        Q[b - 1] = (uint8_t)i;

        uint32_t tmp = numB;
        for (size_t k = 0; k < d; k++) {
            Q[b + d - 1 - k] = (uint8_t)(tmp & 0xFF);
            tmp >>= 8;
        }

        size_t Qlen = b + d;
        size_t PQlen = 16 + Qlen;
        memcpy(PQ, P, 16);
        memcpy(PQ + 16, Q, Qlen);
        if (PQlen < 16) {
            memset(PQ + PQlen, 0, 16 - PQlen);
            PQlen = 16;
        }

        uint8_t Y[16];
        FF1_PRF_Speck(K, PQ, PQlen, Y);

        uint32_t y = 0;
        for (size_t k = 0; k < d; k++) {
            y = (y << 8) | Y[k];
        }

        uint32_t a = (i % 2 == 0) ? NUM2_Speck(A, mlen) : NUM2_Speck(B, mlen);
        uint32_t mod = (mlen >= 32) ? 0xFFFFFFFFu : ((1u << mlen) - 1u);
        uint32_t c = (a + y) & mod;

        uint8_t C[32];
        STR2_Speck(c, C, mlen);

        if (i % 2 == 0) {
            memcpy(A, B, v);
            memcpy(B, C, mlen);
        } else {
            memcpy(B, A, u);
            memcpy(A, C, mlen);
        }
    }
}

// 6) FF1Permute_Speck: public API

uint32_t FF1Permute_Speck(uint32_t index,
                          uint32_t maxblocks,
                          const FF1_Key_Speck *key,
                          uint32_t tweak)
{
    if (index >= maxblocks) return index;

    uint32_t n = ceillog2_Speck(maxblocks);
    if (n == 0) return index;
    if (n > 32) n = 32;

    uint8_t X[32];
    for (uint32_t i = 0; i < n; i++) {
        X[n - 1 - i] = (uint8_t)((index >> i) & 1u);
    }

    uint8_t T[4];
    T[0] = (uint8_t)((tweak >> 24) & 0xFF);
    T[1] = (uint8_t)((tweak >> 16) & 0xFF);
    T[2] = (uint8_t)((tweak >> 8)  & 0xFF);
    T[3] = (uint8_t)(tweak & 0xFF);

    FF1EncryptBinary_Speck(key, X, n, T, sizeof(T));

    uint32_t out = 0;
    for (uint32_t i = 0; i < n; i++) {
        out = (out << 1) | (X[i] & 1u);
    }

    if (out >= maxblocks)
        return FF1Permute_Speck(out, maxblocks, key, tweak + 1);

    return out;
}
