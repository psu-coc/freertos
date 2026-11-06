"""Line-by-line port of ff1_runtime:ff1_speck.c (FF1EncryptBinary_Speck,
FF1Permute_Speck) to test whether it is a bijection on [0, maxblocks).

The block-cipher PRF is replaced by a keyed hash; bijectivity of a Feistel
network does not depend on the round function, only on the data handling.
"""
import hashlib
import os


def prf(key, data):
    return hashlib.sha256(key + bytes(data)).digest()[:16]


def num2(bits, off, m):
    x = 0
    for i in range(m):
        x = ((x << 1) | (bits[off + i] & 1)) & 0xFFFFFFFF
    return x


def str2(x, m):
    out = [0] * m
    for i in range(m):
        out[m - 1 - i] = x & 1
        x >>= 1
    return out


def ceillog2(x):
    n, v = 0, x - 1
    while v > 0:
        v >>= 1
        n += 1
    return n


def ff1_encrypt_binary(key, X, n, T):
    t = len(T)
    u = n // 2
    v = n - u
    A, Boff = 0, u  # offsets into X
    P = [0x01, 0x00, 0x00, 0x02, 0x0A,
         (n >> 24) & 0xFF, (n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF,
         0, 0, t & 0xFF, 0, 0, 0, 0]
    d = 4 * ((v + 15) // 16)
    if d < 4:
        d = 4
    b = ((t + 15) // 16) * 16
    if b == 0:
        b = 16
    for i in range(10):
        m = v if i % 2 == 0 else u
        mlen = m
        numB = num2(X, Boff, v) if i % 2 == 0 else num2(X, A, u)
        Q = [0] * 64
        Q[:t] = list(T)
        Q[b - 1] = i
        tmp = numB
        for k in range(d):
            Q[b + d - 1 - k] = tmp & 0xFF
            tmp >>= 8
        PQ = P + Q[:b + d]
        Y = prf(key, PQ)
        y = 0
        for k in range(d):
            y = ((y << 8) | Y[k]) & 0xFFFFFFFF
        a = num2(X, A, mlen) if i % 2 == 0 else num2(X, Boff, mlen)
        mod = 0xFFFFFFFF if mlen >= 32 else (1 << mlen) - 1
        c = (a + y) & mod
        C = str2(c, mlen)
        if i % 2 == 0:
            X[A:A + v] = X[Boff:Boff + v]          # memcpy(A, B, v)
            X[Boff:Boff + mlen] = C                # memcpy(B, C, mlen)
        else:
            X[Boff:Boff + u] = X[A:A + u]          # memcpy(B, A, u)
            X[A:A + mlen] = C                      # memcpy(A, C, mlen)
        del X[n:]  # memcpy past n would write outside the logical buffer


def ff1_permute(index, maxblocks, key, tweak, depth=0):
    if index >= maxblocks:
        return index
    n = ceillog2(maxblocks)
    if n == 0:
        return index
    X = [(index >> (n - 1 - j)) & 1 for j in range(n)]
    X += [0] * 32  # C buffer is uint8_t X[32]; allow overrun like the C code
    T = [(tweak >> 24) & 0xFF, (tweak >> 16) & 0xFF, (tweak >> 8) & 0xFF, tweak & 0xFF]
    ff1_encrypt_binary(key, X, n, T)
    out = 0
    for j in range(n):
        out = (out << 1) | (X[j] & 1)
    if out >= maxblocks:
        return ff1_permute(out, maxblocks, key, (tweak + 1) & 0xFFFFFFFF, depth + 1)
    return out


MEM = 512 * 1024
for B in [64, 128, 256, 512, 1024, 2048, 4096]:
    nblk = MEM // B
    results = []
    for trial in range(5):
        key = os.urandom(16)
        tweak = int.from_bytes(os.urandom(4), "big")
        outs = [ff1_permute(i, nblk, key, tweak) for i in range(nblk)]
        distinct = len(set(outs))
        results.append(distinct)
    bits = ceillog2(nblk)
    covered = sum(results) / len(results)
    print(f"B={B:5d} n={nblk:5d} bits={bits:2d} ({'odd ' if bits % 2 else 'even'}) "
          f"distinct outputs per trial={results}  avg coverage={covered / nblk * 100:.1f}%")
