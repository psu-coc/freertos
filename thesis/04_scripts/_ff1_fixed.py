"""Corrected radix-2 FF1-style Feistel (A: u bits, B: v bits, alternating
m = u / v as in NIST SP 800-38G) with cycle-walking under the same tweak.
Counts PRF calls to show the cost equals the buggy version (10 per call)."""
import hashlib
import os

calls = 0


def prf(key, tweak, rnd, b_val):
    global calls
    calls += 1
    data = tweak.to_bytes(4, "big") + bytes([rnd]) + b_val.to_bytes(4, "big")
    return int.from_bytes(hashlib.sha256(key + data).digest()[:4], "big")


def ceillog2(x):
    n, v = 0, x - 1
    while v > 0:
        v >>= 1
        n += 1
    return n


def enc_bits(x, nbits, key, tweak):
    u = nbits // 2
    v = nbits - u
    A = x >> v
    B = x & ((1 << v) - 1)
    for i in range(10):
        m = u if i % 2 == 0 else v
        c = (A + prf(key, tweak, i, B)) & ((1 << m) - 1)
        A, B = B, c
    return (A << v) | B


def ff1_permute(index, maxblocks, key, tweak):
    nbits = max(1, ceillog2(maxblocks))
    out = enc_bits(index, nbits, key, tweak)
    while out >= maxblocks:
        out = enc_bits(out, nbits, key, tweak)
    return out


for nblk in [8192, 4096, 2048, 1024, 512, 256, 128, 100, 1000]:
    key = os.urandom(16)
    tweak = int.from_bytes(os.urandom(4), "big")
    calls = 0
    outs = [ff1_permute(i, nblk, key, tweak) for i in range(nblk)]
    ok = sorted(outs) == list(range(nblk))
    print(f"n={nblk:5d} bijection={ok} PRF calls per index={calls / nblk:.2f}")
