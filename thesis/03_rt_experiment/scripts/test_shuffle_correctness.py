#!/usr/bin/env python3
"""Host-side permutation correctness for Stage-1 PRNG bitarray + fixed FF1 Feistel.

Does not flash hardware. Mirrors Secure algorithms for property tests:
  - bitarray used-bits / k-th zero produces a permutation of [0, n)
  - fixed FF1-style Feistel is a bijection; inverse round-trips
  - all |M|/B block counts for B in {64,128,256,512,1024,2048,4096}, |M|=128 KiB
  - edge n values (1, 2, 3, 7, 100, 1000)

Exit 0 on success.
"""
from __future__ import annotations

import hashlib
import os
import random
import sys

TOTAL_SIZE = 0x20000  # 128 KiB — smoke default
BLOCK_SIZES = (64, 128, 256, 512, 1024, 2048, 4096)


# ---- PRNG bitarray (port of rtsmarm_find_kth_zero selection) -----------------
def bitarray_permute(n: int, draws: list[int]) -> list[int]:
    used = [0] * n
    out = []
    for round_i, k in enumerate(draws):
        remaining = n - round_i
        assert 0 <= k < remaining
        count = 0
        idx = 0
        for i in range(n):
            if used[i] == 0:
                if count == k:
                    idx = i
                count += 1
        used[idx] = 1
        out.append(idx)
    return out


def test_bitarray() -> None:
    rng = random.Random(0xC0FFEE)
    for n in [1, 2, 3, 7, 16, 128, 1024] + [TOTAL_SIZE // b for b in BLOCK_SIZES]:
        draws = []
        for r in range(n):
            draws.append(rng.randrange(n - r))
        perm = bitarray_permute(n, draws)
        if sorted(perm) != list(range(n)):
            raise AssertionError(f"bitarray not a permutation n={n}")
    print(f"PASS bitarray permutations (incl. |M|/B for all B)")


# ---- Fixed FF1-style Feistel (matches ff1_speck.c integer form; mock PRF) ---
def ceillog2(x: int) -> int:
    n, v = 0, x - 1
    while v > 0:
        v >>= 1
        n += 1
    return n


def prf4(key: bytes, tweak: int, rnd: int, b_val: int) -> int:
    data = tweak.to_bytes(4, "big") + bytes([rnd & 0xFF]) + b_val.to_bytes(4, "big")
    return int.from_bytes(hashlib.sha256(key + data).digest()[:4], "big")


def ff1_encrypt_bits(x: int, nbits: int, key: bytes, tweak: int) -> int:
    u = nbits // 2
    v = nbits - u
    A = x >> v
    B = x & ((1 << v) - 1)
    for i in range(10):
        m = u if (i % 2) == 0 else v
        y = prf4(key, tweak, i, B)
        mod = (1 << m) - 1 if m < 32 else 0xFFFFFFFF
        c = (A + y) & mod
        A, B = B, c
    return (A << v) | B


def ff1_decrypt_bits(x: int, nbits: int, key: bytes, tweak: int) -> int:
    u = nbits // 2
    v = nbits - u
    A = x >> v
    B = x & ((1 << v) - 1)
    for i in range(9, -1, -1):
        m = u if (i % 2) == 0 else v
        # forward: A'=B, B'=(A+y)&mod with y=PRF(B)
        y = prf4(key, tweak, i, A)  # B_prev = A_curr
        mod = (1 << m) - 1 if m < 32 else 0xFFFFFFFF
        c = B
        B = A
        A = (c - y) & mod
    return (A << v) | B


def ff1_permute(index: int, maxblocks: int, key: bytes, tweak: int) -> int:
    # Match firmware: ceillog2(1)==0 → identity
    nbits = ceillog2(maxblocks)
    if nbits == 0:
        return index
    out = ff1_encrypt_bits(index, nbits, key, tweak)
    while out >= maxblocks:
        out = ff1_encrypt_bits(out, nbits, key, tweak)
    return out


def ff1_inverse_permute(image: int, maxblocks: int, key: bytes, tweak: int) -> int:
    """Recover preimage: Feistel decrypt when power-of-2 and nbits>=2; else search."""
    nbits = ceillog2(maxblocks)
    if nbits == 0:
        return image
    if (maxblocks & (maxblocks - 1)) == 0 and nbits >= 2:
        return ff1_decrypt_bits(image, nbits, key, tweak)
    for cand in range(maxblocks):
        if ff1_permute(cand, maxblocks, key, tweak) == image:
            return cand
    raise AssertionError("no inverse")


def test_ff1() -> None:
    rng = random.Random(0xF1F1)
    ns = [TOTAL_SIZE // b for b in BLOCK_SIZES] + [1, 2, 3, 7, 100, 1000]
    for n in ns:
        key = rng.randbytes(16)
        tweak = rng.randrange(2**32)
        outs = [ff1_permute(i, n, key, tweak) for i in range(n)]
        if sorted(outs) != list(range(n)):
            raise AssertionError(f"FF1 not bijective n={n}")
        sample = list(range(n)) if n <= 256 else list(rng.sample(range(n), 64))
        for i in sample:
            j = ff1_permute(i, n, key, tweak)
            back = ff1_inverse_permute(j, n, key, tweak)
            if back != i:
                raise AssertionError(f"FF1 inverse fail n={n} i={i} j={j} back={back}")
    print(f"PASS FF1 bijection + inverse checks ({len(ns)} domain sizes)")

def main() -> int:
    test_bitarray()
    test_ff1()
    print("ALL HOST SHUFFLE TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
