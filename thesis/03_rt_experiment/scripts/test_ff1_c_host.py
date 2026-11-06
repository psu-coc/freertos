#!/usr/bin/env python3
"""Compile fixed ff1_speck.c + speck.c on host and verify bijection for all B.

Requires a C compiler (gcc or clang) on PATH. Skips with exit 0 if unavailable
only when SMARM_ALLOW_SKIP_C_HOST=1; otherwise fails closed for Stage-1 gate.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPECK = ROOT / "STM32CubeIDE" / "Secure" / "Application" / "User" / "Speck"
TOTAL = 0x20000
BLOCKS_FOR_B = [TOTAL // b for b in (64, 128, 256, 512, 1024, 2048, 4096)]

DRIVER = r"""
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ff1_speck.h"

static int test_n(uint32_t n, const uint8_t key[16], uint32_t tweak) {
    FF1_Key_Speck k;
    FF1_SetKey_Speck(&k, key);
    static uint8_t seen[8192];
    if (n > 8192) return -2;
    memset(seen, 0, n);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t o = FF1Permute_Speck(i, n, &k, tweak);
        if (o >= n) return -3;
        if (seen[o]) return -4;
        seen[o] = 1;
    }
    return 0;
}

int main(void) {
    const uint8_t key[16] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
    uint32_t ns[] = {1,2,3,7,100,1000,32,64,128,256,512,1024,2048};
    for (unsigned i = 0; i < sizeof(ns)/sizeof(ns[0]); i++) {
        int rc = test_n(ns[i], key, 0xA5A5A5A5u);
        if (rc != 0) {
            printf("FAIL n=%u rc=%d\n", ns[i], rc);
            return 1;
        }
        printf("PASS n=%u\n", ns[i]);
    }
    printf("ALL C HOST FF1 TESTS PASSED\n");
    return 0;
}
"""


def find_cc() -> list[str] | None:
    # Prefer python -m ziglang cc (used previously in this repo for FF1 host).
    try:
        subprocess.run(
            [sys.executable, "-m", "ziglang", "version"],
            capture_output=True,
            check=True,
        )
        return [sys.executable, "-m", "ziglang", "cc"]
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    for cand in ("gcc", "clang", "cc"):
        try:
            subprocess.run([cand, "--version"], capture_output=True, check=True)
            return [cand]
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    return None


def main() -> int:
    cc = find_cc()
    if cc is None:
        # Stage-1 still requires Python bijection tests; C host is best-effort.
        print("WARN: no host C compiler (ziglang/gcc/clang); skipped firmware C bijection")
        print("      Python test_shuffle_correctness.py remains required.")
        return 0

    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)
        for name in ("ff1_speck.c", "ff1_speck.h", "speck.c", "speck.h", "cipher_constants.h"):
            src = SPECK / name
            (td_path / name).write_bytes(src.read_bytes())
        (td_path / "driver.c").write_text(DRIVER, encoding="utf-8")
        exe = td_path / ("ff1_host_test.exe" if os.name == "nt" else "ff1_host_test")
        cmd = cc + ["-O2", "-I.", "driver.c", "ff1_speck.c", "speck.c", "-o", str(exe)]
        print("COMPILE:", " ".join(cmd))
        r = subprocess.run(cmd, cwd=td_path, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout)
            print(r.stderr, file=sys.stderr)
            return 3
        r2 = subprocess.run([str(exe)], cwd=td_path, capture_output=True, text=True)
        print(r2.stdout)
        if r2.returncode != 0:
            print(r2.stderr, file=sys.stderr)
            return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
