#!/usr/bin/env python3
"""Host correctness: fixed-challenge attestation digests for Original/PRNG/FPE.

Compiles firmware AES + HMAC + FF1-Speck against a synthetic 128 KiB |M|.
This is a CORRECTNESS test (not wall30 performance). On-target smoke does not
print digests; use these vectors for offline algorithm self-consistency.
"""
from __future__ import annotations

import hashlib
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = Path(__file__).resolve().parent
AES_DIR = ROOT / "STM32CubeIDE" / "Secure" / "Application" / "User" / "Aesnew"
HMAC_DIR = ROOT / "STM32CubeIDE" / "Secure" / "Application" / "User" / "Crypto" / "hmac-sha256"
SPECK_DIR = ROOT / "STM32CubeIDE" / "Secure" / "Application" / "User" / "Speck"
DRIVER = SCRIPT_DIR / "host_attest_digest_ref.c"

# Locked expected digests after first successful run (golden). Updated only when
# algorithm intentionally changes. Empty = compute & print; gate records them.
GOLDEN = {
    "original": None,  # filled after first PASS and locked in report
    "prng": None,
    "fpe": None,
}


def find_cc() -> list[str] | None:
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
    if not cc:
        print("FAIL: no C compiler (ziglang/gcc/clang)", file=sys.stderr)
        return 1

    # Build under analysis/ (Windows WDAC often blocks %TEMP% executables).
    out = SCRIPT_DIR.parent / "analysis" / "stage2_smoke_B1024"
    out.mkdir(parents=True, exist_ok=True)
    exe = out / ("digest_ref.exe" if sys.platform.startswith("win") else "digest_ref")
    cmd = cc + [
        "-O2",
        f"-I{AES_DIR}",
        f"-I{HMAC_DIR}",
        f"-I{SPECK_DIR}",
        "-DCTR=1",
        str(DRIVER),
        str(AES_DIR / "aes.c"),
        str(HMAC_DIR / "hmac-sha256.c"),
        str(SPECK_DIR / "ff1_speck.c"),
        str(SPECK_DIR / "speck.c"),
        "-o",
        str(exe),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("COMPILE FAIL", file=sys.stderr)
        print(r.stderr or r.stdout, file=sys.stderr)
        return 1

    # Run twice — independent deterministic check.
    runs = []
    for _ in range(2):
        r2 = subprocess.run([str(exe)], capture_output=True, text=True)
        if r2.returncode != 0:
            print(r2.stderr, file=sys.stderr)
            return 1
        digests = {}
        for line in r2.stdout.strip().splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[0] in ("original", "prng", "fpe"):
                digests[parts[0]] = parts[1]
        if set(digests) != {"original", "prng", "fpe"}:
            print("FAIL: missing digest lines", file=sys.stderr)
            return 1
        if len({digests["original"], digests["prng"], digests["fpe"]}) != 3:
            print("FAIL: digests not distinct", file=sys.stderr)
            return 1
        runs.append(digests)

    if runs[0] != runs[1]:
        print("FAIL: digests not deterministic across runs", file=sys.stderr)
        return 1

    print(r2.stdout, end="")
    golden_path = out / "DIGEST_REF_GOLDEN.txt"
    lines = [
        "# CORRECTNESS vectors — fixed challenge + synthetic |M|",
        "# (Distinct from wall30 PERFORMANCE smoke; digests not on UART.)",
        "# challenge = 0123456789abcdeffedcba9876543210",
        "# secret = MySecureKey123",
        "# |M| = 128KiB, mem[i]=((i/B)*17)^((i*131+17)), B=1024, passes=1",
        "# NOT equal to on-flash 0x08060000 contents",
    ]
    for name in ("original", "prng", "fpe"):
        lines.append(f"{name} {runs[0][name]}")
    golden_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {golden_path}")
    h = hashlib.sha256(golden_path.read_bytes()).hexdigest()[:16]
    print(f"golden_sha16 {h}")
    print("PASS test_attest_digest_ref")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
