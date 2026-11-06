"""Secure-world footprint of the attestation path, from committed build outputs.

Inputs per variant (extracted by git show into %TEMP%/fp/<variant>/):
  s.elf (disassembled -> dis.txt), nm.txt (nm -S), all.su (every .su of the Secure build).
Worst-case stack = max over call paths from the NSC entry of the sum of .su frames
(+8 B per call for nothing: Cortex-M bl does not push; the callee's frame already
includes pushed LR). Code = sum of sizes of functions reachable from the entry.
"""
import os, re, sys, collections

BASE = os.path.join(os.environ["TEMP"], "fp")
ENTRY = {
    "smarm": "SECURE_ShuffledHMAC_secure",
    "prng": "SECURE_RTSMARM_ShuffledHMAC_secure",
    "fpe": "SECURE_RTSMARM_FF1_Speck_ShuffledHMAC_secure",
}

_open = open

def open(path, encoding=None, errors=None):
    raw = _open(path, "rb").read()
    enc = "utf-16" if raw[:2] in (b"\xff\xfe", b"\xfe\xff") else "utf-8"
    return raw.decode(enc, errors="ignore").splitlines()

def load(v):
    d = os.path.join(BASE, v)
    su = {}
    for line in open(os.path.join(d, "all.su"), encoding="utf-8", errors="ignore"):
        parts = line.rstrip("\n").split("\t")
        if len(parts) < 3:
            continue
        name = parts[0].split(":")[-1]
        su[name] = max(su.get(name, 0), int(parts[1]))
    su.update({"memcpy": 8, "memset": 0})  # newlib leaf routines, frames read from disassembly
    calls = collections.defaultdict(set)
    cur = None
    hdr = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:$")
    call = re.compile(r"\t(bl|blx|b\.w|b)\s+[0-9a-f]+ <([^>+]+)>")
    for line in open(os.path.join(d, "dis.txt"), encoding="utf-8", errors="ignore"):
        m = hdr.match(line.strip())
        if m:
            cur = m.group(2)
            continue
        m = call.search(line)
        if m and cur and m.group(2) != cur:
            calls[cur].add(m.group(2))
    size, kind, addr = {}, {}, {}
    for line in open(os.path.join(d, "nm.txt"), encoding="utf-8", errors="ignore"):
        p = line.split()
        if len(p) == 4:
            size[p[3]] = int(p[1], 16)
            kind[p[3]] = p[2]
            addr[p[3]] = int(p[0], 16)
        elif len(p) == 3 and p[1] in "Tt":
            kind[p[2]] = p[1]
            addr[p[2]] = int(p[0], 16)
    text = sorted((a, n) for n, a in addr.items() if kind[n] in "Tt" and a < 0x0C03E000)
    for (a, n), (a2, _) in zip(text, text[1:]):
        if n not in size:
            size[n] = a2 - a
    return su, calls, size, kind

def worst(fn, su, calls, missing, stack=()):
    if fn in stack:
        return 0, [fn + "(rec)"]
    best, path = 0, []
    for c in calls.get(fn, ()):
        s, p = worst(c, su, calls, missing, stack + (fn,))
        if s > best:
            best, path = s, p
    if fn not in su:
        missing.add(fn)
    return su.get(fn, 0) + best, [f"{fn}({su.get(fn, '?')})"] + path

def reach(fn, calls):
    seen, st = set(), [fn]
    while st:
        f = st.pop()
        if f in seen:
            continue
        seen.add(f)
        st.extend(calls.get(f, ()))
    return seen

for v, e in ENTRY.items():
    su, calls, size, kind = load(v)
    missing = set()
    s, p = worst(e, su, calls, missing)
    r = reach(e, calls)
    code = sum(size.get(f, 0) for f in r if kind.get(f, "").lower() == "t")
    print(f"=== {v}: entry {e}")
    print(f"  worst-case stack {s} B via {' -> '.join(p)}")
    print(f"  reachable functions {len(r)}, code {code} B")
    print("  no .su (asm/lib):", sorted(f for f in r if f not in su))
    big = sorted(((size.get(f, 0), f) for f in r), reverse=True)[:12]
    print("  largest:", big)
