#!/usr/bin/env python3
"""READ-ONLY raw integrity audit for wall30 B-sweep (24 runs).

Does not modify, regenerate, overwrite, clean, or rename any raw log files.
Writes only a new audit report under analysis/wall30_bsweep/.
"""
from __future__ import annotations

import csv
import hashlib
import re
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

LOG = Path(__file__).resolve().parents[1] / "logs"
OUT = Path(__file__).resolve().parents[1] / "analysis" / "wall30_bsweep" / "RAW_INTEGRITY_AUDIT.md"

CONFIGS = [
    ("noattest", "phase2r_noattest"),
    ("B64", "phase3r_B64"),
    ("B128", "phase3r_B128"),
    ("B256", "phase3r_B256"),
    ("B512", "phase3r_B512"),
    ("B1024", "phase3r_B1024"),
    ("B2048", "phase3r_B2048"),
    ("B4096", "phase3r_B4096"),
]
SUFFIXES = [
    "_uart_capture.txt",
    "_sonar_timing.csv",
    "_wheel_timing.csv",
    "_obs_per_second.csv",
    "_run_meta.txt",
]


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_mono(xs: List[int]) -> Tuple[bool, int]:
    if len(xs) < 2:
        return True, 0
    bad = sum(1 for a, b in zip(xs, xs[1:]) if b < a)
    return bad == 0, bad


def check_dups(xs: List[int]) -> int:
    return sum(1 for v in Counter(xs).values() if v > 1)


def audit_run(cfg: str, prefix: str, run: int) -> Tuple[Dict[str, Any], List[Tuple[str, int, str]]]:
    tag = f"{prefix}_r{run}"
    files = {s: LOG / f"{tag}{s}" for s in SUFFIXES}
    present = {s: files[s].is_file() and files[s].stat().st_size > 0 for s in SUFFIXES}
    complete = all(present.values())
    issues: List[str] = []
    if not complete:
        issues.append("missing/empty:" + ",".join(s for s, ok in present.items() if not ok))

    sonar_n = wheel_n = None
    s_mono = w_mono = None
    s_dups = w_dups = None
    s_err: Optional[int] = None
    hz_uart_s = hz_uart_w = None
    ovf_s = ovf_w = None
    freq_ok: Optional[bool] = None
    raw_ts: Optional[bool] = None
    t0 = tend = None
    uart = ""

    hashes: List[Tuple[str, int, str]] = []
    for s, p in files.items():
        if p.is_file():
            hashes.append((f"{tag}{s}", p.stat().st_size, sha256_file(p)))

    if present["_uart_capture.txt"]:
        uart = files["_uart_capture.txt"].read_text(encoding="utf-8", errors="replace")
        m = re.search(
            r"sonar_starts=(\d+).*?sonar_freq_mHz=(\d+).*?wheel_starts=(\d+).*?wheel_freq_mHz=(\d+)",
            uart,
            re.S,
        )
        if m:
            hz_uart_s = int(m.group(2)) / 1000.0
            hz_uart_w = int(m.group(4)) / 1000.0
        mo = re.search(r"sonar_ovf=(\d+)\s+wheel_ovf=(\d+)", uart)
        if mo:
            ovf_s, ovf_w = int(mo.group(1)), int(mo.group(2))
        mt0 = re.search(r"T0=(\d+)", uart)
        mt1 = re.search(r"Tend=(\d+)", uart)
        if mt0:
            t0 = int(mt0.group(1))
        if mt1:
            tend = int(mt1.group(1))

    if present["_sonar_timing.csv"]:
        try:
            with files["_sonar_timing.csv"].open(encoding="utf-8", errors="replace") as f:
                rdr = csv.DictReader(f)
                srows = list(rdr)
                cols = set(rdr.fieldnames or [])
            sonar_n = len(srows)
            need = {"actual_start_us", "finish_us"}
            if not need.issubset(cols):
                issues.append(f"sonar_missing_cols:{sorted(need - cols)}")
                raw_ts = False
            else:
                starts = [int(r["actual_start_us"]) for r in srows]
                finishes = [int(r["finish_us"]) for r in srows]
                if (all(x == 0 for x in starts)) or (sonar_n > 2 and len(set(starts)) <= 1):
                    issues.append("sonar_starts_degenerate")
                    raw_ts = False
                else:
                    raw_ts = True
                s_mono, s_bad = check_mono(starts)
                if not s_mono:
                    issues.append(f"sonar_nonmono={s_bad}")
                s_dups = check_dups(starts)
                if s_dups:
                    issues.append(f"sonar_dup_starts={s_dups}")
                bad_fin = sum(1 for a, b in zip(starts, finishes) if b < a)
                if bad_fin:
                    issues.append(f"sonar_finish_before_start={bad_fin}")
                s_err = sum(1 for r in srows if int(r.get("status", "0")) != 0)
                if s_err:
                    issues.append(f"sonar_status_err={s_err}")
        except Exception as e:
            issues.append(f"sonar_parse:{e}")

    if present["_wheel_timing.csv"]:
        try:
            with files["_wheel_timing.csv"].open(encoding="utf-8", errors="replace") as f:
                rdr = csv.DictReader(f)
                wrows = list(rdr)
                cols = list(rdr.fieldnames or [])
            wheel_n = len(wrows)
            scol = (
                "s_exec_us"
                if "s_exec_us" in cols
                else ("window_end_us" if "window_end_us" in cols else None)
            )
            if scol is None or "finish_us" not in cols or "window_open_us" not in cols:
                issues.append("wheel_missing_ts_cols")
                raw_ts = False
            else:
                opens = [int(r["window_open_us"]) for r in wrows]
                ends = [int(r[scol]) for r in wrows]
                if (all(x == 0 for x in ends)) or (wheel_n > 2 and len(set(ends)) <= 1):
                    issues.append("wheel_ends_degenerate")
                    raw_ts = False
                elif raw_ts is None:
                    raw_ts = True
                w_mono, w_bad = check_mono(ends)
                if not w_mono:
                    issues.append(f"wheel_nonmono={w_bad}")
                w_dups = check_dups(ends)
                if w_dups:
                    issues.append(f"wheel_dup_ends={w_dups}")
                bad_wo = sum(1 for o, e in zip(opens, ends) if e < o)
                if bad_wo:
                    issues.append(f"wheel_end_before_open={bad_wo}")
        except Exception as e:
            issues.append(f"wheel_parse:{e}")

    if sonar_n is not None and t0 is not None and tend is not None and tend > t0:
        hz_from_n = sonar_n / ((tend - t0) / 1e6)
        if hz_uart_s is not None:
            if abs(hz_from_n - hz_uart_s) > 0.05 and abs(sonar_n - round(hz_uart_s * 30)) > 1:
                issues.append(
                    f"freq_mismatch sonar n={sonar_n} uart_hz={hz_uart_s} from_n={hz_from_n:.3f}"
                )
                freq_ok = False
            else:
                freq_ok = True
        ms = re.search(r"sonar_starts=(\d+)", uart)
        if ms and int(ms.group(1)) != sonar_n:
            issues.append(f"sonar_starts_vs_csv {ms.group(1)}!={sonar_n}")
            freq_ok = False
        mw = re.search(r"wheel_starts=(\d+)", uart)
        if mw and wheel_n is not None and int(mw.group(1)) != wheel_n:
            issues.append(f"wheel_starts_vs_csv {mw.group(1)}!={wheel_n}")
            freq_ok = False

    if ovf_s not in (None, 0) or ovf_w not in (None, 0):
        issues.append(f"overflow s={ovf_s} w={ovf_w}")

    if present["_obs_per_second.csv"]:
        obs_lines = [
            ln
            for ln in files["_obs_per_second.csv"]
            .read_text(encoding="utf-8", errors="replace")
            .splitlines()
            if ln.strip()
        ]
        if len(obs_lines) < 2:
            issues.append("obs_per_second_too_short")

    status = (
        "PASS"
        if (complete and not issues and raw_ts is not False and freq_ok is not False)
        else "FAIL"
    )

    row = {
        "config": cfg,
        "run": run,
        "tag": tag,
        "complete": complete,
        "sonar_n": sonar_n,
        "wheel_n": wheel_n,
        "hz_uart_s": hz_uart_s,
        "hz_from_n": (sonar_n / 30.0) if sonar_n is not None else None,
        "raw_ts": raw_ts,
        "s_mono": s_mono,
        "w_mono": w_mono,
        "s_dups": s_dups,
        "w_dups": w_dups,
        "ovf_s": ovf_s,
        "ovf_w": ovf_w,
        "s_err": s_err,
        "freq_ok": freq_ok,
        "status": status,
        "issues": issues,
    }
    return row, hashes


def main() -> int:
    rows: List[Dict[str, Any]] = []
    hashes: List[Tuple[str, int, str]] = []
    for cfg, prefix in CONFIGS:
        for run in (1, 2, 3):
            row, hs = audit_run(cfg, prefix, run)
            rows.append(row)
            hashes.extend(hs)

    all_pass = all(r["status"] == "PASS" for r in rows)

    # Console table
    print(
        "| config | run | tag | files | Sonar | Wheel | raw TIM2 | mono S/W | "
        "dups S/W | ovf S/W | sens_err | freq | PASS/FAIL |"
    )
    print(
        "|--------|-----|-----|-------|-------|-------|----------|----------|"
        "----------|---------|----------|------|-----------|"
    )
    for r in rows:
        print(
            f"| {r['config']} | {r['run']} | `{r['tag']}` | "
            f"{'OK' if r['complete'] else 'MISS'} | {r['sonar_n']} | {r['wheel_n']} | "
            f"{'OK' if r['raw_ts'] else 'FAIL'} | {r['s_mono']}/{r['w_mono']} | "
            f"{r['s_dups']}/{r['w_dups']} | {r['ovf_s']}/{r['ovf_w']} | {r['s_err']} | "
            f"{'OK' if r['freq_ok'] else r['freq_ok']} | **{r['status']}** |"
        )

    print()
    print(f"OVERALL {'PASS' if all_pass else 'FAIL'} runs={len(rows)}")
    for r in rows:
        if r["issues"]:
            print(f"  ISSUES {r['tag']}: {'; '.join(r['issues'])}")

    # Audit report only (does not touch raw)
    lines: List[str] = [
        "# wall30 B-sweep READ-ONLY raw integrity audit",
        "",
        f"Overall: **{'PASS' if all_pass else 'FAIL'}** ({len(rows)}/24 runs). "
        "No raw files were modified, regenerated, overwritten, cleaned, or renamed.",
        "",
        "| config | run | tag | files | Sonar rows | Wheel rows | raw TIM2 | "
        "mono S/W | dups S/W | ovf S/W | sens_err | freq vs n | result |",
        "|--------|-----|-----|-------|------------|------------|----------|"
        "----------|----------|---------|----------|-----------|--------|",
    ]
    for r in rows:
        lines.append(
            f"| {r['config']} | {r['run']} | `{r['tag']}` | "
            f"{'OK' if r['complete'] else 'MISS'} | {r['sonar_n']} | {r['wheel_n']} | "
            f"{'OK' if r['raw_ts'] else 'FAIL'} | {r['s_mono']}/{r['w_mono']} | "
            f"{r['s_dups']}/{r['w_dups']} | {r['ovf_s']}/{r['ovf_w']} | {r['s_err']} | "
            f"{'OK' if r['freq_ok'] else r['freq_ok']} | **{r['status']}** |"
        )
    lines += ["", "## Issues detail"]
    any_issue = False
    for r in rows:
        if r["issues"]:
            any_issue = True
            lines.append(f"- `{r['tag']}`: {'; '.join(r['issues'])}")
    if not any_issue:
        lines.append("- none")

    lines += [
        "",
        "## Notes",
        "",
        "- Sonar TIM2 columns checked: `actual_start_us`, `finish_us`.",
        "- Wheel TIM2 columns checked: `window_open_us`, `s_exec_us`/`window_end_us`, `finish_us`.",
        "- Frequency check: CSV row count vs UART `sonar_starts`/`wheel_starts`/`freq_mHz` and TIM2 window.",
        "- SHA256 below covers every existing raw companion file for the 24 tags.",
        "",
        "## SHA256 manifest (raw files)",
        "",
        "| file | bytes | sha256 |",
        "|------|-------|--------|",
    ]
    for name, sz, hx in hashes:
        lines.append(f"| `{name}` | {sz} | `{hx}` |")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"WROTE {OUT}")
    print(f"HASH_COUNT {len(hashes)}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
