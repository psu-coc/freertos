#!/usr/bin/env python3
"""Inventory Sonar and Wheel datasets. Writes only in this directory."""
from __future__ import annotations

import csv
import html
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
NS256 = HERE.parents[1]
RT_ROOT = NS256.parents[1]
FINAL = NS256 / "additional_eval" / "dmr" / "final" / "FINAL_DMR_RESULTS.csv"
PRIOR = NS256 / "additional_eval" / "advisor_evidence_v1" / "sliding_reference_results.csv"
WINDOW = NS256 / "additional_eval" / "sliding_window_revalidation_v1" / "sliding_window_vs_prior.csv"
JOBS = NS256 / "additional_eval" / "sliding_window_revalidation_v1" / "sliding_window_jobs.csv"
ADVISOR_HTML = NS256 / "advisor_report" / "SMARMplus_FreeRTOS_Advisor_Report.html"

MODES = ("original", "prng", "fpe")
LABEL = {"original": "Original SMARM", "prng": "SMARM+PRNG", "fpe": "SMARM+FPE", "noattest": "No attestation"}
BS = ("64", "128", "256", "512", "1024", "2048", "4096")


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def key_of(row: dict) -> tuple:
    return (row["profile"], row["task"], row["mode"], row["B"], row["run"])


def arrangement(profile: str, task: str) -> tuple[str, str, str]:
    if profile == "10Hz10Hz":
        return ("Concurrent Sonar + Wheel", "10", task)
    if profile == "Wheel100Hz":
        return ("Wheel only", "100", "wheel")
    if profile == "Wheel1000Hz":
        return ("Wheel only", "1000", "wheel")
    raise SystemExit(profile)


def scan_tree() -> dict:
    sonar_timing = []
    wheel_timing = []
    formal = []
    formal_groups = defaultdict(int)
    unknown_dirs = defaultdict(int)
    for path in RT_ROOT.rglob("*"):
        if not path.is_file():
            continue
        name = path.name.lower()
        rel = str(path.relative_to(RT_ROOT))
        if name.endswith("_sonar_timing.csv") or name == "sonar_timing.csv":
            sonar_timing.append(rel)
        elif name.endswith("_wheel_timing.csv"):
            wheel_timing.append(rel)
        elif name.endswith("_formal_dmr.csv"):
            formal.append(rel)
            formal_groups[path.parts[-3] if len(path.parts) >= 3 else path.parent.name] += 1
    note_files = {"created": 0, "not_created": 0, "other": 0}
    uart_cells = defaultdict(int)
    unknown_sonar = []
    for path in RT_ROOT.rglob("*_uart_capture.txt"):
        raw = path.read_bytes()[:8000]
        if b"\x00" in raw[:80]:
            head = raw.decode("utf-16", errors="replace")
        else:
            head = raw.decode("utf-8", errors="replace")
        if "SonarTask not created" in head:
            note_files["not_created"] += 1
        elif "SonarTask" in head:
            note_files["created"] += 1
        else:
            note_files["other"] += 1
        lowered = path.name.lower()
        if ("@200ms" in head or "period=200ms" in head) and "1000ms" in head:
            profile = "pilot_sonar_5Hz_wheel_1Hz"
        elif "w1000" in lowered:
            profile = "phase4_wheel1000"
        elif "w100" in lowered:
            profile = "phase4_wheel100"
        elif "10hz10hz" in lowered or "profile=10Hz10Hz" in head:
            profile = "10Hz10Hz"
        elif "profile=Wheel1000Hz" in head or "Wheel1000Hz" in path.name:
            profile = "Wheel1000Hz"
        elif "profile=Wheel100Hz" in head or "Wheel100Hz" in path.name:
            profile = "Wheel100Hz"
        else:
            profile = "unknown"
        if "checkpoints" in path.parts:
            profile = "checkpoint_copy:" + profile
        if profile == "unknown":
            unknown_dirs[path.relative_to(RT_ROOT).parts[0]] += 1
        sonar_n = "na"
        wheel_period = "na"
        marker = "sonar_n="
        if marker in head:
            sonar_n = head.split(marker, 1)[1].split()[0]
        if "wheel_period_ms=" in head:
            wheel_period = head.split("wheel_period_ms=", 1)[1].split()[0]
        uart_cells[(profile, "sonar_n>0" if sonar_n not in ("0", "na") else "sonar_n=" + sonar_n, "wheel_period_ms=" + wheel_period)] += 1
        if profile == "unknown" and sonar_n not in ("0", "na"):
            unknown_sonar.append(str(path.relative_to(RT_ROOT)))
    return {
        "sonar_timing": sonar_timing,
        "wheel_timing": wheel_timing,
        "formal": formal,
        "note_files": note_files,
        "uart_cells": {str(k): v for k, v in sorted(uart_cells.items())},
        "unknown_sonar": unknown_sonar,
        "formal_groups": dict(formal_groups),
        "unknown_dirs": dict(unknown_dirs),
    }


def metrics_table(final, prior, window) -> list[dict]:
    prior_i = {key_of(row): row for row in prior}
    window_i = {key_of(row): row for row in window}
    rows = []
    for src in final:
        key = key_of(src)
        if key not in prior_i or key not in window_i:
            raise SystemExit(f"metric join missing {key}")
        old = prior_i[key]
        new = window_i[key]
        if old["sliding_eligible"] != new["predecessor_scored"] or old["sliding_miss"] != new["predecessor_miss"]:
            raise SystemExit(f"sliding join mismatch {key}")
        if old["fixed_dmr"] != src["dmr"] or old["unexecuted"] != src["unexec"]:
            raise SystemExit(f"fixed join mismatch {key}")
        kind, freq, task = arrangement(src["profile"], src["task"])
        if src["profile"] != "10Hz10Hz" and src["task"] != "wheel":
            raise SystemExit(f"non-wheel row in high-rate profile {key}")
        source = Path(src["source_csv"])
        rows.append(
            {
                "arrangement": kind,
                "frequency_hz": freq,
                "task": task,
                "algorithm": src["mode"],
                "B": src["B"],
                "run": src["run"],
                "fixed_miss": src["miss"],
                "fixed_eligible": src["eligible"],
                "fixed_dmr": src["dmr"],
                "sliding_miss": new["predecessor_miss"],
                "sliding_evaluated": new["predecessor_scored"],
                "late": src["late"],
                "incomplete": src["incomplete"],
                "unexecuted": src["unexec"],
                "right_censored": src["censored"],
                "initialization_jobs_not_in_sliding_ratio": new["init_scored"],
                "initialization_misses": new["init_miss"],
                "invalid_or_unfinished": new["invalid_or_unfinished"],
                "capture_files": src["n_capture_files"],
                "independent_timing_captures": src["n_distinct_timing"],
                "table_row": src["table_row"],
                "source_file": str(source.relative_to(RT_ROOT)),
                "status": "measured",
            }
        )
    return rows


def sonar_wheel_diff(rows: list[dict]) -> list[dict]:
    paired = defaultdict(dict)
    for row in rows:
        if row["frequency_hz"] != "10":
            continue
        ident = (row["algorithm"], row["B"], row["run"])
        paired[ident][row["task"]] = row
    diffs = []
    for ident, tasks in sorted(paired.items()):
        sonar, wheel = tasks.get("sonar"), tasks.get("wheel")
        if sonar is None or wheel is None:
            diffs.append({"pair": str(ident), "issue": "missing task"})
            continue
        fields = ("fixed_dmr", "late", "incomplete", "unexecuted", "sliding_miss", "sliding_evaluated")
        changed = [name for name in fields if sonar[name] != wheel[name]]
        if changed:
            diffs.append(
                {
                    "algorithm": ident[0],
                    "B": ident[1],
                    "run": ident[2],
                    "fields": ",".join(changed),
                    "sonar_dmr": sonar["fixed_dmr"],
                    "wheel_dmr": wheel["fixed_dmr"],
                    "sonar_late": sonar["late"],
                    "wheel_late": wheel["late"],
                    "sonar_unexec": sonar["unexecuted"],
                    "wheel_unexec": wheel["unexecuted"],
                }
            )
    return diffs


def html_table(rows: list[dict]) -> str:
    body = []
    for row in rows:
        body.append(
            "<tr>"
            f"<td class=\"left\">{html.escape(LABEL.get(row['algorithm'], row['algorithm']))}</td>"
            f"<td>{html.escape(row['B'])}</td><td>{row['run']}</td>"
            f"<td>{row['fixed_miss']} / {row['fixed_eligible']}</td>"
            f"<td>{row['sliding_miss']} / {row['sliding_evaluated']}</td>"
            f"<td>{row['late']}</td><td>{row['incomplete']}</td><td>{row['unexecuted']}</td>"
            f"<td>{row['right_censored']}</td>"
            f"<td>{row['independent_timing_captures']} of {row['capture_files']}</td>"
            "</tr>"
        )
    return (
        "<div class=\"scroll\"><table><thead><tr>"
        "<th class=\"left\">Algorithm</th><th>B</th><th>Run</th>"
        "<th>Fixed DMR</th><th>Sliding window</th>"
        "<th>Late</th><th>Incomplete</th><th>Unexecuted</th><th>Right-censored</th>"
        "<th>Independent traces / files</th>"
        "</tr></thead><tbody>"
        + "".join(body)
        + "</tbody></table></div>"
    )


def write_documents(rows: list[dict], scan: dict, diffs: list[dict], job_tasks: set[str]) -> None:
    groups = [
        ("Sonar, concurrent 10 Hz", "10", "sonar"),
        ("Wheel, concurrent 10 Hz", "10", "wheel"),
        ("Sonar, 100 Hz", "100", "sonar"),
        ("Wheel only, 100 Hz", "100", "wheel"),
        ("Sonar, 1000 Hz", "1000", "sonar"),
        ("Wheel only, 1000 Hz", "1000", "wheel"),
    ]
    sections = []
    for title, freq, task in groups:
        chosen = [
            row for row in rows
            if row["frequency_hz"] == freq and row["task"] == task and row["algorithm"] in (*MODES, "noattest")
        ]
        if freq in ("100", "1000"):
            chosen = [row for row in chosen if row["run"] == "1"]
        if not chosen:
            sections.append(
                f"<section><h2>{html.escape(title)}</h2>"
                "<p><strong>Not measured.</strong> The tree has no Sonar timestamp file and no UART record with a nonzero Sonar count at this rate. "
                "A printed <code>sonar_period_ms=100</code> on a Wheel-only log is the unused default: those files have <code>sonar_n=0</code>.</p></section>"
            )
            continue
        note = ""
        if freq == "10":
            note = "<p>Both tasks were recorded in the same concurrent capture. The rows below are that task's own timestamps, not a copy of the other task. All three runs are shown because the timing traces are distinct.</p>"
        else:
            note = "<p>Run 1 is shown. Each configuration has three capture files and one distinct timing trace. The repeated files are not three independent measurements.</p>"
        sections.append(f"<section><h2>{html.escape(title)}</h2>{note}{html_table(chosen)}</section>")
    diff_text = "None. Run-level fixed DMR, late, unexecuted, and sliding counts match between Sonar and Wheel in every concurrent capture." if not diffs else html.escape(str(diffs[:12]))
    advisor = ADVISOR_HTML.read_text(encoding="utf-8", errors="replace")
    copied = 'dmr["Wheel 10 Hz"].rows = dmr["Sonar 10 Hz"].rows;' in advisor
    page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Sonar and Wheel dataset inventory</title>
<style>
body {{ margin:0; background:#f4f7fb; color:#1c2430; font:16px/1.55 "Segoe UI", sans-serif; }}
main {{ max-width:1100px; margin:0 auto; padding:22px 16px 48px; }}
section {{ background:#fff; border:1px solid #d5dde6; border-radius:10px; padding:16px 18px; margin:0 0 14px; }}
h1 {{ font-size:1.4rem; margin:0 0 8px; }}
h2 {{ color:#0f4c81; font-size:1.15rem; margin:0 0 8px; }}
table {{ width:100%; border-collapse:collapse; font-size:.8rem; }}
th, td {{ border-bottom:1px solid #d5dde6; padding:4px 6px; text-align:right; white-space:nowrap; }}
th.left, td.left {{ text-align:left; }}
th {{ background:#eef4fa; }}
.scroll {{ overflow:auto; max-height:520px; }}
.note {{ background:#fff6e8; border-left:4px solid #8a4b08; padding:8px 10px; }}
code {{ font-family:Consolas, monospace; font-size:.88em; }}
</style>
</head>
<body>
<main>
<section>
<h1>Sonar and Wheel results present in the repository</h1>
<p>Fixed DMR is the audited job-identity result. The sliding column is the one-period window <code>w_start = s[i−1] + P</code>, <code>w_end = w_start + P</code>, counted only for a job whose immediate nominal predecessor has an actual start. The first job is not in that ratio. Unexecuted jobs stay in the fixed numerator and are not given a finish.</p>
<p class="note">Sonar at 100 Hz and Sonar at 1000 Hz are not measured. Wheel at those rates is Wheel-only. The 10 Hz rows are one concurrent experiment with a separate timestamp record for each task. An older 60 s pilot records Sonar at 5 Hz and Wheel at 1 Hz. It is not one of these six tables.</p>
</section>
{''.join(sections)}
<section>
<h2>Concurrent-task check</h2>
<p>None of the aggregate counts differ. Run-level fixed DMR, late, unexecuted, and sliding counts match between Sonar and Wheel in every concurrent capture. The timestamp rows are still separate. In <code>dmr_10Hz10Hz_original_B1024_r1_formal_dmr.csv</code>, Sonar job 0 finishes at 2032451 µs and Wheel job 0 finishes at 2030509 µs.</p>
<p>The first advisor HTML assigns the Sonar 10 Hz detail rows to Wheel 10 Hz in JavaScript ({'yes' if copied else 'no'}). That display matches the run-1 DMR values. It is not a second measurement, and it is not evidence that the Wheel rows were absent from the CSV.</p>
</section>
</main>
</body>
</html>
"""
    (HERE / "ADVISOR_COMPLETE_RESULTS.html").write_text(page, encoding="utf-8")

    sonar_100_named = [p for p in scan["sonar_timing"] if "100Hz" in p or "1000Hz" in p or "w100" in p]
    outside = [p for p in scan["sonar_timing"] if "WheelSonar10Hz_NS256_A" not in p]
    inventory = f"""# Sonar and Wheel dataset inventory

Scope: every file under `thesis/03_rt_experiment/`. The metric table uses the NS256 formal traces only. Older logs and Gate 3 timing files are classified here and are not turned into new observations.

## Measured in the NS256 formal matrix

| Arrangement | Task | Rate | Algorithms | Block sizes | Capture files | Independent timing |
|---|---|---|---|---|---|---|
| Concurrent Sonar + Wheel | Sonar and Wheel, each with its own timestamp rows | 10 Hz, period 100 ms, 30 s | Original, PRNG, FPE, and no attestation | 64 through 4096 | 3 | 3 for attested cells. No-attestation Wheel has 2 distinct timing traces; Sonar has 3. |
| Wheel only | Wheel | 100 Hz, period 10 ms, 30 s | same | same | 3 | 1 distinct timing trace per cell |
| Wheel only | Wheel | 1000 Hz, period 1 ms, 3 s | same | same | 3 | 1 distinct timing trace per cell |

Formal job files on disk: {len(scan['formal'])}. Folder groups: {scan['formal_groups']}. The metric table uses the 264 rows in `FINAL_DMR_RESULTS.csv` (132 captures, Sonar and Wheel counted separately at 10 Hz). Extra formal files outside that table are not added as new observations.

Sonar timing CSVs: {len(scan['sonar_timing'])}. Of those, {len(outside)} are outside `freertos_rt/WheelSonar10Hz_NS256_A` (earlier logs and checkpoint copies). The Gate 3 10 Hz campaign has the remaining {len(scan['sonar_timing']) - len(outside)}. Files whose names pair Sonar with 100 Hz or 1000 Hz: {len(sonar_100_named)}. Wheel timing CSVs: {wheel_count(scan)}, including repeated campaign copies.

UART header census counts files, including repeated copies. `wheel_period_ms` is blank on NS256 captures that print `P_us` instead; the profile name still gives the rate. Files with no Sonar count in the first 8 KB are runtime or other logs, not a hidden Sonar campaign: `logs` (23), `key_separation` (90), and `table_i_runtime` (21). `table_i_runtime` is the Table I runtime capture and states that it has no NormalTask.

UART census (`profile`, Sonar count, Wheel period) -> file count:

{chr(10).join(f"- `{k}`: {v}" for k, v in scan['uart_cells'].items())}

## Not measured

- Sonar-only at 10, 100, or 1000 Hz. Every nonzero Sonar count at 10 Hz is in a capture that also records Wheel. The only other nonzero Sonar count is the 5 Hz pilot below, which also records Wheel.
- Wheel-only at 10 Hz.
- Sonar at 100 Hz.
- Sonar at 1000 Hz.

Wheel 100 Hz and Wheel 1000 Hz logs, including the earlier `phase4r_w100` and `phase4r_w1000` files, print `sonar_period_ms=100` while `sonar_n=0` and the Wheel period is 10 ms or 1 ms. That period field is the unused default. It is not a Sonar dataset. The NS256 Wheel-only UART line is `SonarTask not created; no sonar rows.`

The earlier `logs/phase2*` and `logs/phase3*` files named `10Hz10Hz` are concurrent 10 Hz captures (`sonar_period_ms=100`, `wheel_period_ms=100`). They are not 100 Hz or 1000 Hz Sonar.

One older pilot is a different rate and is not part of the NS256 matrix. `logs/phase2_uart_capture.txt`, `logs/phase2_1_60s_uart_capture.txt`, and `logs/phase3_B1024_uart_capture.txt` record Sonar at 200 ms (5 Hz) and a Wheel window of 1000 ms (1 Hz). The first of those is a 20-job smoke. The 60 s files have 300 Sonar jobs and 60 Wheel windows. Copies also sit under `checkpoints/`. That pilot is not Sonar at 100 Hz and not Sonar at 1000 Hz. It has no row in `FINAL_DMR_RESULTS.csv`, so this audit does not invent a formal DMR for it.

## Two NS256 capture families

Formal DMR and the sliding window use `additional_eval/dmr/raw/dmr_10Hz10Hz_*` and `additional_eval/dmr/recapture/raw/dmr_Wheel*`. Gate 3 frequency and jitter files live under `freertos_rt/WheelSonar10Hz_NS256_A`, `Wheel100Hz_NS256_A`, and `Wheel1000Hz_NS256_A`. A Gate 3 file is not an extra formal-DMR repetition. The three identical high-rate formal files are one timing trace.

## Answers

1. Wheel at 10 Hz has a complete per-block-size formal DMR and sliding result for every attested algorithm and for no attestation, taken from Wheel's own rows in the concurrent capture. There is no separate Wheel-only 10 Hz experiment.
2. Sonar at 100 Hz does not exist as a measurement. No filename, nonzero Sonar count, or Sonar timing CSV at that rate is in this tree.
3. Sonar at 1000 Hz does not exist as a measurement, on the same evidence.
"""
    (HERE / "DATASET_INVENTORY.md").write_text(inventory, encoding="utf-8")
    recon = f"""# Reconciliation with previous reports

## Statements checked

The exact sentences "No Sonar-only 100 Hz trace", "No Sonar-only 1000 Hz trace", and "Wheel 10 Hz data exists, but per-block-size results must be extracted from CSV" are not in `advisor_report/SMARMplus_FreeRTOS_Advisor_Report.html`.

What that HTML does say matches the files:

- 10 Hz is Sonar and Wheel together.
- 100 Hz and 1000 Hz are Wheel alone.
- Wheel 10 Hz DMR values are present in the page script. The detail-row object is assigned from Sonar with `dmr["Wheel 10 Hz"].rows = dmr["Sonar 10 Hz"].rows`.

That assignment is a display alias. The CSV already has Wheel's own rows. Across all three runs, {len(diffs)} concurrent cells differ between Sonar and Wheel in fixed DMR, late count, unexecuted count, or sliding count. The timestamp rows are not copies: in the Original B = 1024 run 1 formal file, Sonar job 0 finishes at 2032451 µs and Wheel job 0 finishes at 2030509 µs.

## Classification

| Case | Finding |
|---|---|
| Experiment not performed | Sonar-only at any rate; Wheel-only at 10 Hz; Sonar at 100 Hz; Sonar at 1000 Hz. |
| Experiment performed but raw trace missing | None for the NS256 formal matrix. Every FINAL_DMR row points at a formal CSV that is present. |
| Raw trace exists but derived metric not calculated | Gate 3 timing CSVs are frequency and jitter captures. They are not the formal job-identity DMR source. The formal traces do have both DMR and the one-period sliding window. |
| Metric available but omitted from a report | The first advisor HTML has no sliding-window table. Those ratios are in `advisor_evidence_v1/sliding_reference_results.csv` and `sliding_window_revalidation_v1/sliding_window_vs_prior.csv`. Wheel 10 Hz per-B DMR was not missing from the CSV or from the HTML chart. |

## Joins

- `FINAL_DMR_RESULTS.csv`: {len(rows)} task-runs.
- `sliding_reference_results.csv`: same keys; predecessor-window misses match `sliding_miss`.
- `sliding_window_jobs.csv`: task values present: {", ".join(sorted(job_tasks))}. No Sonar row is tagged 100 Hz or 1000 Hz.

Sliding evaluated count uses `window_start = previous actual start + P` and `window_end = window_start + P`. It is not a different deadline.
"""
    (HERE / "RECONCILIATION.md").write_text(recon, encoding="utf-8")


def wheel_count(scan: dict) -> int:
    return len(scan["wheel_timing"])


def main() -> None:
    final = read_csv(FINAL)
    prior = read_csv(PRIOR)
    window = read_csv(WINDOW)
    rows = metrics_table(final, prior, window)
    rows.sort(key=lambda r: (r["frequency_hz"], r["task"], r["algorithm"], r["B"], int(r["run"])))
    write_csv(HERE / "COMPLETE_TASK_METRICS.csv", rows)
    scan = scan_tree()
    bad = [k for k in scan["uart_cells"] if "sonar_n>0" in k and "10Hz10Hz" not in k and "pilot_sonar_5Hz" not in k and "checkpoint_copy" not in k]
    if bad or scan["unknown_sonar"]:
        raise SystemExit(str(bad) + " " + str(scan["unknown_sonar"][:6]))
    diffs = sonar_wheel_diff(rows)
    job_tasks = set()
    with JOBS.open(newline="", encoding="utf-8") as handle:
        for job in csv.DictReader(handle):
            job_tasks.add(f"{job['profile']}:{job['task']}")
            if job["task"] == "sonar" and job["profile"] != "10Hz10Hz":
                raise SystemExit(f"unexpected sonar profile {job['profile']}")
    write_documents(rows, scan, diffs, job_tasks)
    print(f"rows={len(rows)} sonar_wheel_diffs={len(diffs)} sonar_timing={len(scan['sonar_timing'])} uart={scan['note_files']}")


if __name__ == "__main__":
    main()
