"""Wall30 metric helpers — David regularity jitter + frequency.

Startup policy (advisor campaign):
  - Preserve ALL raw records, including iteration 0.
  - Observed frequency uses every valid in-window start [T0, Tend).
  - PRIMARY jitter excludes the first in-window sample and the interval
    that involves that sample (i.e. pairs start at the 2nd in-window stamp).
  - Sensitivity jitter uses all consecutive in-window pairs (includes startup).

Formal DMR (floor/spill) remains QUARANTINED — do not cite for paper.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

P_US_DEFAULT = 100_000
NOMINAL_DEFAULT = 300

# Hard quarantine: floor/spill formal DMR must not run unless explicitly enabled.
ENABLE_PROVISIONAL_FLOOR_DMR = False


@dataclass
class JobStamp:
    s_exec: int
    finish: int
    window_open: Optional[int] = None  # Wheel only


@dataclass
class MappedJob:
    job_ord: int
    s_exec: int
    finish: int
    release_k: Optional[int]
    r_us: Optional[int]
    d_us: Optional[int]
    formal_miss: bool
    tag: str  # assigned | boundary | unmapped_overflow


@dataclass
class JitterStats:
    n_pairs: int
    mean_us: float
    sd_us: float
    p95_us: float
    max_us: float
    jitters_us: List[int] = field(default_factory=list)


@dataclass
class ScoreResult:
    n_starts: int
    n_nominal: int
    obs_hz: float
    freq_ratio: float
    n_startup_excluded: int  # 0 or 1 for primary path
    primary: JitterStats
    sensitivity_incl_startup: JitterStats
    n_assigned: int = 0
    n_missed_slots: int = 0
    n_formal_miss: int = 0
    dmr: Optional[float] = None
    # Back-compat aliases used by older callers/tests
    jitters_us: List[int] = field(default_factory=list)
    jitter_pct: List[float] = field(default_factory=list)
    mapped: List[MappedJob] = field(default_factory=list)
    missed_ks: List[int] = field(default_factory=list)


def in_window_jobs(
    jobs: Sequence[JobStamp], t0: int, tend: int
) -> List[JobStamp]:
    return [j for j in jobs if t0 <= j.s_exec < tend]


def consecutive_interval_jitters(
    stamps_us: Sequence[int], p_us: int = P_US_DEFAULT
) -> List[int]:
    out: List[int] = []
    for a, b in zip(stamps_us, stamps_us[1:]):
        out.append(abs((b - a) - p_us))
    return out


def percentile_nearest(xs: Sequence[float], p: float) -> float:
    """Inclusive nearest-rank percentile; p in [0, 100]."""
    if not xs:
        return 0.0
    s = sorted(xs)
    if len(s) == 1:
        return float(s[0])
    rank = p / 100.0 * (len(s) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(s) - 1)
    frac = rank - lo
    return float(s[lo] * (1.0 - frac) + s[hi] * frac)


def summarize_jitter_stats(jitters_us: Sequence[int]) -> JitterStats:
    if not jitters_us:
        return JitterStats(0, 0.0, 0.0, 0.0, 0.0, [])
    n = len(jitters_us)
    mean = sum(jitters_us) / n
    var = sum((x - mean) ** 2 for x in jitters_us) / n
    return JitterStats(
        n_pairs=n,
        mean_us=mean,
        sd_us=var**0.5,
        p95_us=percentile_nearest(jitters_us, 95.0),
        max_us=float(max(jitters_us)),
        jitters_us=list(jitters_us),
    )


def summarize_jitter(jitters_us: Sequence[int]) -> Tuple[float, float, int, int]:
    """Legacy (mean, sd, min, max) tuple."""
    if not jitters_us:
        return 0.0, 0.0, 0, 0
    st = summarize_jitter_stats(jitters_us)
    return st.mean_us, st.sd_us, min(jitters_us), int(st.max_us)


def score_regularity(
    jobs: Sequence[JobStamp],
    t0: int,
    tend: int,
    p_us: int = P_US_DEFAULT,
    n_nominal: int = NOMINAL_DEFAULT,
    *,
    exclude_startup_for_primary: bool = True,
) -> ScoreResult:
    """Frequency on all in-window starts; primary jitter drops startup sample."""
    in_obs = in_window_jobs(jobs, t0, tend)
    n_starts = len(in_obs)
    all_stamps = [j.s_exec for j in in_obs]
    sens = summarize_jitter_stats(consecutive_interval_jitters(all_stamps, p_us))

    n_excl = 0
    if exclude_startup_for_primary and len(in_obs) >= 1:
        # Drop first in-window sample => also drops the (j0,j1) interval.
        primary_stamps = all_stamps[1:]
        n_excl = 1
    else:
        primary_stamps = all_stamps
    primary = summarize_jitter_stats(
        consecutive_interval_jitters(primary_stamps, p_us)
    )

    dur_s = (tend - t0) / 1_000_000.0 if tend > t0 else 0.0
    obs_hz = n_starts / dur_s if dur_s > 0 else 0.0
    pct = [100.0 * j / p_us for j in primary.jitters_us]

    return ScoreResult(
        n_starts=n_starts,
        n_nominal=n_nominal,
        obs_hz=obs_hz,
        freq_ratio=n_starts / float(n_nominal) if n_nominal else 0.0,
        n_startup_excluded=n_excl if n_starts else 0,
        primary=primary,
        sensitivity_incl_startup=sens,
        dmr=None,
        jitters_us=list(primary.jitters_us),
        jitter_pct=pct,
    )


def map_releases(
    jobs: Sequence[JobStamp],
    t0: int,
    tend: int,
    p_us: int = P_US_DEFAULT,
    n_nominal: int = NOMINAL_DEFAULT,
    d_us: int = P_US_DEFAULT,
    *,
    allow_provisional_dmr: bool = False,
    exclude_startup_for_primary: bool = True,
) -> ScoreResult:
    """David jitter + optional quarantined floor/spill DMR (off by default)."""
    base = score_regularity(
        jobs,
        t0,
        tend,
        p_us=p_us,
        n_nominal=n_nominal,
        exclude_startup_for_primary=exclude_startup_for_primary,
    )
    if not (ENABLE_PROVISIONAL_FLOOR_DMR and allow_provisional_dmr):
        return base

    in_obs = in_window_jobs(jobs, t0, tend)
    slots: List[Optional[int]] = [None] * n_nominal
    mapped: List[MappedJob] = []

    for ord_i, j in enumerate(in_obs):
        k = (j.s_exec - t0) // p_us
        tag = "assigned"
        release_k: Optional[int] = None
        if k < 0 or k >= n_nominal:
            tag = "boundary"
        else:
            k_assign = k
            while k_assign < n_nominal and slots[k_assign] is not None:
                k_assign += 1
            if k_assign >= n_nominal:
                tag = "unmapped_overflow"
            else:
                slots[k_assign] = ord_i
                release_k = k_assign

        r_us = (t0 + release_k * p_us) if release_k is not None else None
        deadline = (r_us + d_us) if r_us is not None else None
        formal_miss = bool(deadline is not None and j.finish > deadline)
        mapped.append(
            MappedJob(
                job_ord=ord_i,
                s_exec=j.s_exec,
                finish=j.finish,
                release_k=release_k,
                r_us=r_us,
                d_us=deadline,
                formal_miss=formal_miss,
                tag=tag,
            )
        )

    missed_ks = [k for k, occ in enumerate(slots) if occ is None]
    assigned = [m for m in mapped if m.tag == "assigned"]
    n_formal = sum(1 for m in assigned if m.formal_miss)
    dmr = (n_formal / len(assigned)) if assigned else None

    base.n_assigned = len(assigned)
    base.n_missed_slots = len(missed_ks)
    base.n_formal_miss = n_formal
    base.dmr = dmr
    base.mapped = mapped
    base.missed_ks = missed_ks
    return base
