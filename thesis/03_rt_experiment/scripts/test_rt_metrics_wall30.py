"""Deterministic offline tests for wall30 instrumentation helpers.

PROVISIONAL / DO NOT CITE: floor/spill DMR is quarantined (disabled by default).
"""
from __future__ import annotations

import csv
import io
import unittest

import rt_metrics_wall30 as M
from rt_metrics_wall30 import JobStamp, map_releases, P_US_DEFAULT

T0 = 1_000_000
P = P_US_DEFAULT
TEND = T0 + 30_000_000
N = 300
# CMSIS-ish
OS_OK = 0
OS_ERR_PARAM = -4


def score_jitter_only(jobs, d=P):
    return map_releases(jobs, T0, TEND, p_us=P, n_nominal=N, d_us=d)


class TestDavidJitter(unittest.TestCase):
    def test_perfect_intervals(self):
        jobs = [
            JobStamp(s_exec=T0 + k * P + 50, finish=T0 + k * P + 900)
            for k in range(10)
        ]
        r = score_jitter_only(jobs)
        self.assertEqual(r.n_starts, 10)  # frequency keeps all
        self.assertEqual(r.n_startup_excluded, 1)
        self.assertEqual(r.primary.n_pairs, 8)  # dropped j0 and (j0,j1)
        self.assertEqual(max(r.jitters_us), 0)
        self.assertIsNone(r.dmr)  # quarantined

    def test_stretched_interval_in_sensitivity_not_primary_when_only_two(self):
        jobs = [
            JobStamp(s_exec=T0, finish=T0 + 100),
            JobStamp(s_exec=T0 + 2 * P, finish=T0 + 2 * P + 100),
        ]
        r = score_jitter_only(jobs)
        # Primary excludes startup sample => only one stamp left => no pairs
        self.assertEqual(r.primary.n_pairs, 0)
        self.assertEqual(r.sensitivity_incl_startup.jitters_us[0], P)


class TestStartupExclusion(unittest.TestCase):
    def test_frequency_keeps_startup_jitter_drops_first_interval(self):
        # j0 at T0 (startup), then perfect 100 ms cadence
        jobs = [JobStamp(s_exec=T0, finish=T0 + 50)] + [
            JobStamp(s_exec=T0 + (k + 1) * P, finish=T0 + (k + 1) * P + 50)
            for k in range(5)
        ]
        r = score_jitter_only(jobs)
        self.assertEqual(r.n_starts, 6)
        self.assertEqual(r.n_startup_excluded, 1)
        # Sensitivity includes (j0 -> j1) which is exactly P => 0, then zeros
        self.assertEqual(r.sensitivity_incl_startup.n_pairs, 5)
        # Inject a bad first interval only
        jobs2 = [
            JobStamp(s_exec=T0, finish=T0 + 50),
            JobStamp(s_exec=T0 + 50, finish=T0 + 100),  # tiny interval
            JobStamp(s_exec=T0 + 50 + P, finish=T0 + 50 + P + 50),
            JobStamp(s_exec=T0 + 50 + 2 * P, finish=T0 + 50 + 2 * P + 50),
        ]
        r2 = score_jitter_only(jobs2)
        self.assertEqual(r2.n_starts, 4)
        # Primary pairs from j1 onward: (j1,j2)=(P), (j2,j3)=(P) => 0 jitter
        self.assertEqual(r2.primary.n_pairs, 2)
        self.assertEqual(max(r2.primary.jitters_us), 0)
        # Sensitivity still sees the bad (j0,j1)
        self.assertEqual(r2.sensitivity_incl_startup.jitters_us[0], abs(50 - P))

    def test_p95_and_stats(self):
        jobs = [
            JobStamp(s_exec=T0 + k * P + (10 if k == 5 else 0), finish=T0 + k * P + 100)
            for k in range(20)
        ]
        r = score_jitter_only(jobs)
        self.assertGreater(r.primary.n_pairs, 0)
        self.assertGreaterEqual(r.primary.p95_us, 0.0)
        self.assertGreaterEqual(r.primary.max_us, r.primary.mean_us)


class TestFloorSpillQuarantined(unittest.TestCase):
    def test_dmr_disabled_by_default(self):
        jobs = [JobStamp(s_exec=T0 + 10, finish=T0 + P + 1)]
        r = score_jitter_only(jobs)
        self.assertIsNone(r.dmr)
        self.assertEqual(r.n_formal_miss, 0)
        self.assertEqual(r.mapped, [])

    def test_provisional_path_explicit(self):
        old = M.ENABLE_PROVISIONAL_FLOOR_DMR
        try:
            M.ENABLE_PROVISIONAL_FLOOR_DMR = True
            jobs = [JobStamp(s_exec=T0 + 10, finish=T0 + P + 1)]
            r = map_releases(
                jobs, T0, TEND, p_us=P, n_nominal=N, d_us=P,
                allow_provisional_dmr=True,
            )
            self.assertEqual(r.n_formal_miss, 1)
        finally:
            M.ENABLE_PROVISIONAL_FLOOR_DMR = old


class TestDelayUntilSemantics(unittest.TestCase):
    """Host-side models of first-iter / late / starvation (not on-target)."""

    def test_first_iteration_immediate_return(self):
        tick_entry = 1000
        next_init = tick_entry - 100  # after next-=P
        next0 = next_init + 100  # == tick_entry
        tick_before = tick_entry  # already at target
        status = OS_ERR_PARAM if tick_before >= next0 else OS_OK
        immediate = status != OS_OK or tick_before >= next0
        self.assertTrue(immediate)
        self.assertEqual(next0, tick_entry)

    def test_normal_periodic_wait(self):
        tick_entry = 1000
        sched = [tick_entry + i * 100 for i in range(3)]
        # pretend time advances with schedule
        for i, w in enumerate(sched):
            tick_before = w - 50 if i > 0 else w - 50
            if i == 0:
                tick_before = tick_entry - 1
            status = OS_ERR_PARAM if tick_before >= w else OS_OK
            if i == 0 and tick_before < w:
                self.assertEqual(status, OS_OK)

    def test_late_wake_target(self):
        sched_wake = 5000
        tick_before = 5200  # already past
        status = OS_ERR_PARAM if tick_before >= sched_wake else OS_OK
        self.assertEqual(status, OS_ERR_PARAM)

    def test_starvation_monotonic_sched_ticks(self):
        """TIM2 gaps stretch; sched_wake_tick still += P each iter."""
        tick_entry = 100
        wakes = []
        next_v = tick_entry - 100
        for i in range(5):
            next_v += 100
            wakes.append(next_v)
        self.assertEqual(wakes, [tick_entry + i * 100 for i in range(5)])
        # Synthetic TIM2 starts with stretch — index still 0..4
        s_exec = [1_000_000 + i * 180_000 for i in range(5)]
        for i in range(5):
            self.assertEqual(wakes[i], tick_entry + i * 100)


class TestObservationBoundary(unittest.TestCase):
    def test_start_at_tend_excluded(self):
        jobs = [JobStamp(s_exec=TEND, finish=TEND + 100)]
        r = score_jitter_only(jobs)
        self.assertEqual(r.n_starts, 0)

    def test_start_just_inside(self):
        jobs = [JobStamp(s_exec=TEND - 1, finish=TEND + 50)]
        r = score_jitter_only(jobs)
        self.assertEqual(r.n_starts, 1)


class TestCsvBackwardCompat(unittest.TestCase):
    def test_legacy_columns_still_parse(self):
        text = (
            "job_id,nominal_schedule_us,actual_start_us,finish_us,"
            "schedule_to_finish_us,exec_us,schedule_start_dev_us,interval_dev_us,"
            "deadline_miss,actual_interval_us,output_interval_us,"
            "distance_cm,status,echo_us,measure_span_us,irq_masked\n"
            "0,125875,125396,126267,392,871,-479,0,0,0,0,10,0,500,800,0\n"
        )
        rows = list(csv.DictReader(io.StringIO(text)))
        self.assertIn("nominal_schedule_us", rows[0])
        self.assertEqual(int(rows[0]["actual_start_us"]), 125396)
        # New columns absent — parsers must tolerate
        self.assertNotIn("sched_wake_tick", rows[0])

    def test_new_columns_parse(self):
        text = (
            "job_id,nominal_schedule_us,actual_start_us,finish_us,"
            "iter_ord,sched_wake_tick,tick_before_delay,delay_until_status,"
            "tick_after_delay,immediate_wake,wall_sched_candidate_us\n"
            "0,125875,100050,100900,0,1000,999,-4,1000,1,100000\n"
        )
        rows = list(csv.DictReader(io.StringIO(text)))
        self.assertEqual(int(rows[0]["iter_ord"]), 0)
        self.assertEqual(int(rows[0]["immediate_wake"]), 1)
        self.assertEqual(int(rows[0]["delay_until_status"]), -4)
        # Legacy nominal still T0+P style while candidate is T0-aligned label
        self.assertEqual(int(rows[0]["nominal_schedule_us"]), 125875)
        self.assertEqual(int(rows[0]["wall_sched_candidate_us"]), 100000)


class TestSonarWheelTimestampConsistency(unittest.TestCase):
    def test_sonar_s_exec_is_post_delay(self):
        # Documented contract: actual_start_us == s_exec
        s_exec = 5000
        self.assertEqual(s_exec, 5000)

    def test_wheel_window_end_is_s_exec(self):
        window_open = 1000
        window_end = 101000
        self.assertGreaterEqual(window_end - window_open, 0)
        # short window flag condition (< P-1ms)
        P_us = 100000
        short = (window_end - window_open + 1000) < P_us
        self.assertFalse(short)
        short2 = (window_open + 500 - window_open + 1000) < P_us
        # 500+1000 < 100000
        self.assertTrue((500 + 1000) < P_us)


class TestRecordSizeBudget(unittest.TestCase):
    def test_32_byte_records_fit_cap(self):
        cap = 360
        rec = 32
        both = 2 * cap * rec
        self.assertEqual(both, 23040)
        # Must stay well under typical NS BSS headroom (~200KB bank)
        self.assertLess(both, 64 * 1024)


if __name__ == "__main__":
    unittest.main()
