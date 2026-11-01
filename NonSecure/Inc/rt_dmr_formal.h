/**
 * Formal DMR instrumentation (author-approved Q1–Q5).
 * Enable with RT_DMR_FORMAL=1. Default 0 preserves frozen RT corpora builds.
 *
 * On-device ledger is 16 B/job (HR 3100×16 ≈ 50 KiB — fits 96 KiB NS RAM).
 * r_us/d_us derived at dump from the measured tick0 edge:
 *   r = tick0_tim2_us + (sched_wake_tick - tick0) * 1000.
 * tick0_tim2_us is the TIM2 time of that SysTick edge, not T0 and not a
 * later osDelayUntil wake.
 * Wheel window_open_us reconstructed: s_us - window_open_before_ms*1000.
 */
#ifndef RT_DMR_FORMAL_H
#define RT_DMR_FORMAL_H

#ifndef RT_DMR_FORMAL
#define RT_DMR_FORMAL  0
#endif

#ifndef RT_DMR_INJECT_DELAY_US
#define RT_DMR_INJECT_DELAY_US  0
#endif

#if RT_DMR_FORMAL

#define DMR_ST_ONTIME           0U
#define DMR_ST_LATE             1U
#define DMR_ST_INCOMPLETE_MISS  2U
#define DMR_ST_CENSORED         3U
#define DMR_ST_INSTR_LOST       4U

typedef struct __attribute__((packed)) {
  uint32_t s_us;                 /* actual CPU start / window-close actual */
  uint32_t f_us;                 /* finish; 0 if none */
  uint32_t sched_wake_tick;      /* nominal W */
  uint16_t window_open_before_ms;/* Wheel: ms before s_us; 0 Sonar */
  uint8_t  status;               /* DMR_ST_* */
  uint8_t  flags;                /* bit0 immediate_wake, bit1 short_window */
} dmr_job_rec_t;

_Static_assert(sizeof(dmr_job_rec_t) == 16, "dmr_job_rec_t must be 16 bytes");

static inline uint32_t dmr_tick_delta(uint32_t later, uint32_t earlier)
{
  return later - earlier;
}

/* origin_us is the TIM2 timestamp of the tick0 SysTick edge. */
static inline uint32_t dmr_map_release_us(uint32_t origin_us, uint32_t t0_tick,
                                          uint32_t W_tick)
{
  return origin_us + dmr_tick_delta(W_tick, t0_tick) * 1000U;
}

static inline uint32_t dmr_deadline_us(uint32_t r_us, uint32_t period_us)
{
  return r_us + period_us;
}

static inline int dmr_deadline_fully_observed(uint32_t d_us, uint32_t Tend_us)
{
  return (d_us <= Tend_us) ? 1 : 0;
}

static inline uint8_t dmr_classify(uint32_t d_us, uint32_t Tend_us,
                                   uint32_t f_us, int have_finish)
{
  if (!dmr_deadline_fully_observed(d_us, Tend_us)) {
    return DMR_ST_CENSORED;
  }
  if (!have_finish) {
    return DMR_ST_INCOMPLETE_MISS;
  }
  if ((int32_t)(f_us - d_us) > 0) {
    return DMR_ST_LATE;
  }
  return DMR_ST_ONTIME;
}

static inline uint16_t dmr_open_before_ms(uint32_t window_open_us, uint32_t s_us)
{
  if (window_open_us == 0U || (int32_t)(s_us - window_open_us) < 0) {
    return 0U;
  }
  uint32_t dus = s_us - window_open_us;
  uint32_t ms = (dus + 500U) / 1000U;
  return (ms > 65535U) ? 65535U : (uint16_t)ms;
}

static inline uint32_t dmr_window_open_us(uint32_t s_us, uint16_t before_ms)
{
  if (before_ms == 0U) {
    return 0U;
  }
  return s_us - (uint32_t)before_ms * 1000U;
}

#endif /* RT_DMR_FORMAL */
#endif /* RT_DMR_FORMAL_H */
