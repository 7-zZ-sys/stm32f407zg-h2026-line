#ifndef CONTROL_TIMING_H
#define CONTROL_TIMING_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t consumed, last_start;
  uint32_t missed_events, late_wakes, max_interval;
  bool started;
} ControlTiming;
typedef enum { CONTROL_TIMING_NONE, CONTROL_TIMING_RUN, CONTROL_TIMING_FAULT } ControlTimingResult;

/* 单bit唤醒标志可能合并多拍，单独的累计事件计数才是判断依据。
 * uint32_t差值自然支持tick与累计计数回绕；15ms为10ms周期的迟到保护线。 */
static inline ControlTimingResult ControlTiming_Update(ControlTiming *s,
                                                       uint32_t produced, uint32_t now)
{
  uint32_t pending = produced - s->consumed;
  uint32_t interval = now - s->last_start;
  if (pending == 0U) return CONTROL_TIMING_NONE;
  bool late = s->started && interval > 15U;
  if (s->started && interval > s->max_interval) s->max_interval = interval;
  if (pending > 1U) s->missed_events += pending - 1U;
  if (late) ++s->late_wakes;
  s->consumed = produced;
  s->last_start = now;
  s->started = true;
  return (pending > 1U || late) ? CONTROL_TIMING_FAULT : CONTROL_TIMING_RUN;
}
#endif
