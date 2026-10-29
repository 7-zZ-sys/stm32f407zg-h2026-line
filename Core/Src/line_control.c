#include "line_control.h"
#include "line_config.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }
static float wrap(float x) { while (x > 180.0f) x -= 360.0f; while (x < -180.0f) x += 360.0f; return x; }

LineConfig LineConfig_Default(void)
{
  return (LineConfig){.kp=LINE_DEFAULT_KP, .kd=LINE_DEFAULT_KD,
    .test_speed=LINE_DEFAULT_TEST_SPEED, .test_min_speed=LINE_DEFAULT_TEST_MIN_SPEED,
    .lap_speed=LINE_DEFAULT_LAP_SPEED,
    .curve_speed=LINE_DEFAULT_CURVE_SPEED, .stop_offset_mm=LINE_DEFAULT_STOP_MM,
    .active_high=LINE_DEFAULT_ACTIVE_HIGH, .reverse=LINE_DEFAULT_REVERSE,
    .marker_count=LINE_DEFAULT_MARK_COUNT, .armed=false};
}

LineSample Line_Decode(uint8_t raw, const LineConfig *cfg)
{
  LineSample s = {.raw=raw};
  int sum = 0;
  unsigned first=8U, last=0U;
  bool previous = false;
  for (unsigned i=0; i<8; ++i) {
    unsigned physical = cfg->reverse ? 7U-i : i;
    bool hit = (((unsigned)raw >> physical) & 1U) == cfg->active_high;
    if (hit) {
      if (first==8U) first=i;
      last=i;
      s.mask |= (uint8_t)(1U << i);
      ++s.count;
      sum += (int)(2U*i)-7;
      if (!previous) ++s.groups;
    }
    previous = hit;
  }
  unsigned span=s.count ? last-first+1U : 0U;
  bool single_gap=s.groups==2U && span<=LINE_SINGLE_GAP_MAX_SPAN && span==(unsigned)s.count+1U;
  s.valid = s.count > 0U && (s.groups == 1U || single_gap);
  s.error = s.count ? (float)sum / (7.0f*(float)s.count) : 0.0f;
  /* A tolerated hole can steer, but must never synthesize a start/finish stripe. */
  s.marker = s.groups==1U && s.count >= cfg->marker_count && (s.mask & 0x18U) == 0x18U;
  return s;
}

void Line_Init(LineController *c) { memset(c, 0, sizeof(*c)); c->state=LINE_IDLE; }
bool Line_Active(const LineController *c) { return c->state==LINE_RUNNING || c->state==LINE_STOPPING; }

bool Line_Start(LineController *c, const LineConfig *cfg, const LineInput *in, bool lap)
{
  if (Line_Active(c)) return false;
  Line_Init(c);
  if (!cfg->armed) c->fault=LINE_NOT_ARMED;
  else if (!in->imu_ready) c->fault=LINE_IMU_FAULT;
  else if (!in->sample.valid || in->sample.count==8U) c->fault=LINE_NO_LINE;
  if (c->fault != LINE_OK) { c->state=LINE_FAULT; return false; }
  c->state=LINE_RUNNING;
  c->lap=lap;
  c->start_ms=c->last_ms=c->progress_ms=in->now_ms;
  c->origin_mm=in->distance_mm;
  c->last_yaw=in->yaw_deg;
  c->last_error=in->sample.error;
  c->speed_target_mm_s=lap ? cfg->lap_speed : cfg->test_speed;
  c->speed_reduced_ms=in->now_ms;
  c->narrow_seen=!in->sample.marker;
  return true;
}

void Line_Abort(LineController *c, uint32_t now, LineFault fault)
{
  if (Line_Active(c)) c->elapsed_ms=(uint32_t)(now-c->start_ms);
  c->fault=fault;
  c->state=(fault==LINE_USER_STOP) ? LINE_IDLE : LINE_FAULT;
  c->speed_reason=LINE_SPEED_STOP;
}

LineOutput Line_Step(LineController *c, const LineConfig *cfg, const LineInput *in)
{
  LineOutput out={0};
  if (!Line_Active(c)) return out;
  uint32_t delta=(uint32_t)(in->now_ms-c->last_ms);
  c->elapsed_ms=(uint32_t)(in->now_ms-c->start_ms);
  c->travel_mm=in->distance_mm-c->origin_mm;
  c->turned_deg+=wrap(in->yaw_deg-c->last_yaw);
  c->last_yaw=in->yaw_deg;
  c->last_ms=in->now_ms;
  if (!in->imu_ready) Line_Abort(c,in->now_ms,LINE_IMU_FAULT);
  else if (delta > 30U) Line_Abort(c,in->now_ms,LINE_TIMING_FAULT);
  else if (c->lap && c->elapsed_ms > LINE_LAP_TIMEOUT_MS) Line_Abort(c,in->now_ms,LINE_OVERTIME);
  else if (c->travel_mm < -50.0f || (c->lap && c->travel_mm > LINE_MAX_LAP_DISTANCE_MM))
    Line_Abort(c,in->now_ms,LINE_DISTANCE_FAULT);
  if (!Line_Active(c)) { out.hard_stop=true; return out; }

  if (c->state==LINE_STOPPING) {
    c->speed_reason=LINE_SPEED_STOP;
    out.hard_stop=true;
    if (in->max_wheel_speed_mm_s < 20.0f) {
      if (!c->still_pending) { c->still_pending=true; c->still_ms=in->now_ms; }
      if ((uint32_t)(in->now_ms-c->still_ms)>=100U) c->state=LINE_DONE;
    } else c->still_pending=false;
    if (c->state!=LINE_DONE && (uint32_t)(in->now_ms-c->brake_ms)>LINE_BRAKE_TIMEOUT_MS)
      Line_Abort(c,in->now_ms,LINE_BRAKE_FAULT);
    return out;
  }

  if (c->travel_mm-c->progress_mm >= 10.0f) {
    c->progress_mm=c->travel_mm; c->progress_ms=in->now_ms;
  } else if ((uint32_t)(in->now_ms-c->progress_ms)>LINE_STALL_TIMEOUT_MS) {
    Line_Abort(c,in->now_ms,LINE_STALLED); out.hard_stop=true; return out;
  }

  if (!in->sample.valid) {
    if (!c->lost) { c->lost=true; c->lost_ms=in->now_ms; ++c->invalid_events; }
    c->last_invalid_mask=in->sample.mask;
    c->speed_reason=LINE_SPEED_LOST;
    c->marker_pending=false;
    if ((uint32_t)(in->now_ms-c->lost_ms)>=LINE_LOST_TIMEOUT_MS) {
      Line_Abort(c,in->now_ms,LINE_LOST); out.hard_stop=true;
    } else if (in->sample.mask!=0U &&
               (uint32_t)(in->now_ms-c->lost_ms)<LINE_AMBIGUOUS_HOLD_MS) {
      /* A brief split pattern is not a new line position. Hold the last
       * reliable steering, without accelerating or fabricating a marker.
       * The original loss timer and raw diagnostic event remain active. */
      float floor=c->lap ? fminf(cfg->lap_speed,cfg->curve_speed) :
        fminf(cfg->test_speed,cfg->test_min_speed);
      out.speed_mm_s=fminf(c->speed_target_mm_s,fminf(floor,LINE_AMBIGUOUS_MAX_SPEED));
      out.angular_centirad_s=c->last_angular_centirad_s;
      c->speed_target_mm_s=out.speed_mm_s;
      c->speed_reduced_ms=in->now_ms;
      c->speed_reason=LINE_SPEED_SAMPLE_HOLD;
    } else {
      out.speed_mm_s=60.0f;
      c->speed_target_mm_s=out.speed_mm_s;
      c->speed_reduced_ms=in->now_ms;
      out.angular_centirad_s=(c->last_error>0.0f) ? -40.0f : (c->last_error<0.0f ? 40.0f : 0.0f);
    }
    return out;
  }
  bool reacquired=c->lost;
  c->lost=false;
  if (!in->sample.marker) c->narrow_seen=true;
  if (c->narrow_seen && c->travel_mm>=LINE_START_CLEAR_MM) c->left_start=true;
  bool return_gate=c->lap && c->left_start && c->travel_mm>=LINE_MIN_LAP_DISTANCE_MM &&
                   c->turned_deg<=-LINE_MIN_LAP_YAW_DEG;
  if (return_gate && in->sample.marker) {
    if (!c->marker_pending) {
      c->marker_pending=true; c->marker_ms=in->now_ms;
      c->marker_at_mm=c->travel_mm;
    }
    if ((uint32_t)(in->now_ms-c->marker_ms)>=LINE_MARK_DEBOUNCE_MS && c->stop_at_mm==0.0f)
      c->stop_at_mm=c->marker_at_mm+cfg->stop_offset_mm;
  } else c->marker_pending=false;

  if (c->stop_at_mm>0.0f && c->travel_mm>=c->stop_at_mm) {
    c->state=LINE_STOPPING; c->brake_ms=in->now_ms;
    c->speed_reason=LINE_SPEED_STOP;
    c->still_pending=false; out.hard_stop=true; return out;
  }

  float dt=(float)(delta ? delta : 10U)*0.001f;
  /* Only expected LAP stripes suppress steering; broad bends still use centroid. */
  bool expected_stripe=c->lap && in->sample.marker && (!c->left_start || return_gate);
  float error=expected_stripe ? c->last_error : in->sample.error;
  if (reacquired) { c->last_error=error; c->derivative=0.0f; }
  float d=(error-c->last_error)/dt;
  c->derivative+=0.25f*(d-c->derivative);
  c->last_error=error;
  out.angular_centirad_s=clamp(-cfg->kp*error-cfg->kd*c->derivative,
                              -LINE_MAX_ANGULAR,LINE_MAX_ANGULAR);
  float fast=c->lap ? cfg->lap_speed : cfg->test_speed;
  float slow=c->lap ? cfg->curve_speed : fminf(cfg->test_speed,cfg->test_min_speed);
  float curve=clamp(fabsf(error)*2.0f+fabsf(in->yaw_rate_dps)/80.0f,0.0f,1.0f);
  out.speed_mm_s=fast+(slow-fast)*curve;
  c->speed_reason=out.speed_mm_s<fast ? LINE_SPEED_CURVE : LINE_SPEED_NORMAL;
  if (c->lap && c->travel_mm>=LINE_APPROACH_DISTANCE_MM && out.speed_mm_s>LINE_APPROACH_SPEED) {
    out.speed_mm_s=fminf(out.speed_mm_s,LINE_APPROACH_SPEED);
    c->speed_reason=LINE_SPEED_APPROACH;
  }
  if (c->stop_at_mm>0.0f) {
    float remaining=fmaxf(0.0f,c->stop_at_mm-c->travel_mm);
    float limit=fmaxf(50.0f,sqrtf(2.0f*250.0f*remaining));
    if (out.speed_mm_s>limit) { out.speed_mm_s=limit; c->speed_reason=LINE_SPEED_OFFSET; }
  }
  /* Decelerate promptly; brief recentering must not accelerate in every bend. */
  if (out.speed_mm_s<=c->speed_target_mm_s) {
    c->speed_target_mm_s=out.speed_mm_s;
    c->speed_reduced_ms=in->now_ms;
  } else if ((uint32_t)(in->now_ms-c->speed_reduced_ms)>=LINE_SPEED_RECOVER_MS) {
    c->speed_target_mm_s=fminf(out.speed_mm_s,
        c->speed_target_mm_s+LINE_SPEED_RECOVER_ACCEL*dt);
  }
  if (c->speed_target_mm_s<out.speed_mm_s) c->speed_reason=LINE_SPEED_RECOVERY;
  out.speed_mm_s=c->speed_target_mm_s;
  c->last_angular_centirad_s=out.angular_centirad_s;
  return out;
}
