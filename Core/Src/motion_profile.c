#include "motion_profile.h"
#include <math.h>
#include <string.h>

static float Value(const float c[4], float s)
{ return ((c[3] * s + c[2]) * s + c[1]) * s + c[0]; }

static void Coefficients(float c[4], float value, float accel, float target, float t)
{
  float d = target - value;
  c[0] = value; c[1] = accel * t;
  c[2] = 3.0f * d - 2.0f * c[1];
  c[3] = -2.0f * d + c[1];
}

static bool AxisValid(const float c[4], float t, float a, float j)
{
  float peak = fmaxf(fabsf(c[1] / t), fabsf((c[1] + 2*c[2] + 3*c[3]) / t));
  if (fabsf(c[3]) > 1e-7f) {
    float s = -c[2] / (3.0f*c[3]);
    if (s > 0 && s < 1)
      peak = fmaxf(peak, fabsf((c[1] + 2*c[2]*s + 3*c[3]*s*s) / t));
  }
  float jerk = fmaxf(fabsf(2*c[2]), fabsf(2*c[2] + 6*c[3])) / (t*t);
  return peak <= a + 0.001f && jerk <= j + 0.001f;
}

static bool WheelValid(const float c[4], float limit)
{
  float peak = fmaxf(fabsf(Value(c, 0)), fabsf(Value(c, 1)));
  float a = 3*c[3], b = 2*c[2], d = c[1];
  if (fabsf(a) < 1e-7f) {
    if (fabsf(b) > 1e-7f) {
      float s = -d/b;
      if (s > 0 && s < 1) peak = fmaxf(peak, fabsf(Value(c, s)));
    }
  } else {
    float discriminant = b*b - 4*a*d;
    if (discriminant >= 0) {
      float root = sqrtf(discriminant);
      float s1 = (-b + root)/(2*a), s2 = (-b - root)/(2*a);
      if (s1 > 0 && s1 < 1) peak = fmaxf(peak, fabsf(Value(c, s1)));
      if (s2 > 0 && s2 < 1) peak = fmaxf(peak, fabsf(Value(c, s2)));
    }
  }
  return peak <= limit + 0.001f;
}

static bool Plan(MotionProfile *p, float linear, float diff, const MotionLimits *l)
{
  float scale = fabsf(linear) + fabsf(diff);
  if (scale > l->wheel_limit) { linear *= l->wheel_limit/scale; diff *= l->wheel_limit/scale; }
  float t = fmaxf(0.02f, l->response_s);
  if (fabsf(p->accel[0]) < 0.0001f && fabsf(p->accel[1]) < 0.0001f) {
    t = fmaxf(t, 1.5f*fabsf(linear - p->value[0])/l->linear_accel);
    t = fmaxf(t, sqrtf(6.0f*fabsf(linear - p->value[0])/l->linear_jerk));
    t = fmaxf(t, 1.5f*fabsf(diff - p->value[1])/l->diff_accel);
    t = fmaxf(t, sqrtf(6.0f*fabsf(diff - p->value[1])/l->diff_jerk));
    t *= 1.001f; /* 浮点峰值检查留出小余量。 */
  }
  for (unsigned attempt = 0; attempt < 128 && t <= 3600.0f; ++attempt, t *= 1.10f) {
    float c[2][4], wheel[4];
    Coefficients(c[0], p->value[0], p->accel[0], linear, t);
    Coefficients(c[1], p->value[1], p->accel[1], diff, t);
    if (!AxisValid(c[0], t, l->linear_accel, l->linear_jerk) ||
        !AxisValid(c[1], t, l->diff_accel, l->diff_jerk)) continue;
    bool valid = true;
    for (int sign = -1; sign <= 1; sign += 2) {
      for (unsigned i = 0; i < 4; ++i) wheel[i] = c[0][i] + sign*c[1][i];
      if (!WheelValid(wheel, l->wheel_limit)) valid = false;
    }
    if (!valid) continue;
    memcpy(p->coefficient, c, sizeof(c));
    p->duration = t; p->elapsed = 0;
    p->target[0] = linear; p->target[1] = diff;
    p->limits = *l; p->running = true;
    return true;
  }
  return false;
}

void MotionProfile_Reset(MotionProfile *p) { memset(p, 0, sizeof(*p)); }

void MotionProfile_Step(MotionProfile *p, float linear, float diff,
                         const MotionLimits *l, float dt)
{
  if (!isfinite(linear) || !isfinite(diff) || dt <= 0 || dt > 0.05f ||
      !isfinite(dt) || !isfinite(l->response_s) ||
      !isfinite(l->linear_accel) || !isfinite(l->linear_jerk) ||
      !isfinite(l->diff_accel) || !isfinite(l->diff_jerk) || !isfinite(l->wheel_limit) ||
      l->linear_accel <= 0 || l->linear_jerk <= 0 ||
      l->diff_accel <= 0 || l->diff_jerk <= 0 || l->wheel_limit <= 0) return;
  float scale = fabsf(linear) + fabsf(diff);
  if (scale > l->wheel_limit) { linear *= l->wheel_limit/scale; diff *= l->wheel_limit/scale; }
  bool changed = fabsf(linear - p->target[0]) > 0.01f ||
                 fabsf(diff - p->target[1]) > 0.01f ||
                 l->response_s != p->limits.response_s ||
                 l->linear_accel != p->limits.linear_accel ||
                 l->linear_jerk != p->limits.linear_jerk ||
                 l->diff_accel != p->limits.diff_accel || l->diff_jerk != p->limits.diff_jerk;
  if (changed) (void)Plan(p, linear, diff, l);
  if (!p->running) return;
  p->elapsed = fminf(p->duration, p->elapsed + dt);
  float s = p->elapsed/p->duration;
  for (unsigned axis = 0; axis < 2; ++axis) {
    const float *c = p->coefficient[axis];
    p->value[axis] = Value(c, s);
    p->accel[axis] = (c[1] + 2*c[2]*s + 3*c[3]*s*s)/p->duration;
    if (p->elapsed >= p->duration) { p->value[axis] = p->target[axis]; p->accel[axis] = 0; }
  }
  if (p->elapsed >= p->duration) p->running = false;
}
