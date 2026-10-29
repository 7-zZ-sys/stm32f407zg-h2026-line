#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H
#include <stdbool.h>

/* 两轴共享时间的三次速度轨迹；值的一阶导数为加速度、二阶为jerk。
 * 解析检查整段极值，不靠逐拍裁剪或到位吸附。
 * 改目标时保留当前值/加速度；若新轨迹不可行，沿旧轨迹继续并重试。
 * 限值在线降低只能在新轨迹可行时生效；急停必须另行Reset并撤销PWM。 */
typedef struct {
  float response_s, linear_accel, linear_jerk, diff_accel, diff_jerk, wheel_limit;
} MotionLimits;

typedef struct {
  float coefficient[2][4];
  float value[2], accel[2], target[2];
  float elapsed, duration;
  MotionLimits limits;
  bool running;
} MotionProfile;

void MotionProfile_Reset(MotionProfile *profile);
/* 两轴共用时间轴。三次Hermite轨迹保留初始加速度，解析检查全程约束。
 * 新目标暂不可行时继续原可行轨迹，下一拍重试，不硬切速度/加速度。
 */
void MotionProfile_Step(MotionProfile *profile, float linear, float diff,
                         const MotionLimits *limits, float dt);
#endif
