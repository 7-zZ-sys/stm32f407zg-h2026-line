#ifndef HEADING_PID_H
#define HEADING_PID_H

#include <stdbool.h>

/* 外环单位：角度deg、角速度deg/s、时间s。与速度内环RPM参数不同。 */
#define HEADING_PID_DEFAULT_KP  1.5f
#define HEADING_PID_DEFAULT_KI  0.0f
#define HEADING_PID_DEFAULT_KD  0.04f
/*
 * 航向外环角速度输出限幅，单位 deg/s。
 * 注意它是**前馈和反馈共用**的额度：HeadingPID_Calculate 把
 * (前馈 + Kp误差 + Kd项 + Ki积分) 一起限幅到这个值。
 * 所以做"边转向边前行"的高速圆弧时，前馈占用越小、留给纠偏的余量越多。
 * 30 -> 60 的改动让 0.5 rad/s(28.65 dps) 的圆弧从只剩 1.35 dps 余量
 * 变成有 31.35 dps 余量，同时原地转向也快了一倍。
 * 高速时真正起作用的是 chassis_control.c 里按线速度算出的几何限幅
 * ((单轮上限 - |v|) x 2 / 轮距)，不会因为这里放大而失去保护。
 */
#define HEADING_MAX_RATE_DPS    60.0f

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_dps;
} HeadingPID;

float HeadingPID_WrapDegrees(float degrees);
void HeadingPID_Init(HeadingPID *pid);
void HeadingPID_Reset(HeadingPID *pid);
float HeadingPID_Calculate(HeadingPID *pid, float target_deg, float measured_deg,
                           float measured_rate_dps, float feedforward_dps,
                           float dt_s, float output_limit_dps);

#endif
