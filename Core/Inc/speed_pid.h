#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPEED_PID_WHEEL_COUNT  4U

typedef struct
{
  float kp;
  float ki;
  float kd;
  float previous_error;
  float integral;
  float output;
  float output_limit;
} SpeedPID_Controller;

void SpeedPID_Init(void);
void SpeedPID_Reset(uint32_t wheel);
void SpeedPID_ResetAll(void);
void SpeedPID_SetGains(uint32_t wheel, float kp, float ki, float kd);
/* 优先读取尚待提交的请求，避免同一周期连续KP/KI/KD互相覆盖。 */
void SpeedPID_GetGains(uint32_t wheel, float *kp, float *ki, float *kd);
void SpeedPID_ApplyPendingGains(void);
/* FF与PID统一饱和判断；由控制任务独占计算与历史状态。 */
float SpeedPID_CalculateWithFeedforward(uint32_t wheel, float measured, float target,
                                        bool integral_enable, float feedforward);
float SpeedPID_Calculate(uint32_t wheel, float measured, float target,
                         bool integral_enable);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_PID_H */
