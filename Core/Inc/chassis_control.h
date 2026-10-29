#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WHEEL_COUNT  4U

typedef struct
{
  int16_t target_mm_s[CHASSIS_WHEEL_COUNT];
  int16_t measured_mm_s[CHASSIS_WHEEL_COUNT];
  int16_t pwm[CHASSIS_WHEEL_COUNT];
  float heading_target_deg;
  float heading_measured_deg;
  float heading_error_deg;
  float heading_output_dps;
  bool heading_active;
} ChassisFeedback;

typedef enum {
  CHASSIS_STOP_NONE = 0,
  CHASSIS_STOP_USER,
  CHASSIS_STOP_IMU,
  CHASSIS_STOP_COMMAND_TIMEOUT,
  CHASSIS_STOP_LINE_FAULT,
  CHASSIS_STOP_CONTROL_TIMING,
  CHASSIS_STOP_HANDOVER,
  CHASSIS_STOP_FINISH
} ChassisStopReason;

HAL_StatusTypeDef Chassis_Init(void);
void Chassis_ControlStep(void);
void Chassis_SafetyStep(void);
void Chassis_EmergencyStop(void);
void Chassis_StopWithReason(ChassisStopReason reason);
ChassisStopReason Chassis_GetLastStopReason(void);
void Chassis_SetVelocity(int16_t linear_mm_s, int16_t angular_centirad_s);
/* 相对上电校准零点的目标航向；正值为左转。每500ms内刷新命令。 */
bool Chassis_IsStopped(void);
float Chassis_GetDistanceMm(void);
void Chassis_SetTrackingVelocity(int16_t linear_mm_s, int16_t angular_centirad_s);
void Chassis_SetHeading(int16_t linear_mm_s, float heading_deg);
void Chassis_GetFeedback(ChassisFeedback *feedback);
void Chassis_ControlPeriodElapsedCallback(void);

/* 串口在线调参：由communicationTask在任务上下文里调用，中断只负责收字节。 */
void Chassis_ProcessCommands(void);
/* 可选的非底盘命令分发；初始化阶段注册，true表示该命令已处理。 */
typedef bool (*ChassisAuxCommandHandler)(const char *command);
void Chassis_SetAuxCommandHandler(ChassisAuxCommandHandler handler);
/* 自保持开启时复现上一条指令，从而持续刷新500ms命令超时计时。 */
void Chassis_RefreshCommand(void);
/* 遥测周期（ms），由串口 TRATE= 在线设置；feedbackTask 每轮读它。
 * 默认 100（无线稳态）；精细波形分析用 20~30，但要插 USB（无线会丢字节）。 */
uint32_t Chassis_GetTelemetryPeriodMs(void);

/* 与原工程MotorControl()保持一致的命令入口。 */
void MotorControl(uint8_t angular_direction, uint16_t angular_speed,
                  uint8_t motion_direction, uint16_t motion_speed);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONTROL_H */
