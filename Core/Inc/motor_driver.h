#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_COUNT       4U
/* TIM3的ARR，对应10 kHz PWM（Prescaler=0）。占空比量程0~8399。 */
#define MOTOR_PWM_LIMIT   8399

HAL_StatusTypeDef MotorDriver_Init(void);
void MotorDriver_StopAll(void);
void MotorDriver_SetSignedPwm(const int16_t pwm[MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
