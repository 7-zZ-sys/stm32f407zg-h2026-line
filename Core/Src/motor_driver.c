#include "motor_driver.h"

#include "main.h"
#include "tim.h"

static uint16_t MotorDriver_ClampDuty(int32_t value)
{
  if (value < 0)
  {
    value = -value;
  }

  if (value > MOTOR_PWM_LIMIT)
  {
    value = MOTOR_PWM_LIMIT;
  }

  return (uint16_t)value;
}

static void MotorDriver_SetA(int16_t pwm)
{
  uint16_t duty = MotorDriver_ClampDuty((int32_t)pwm);

  if (duty == 0U)
  {
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  }
  else if (pwm > 0)
  {
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}

static void MotorDriver_SetB(int16_t pwm)
{
  uint16_t duty = MotorDriver_ClampDuty((int32_t)pwm);

  if (duty == 0U)
  {
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
  }
  else if (pwm > 0)
  {
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}

static void MotorDriver_SetC(int16_t pwm)
{
  uint16_t duty = MotorDriver_ClampDuty((int32_t)pwm);

  if (duty == 0U)
  {
    HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_RESET);
  }
  else if (pwm > 0)
  {
    /* 保持原车 MotorC_Forward 的方向电平：IN1=0，IN2=1。 */
    HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_C_IN1_GPIO_Port, MOTOR_C_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_C_IN2_GPIO_Port, MOTOR_C_IN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
}

static void MotorDriver_SetD(int16_t pwm)
{
  uint16_t duty = MotorDriver_ClampDuty((int32_t)pwm);

  if (duty == 0U)
  {
    HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_RESET);
  }
  else if (pwm > 0)
  {
    /* 保持原车 MotorD_Forward 的方向电平：IN1=1，IN2=0。 */
    HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_D_IN1_GPIO_Port, MOTOR_D_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_D_IN2_GPIO_Port, MOTOR_D_IN2_Pin, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
}

HAL_StatusTypeDef MotorDriver_Init(void)
{
  MotorDriver_StopAll();

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void MotorDriver_StopAll(void)
{
  const int16_t stopped[MOTOR_COUNT] = {0, 0, 0, 0};
  MotorDriver_SetSignedPwm(stopped);
}

void MotorDriver_SetSignedPwm(const int16_t pwm[MOTOR_COUNT])
{
  if (pwm == NULL)
  {
    return;
  }

  MotorDriver_SetA(pwm[0]);
  MotorDriver_SetB(pwm[1]);
  MotorDriver_SetC(pwm[2]);
  MotorDriver_SetD(pwm[3]);
}
