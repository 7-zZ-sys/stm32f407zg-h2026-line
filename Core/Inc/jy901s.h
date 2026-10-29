#ifndef JY901S_H
#define JY901S_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* X forward, Y left, Z up: counter-clockwise/left turn is positive. */
#define JY901S_YAW_SIGN                 1.0f
#define JY901S_STALE_TIMEOUT_MS         50U
#define JY901S_CALIBRATION_SAMPLES      200U
#define JY901S_UART_BAUD                115200U
#define JY901S_FIRST_DATA_TIMEOUT_MS    500U
#define JY901S_CALIBRATION_TIMEOUT_MS   500U /* Short gaps restart validation; READY remains 50 ms. */

typedef enum {
  JY901S_DIAG_NONE, JY901S_DIAG_PROBE, JY901S_DIAG_CONFIG_READ,
  JY901S_DIAG_CONFIG_VERIFY, JY901S_DIAG_TX, JY901S_DIAG_UART,
  JY901S_DIAG_QUEUE, JY901S_DIAG_TRANSPORT,
  JY901S_DIAG_ANGLE_TIMEOUT, JY901S_DIAG_GYRO_TIMEOUT, JY901S_DIAG_PAIR_TIMEOUT
} JY901S_Diagnostic;

typedef enum {
  JY901S_OFFLINE = 0,
  JY901S_CALIBRATING = 1, /* Stationary validation; no host gyro integration. */
  JY901S_READY = 2,
  JY901S_FAULT = 3
} JY901S_Status;

typedef struct {
  JY901S_Status status;
  uint16_t calibration_samples;
  uint32_t last_sample_ms;
  uint32_t last_angle_ms;
  uint32_t last_gyro_ms;
  uint32_t error_count;
  JY901S_Diagnostic fault_reason; /* Last fault survives retries until READY. */
  uint32_t uart_error;
  float gyro_dps[3];
  float roll_deg;
  float pitch_deg;
  float raw_yaw_deg;
  float yaw_deg;
  float yaw_rate_dps;
} JY901S_Feedback;

/* Only imuTask calls Init/Poll. IRQs only enqueue DMA bytes/errors. */
HAL_StatusTypeDef JY901S_Init(void);
void JY901S_Poll(void);
void JY901S_GetFeedback(JY901S_Feedback *feedback);
bool JY901S_IsFresh(const JY901S_Feedback *feedback, uint32_t now_ms);
void JY901S_UartError(UART_HandleTypeDef *uart);

#endif
