#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_WHEEL_COUNT  4U
#define ENCODER_COUNTS_PER_REVOLUTION  1320.0f

HAL_StatusTypeDef Encoder_Start(void);
void Encoder_Stop(void);
void Encoder_Reset(void);
void Encoder_Sample10ms(void);
void Encoder_GetTotalCounts(int64_t counts[ENCODER_WHEEL_COUNT]);
void Encoder_GetSpeedsRpm(float speed_rpm[ENCODER_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
