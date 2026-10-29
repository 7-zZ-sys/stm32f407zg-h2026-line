#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_EVENTSOURCE_UPDATE 1U
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event);
void Test_TIM_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t compare);
#define __HAL_TIM_SET_COMPARE(timer, channel, compare) Test_TIM_SetCompare(timer, channel, compare)
typedef struct { uint32_t BaudRate; } UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; uint32_t ErrorCode; } UART_HandleTypeDef;
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length);
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
#define __DMB() ((void)0)
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length, uint32_t timeout);
extern uint32_t test_irq_mask;
static inline uint32_t __get_PRIMASK(void) { return test_irq_mask; }
static inline void __disable_irq(void) { test_irq_mask = 1U; }
static inline void __enable_irq(void) { test_irq_mask = 0U; }
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
#endif
