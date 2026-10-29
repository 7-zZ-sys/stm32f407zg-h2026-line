#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {int unused;} UART_HandleTypeDef;
typedef struct {volatile uint32_t BSRR;} GPIO_TypeDef;
typedef struct {volatile uint32_t CYCCNT;} TestDWT;
typedef enum {GPIO_PIN_RESET,GPIO_PIN_SET} GPIO_PinState;
extern GPIO_TypeDef test_gpio_b,test_gpio_e;
extern TestDWT test_dwt;
extern uint32_t SystemCoreClock,test_irq_mask;
#define GPIOB (&test_gpio_b)
#define GPIOE (&test_gpio_e)
#define DWT (&test_dwt)
#define GPIO_PIN_0 (1U<<0)
#define GPIO_PIN_1 (1U<<1)
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_14 (1U<<14)
#define GPIO_PIN_15 (1U<<15)
static inline uint32_t __get_PRIMASK(void){return test_irq_mask;}
static inline void __disable_irq(void){test_irq_mask=1;}
static inline void __enable_irq(void){test_irq_mask=0;}
static inline void __NOP(void){++test_dwt.CYCCNT;}
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin);
#endif
