#include "line_sensor.h"
#include "line_config.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>

GPIO_TypeDef test_gpio_b,test_gpio_e;
TestDWT test_dwt;
uint32_t SystemCoreClock=8000000,test_irq_mask;
static uint8_t expected=0xA5,channels;
static uint32_t last_read;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin)
{
  assert(port==GPIOE && pin==GPIO_PIN_3);
  unsigned channel=GPIOE->BSRR&7;
  assert(channel==channels);
  assert((uint32_t)(DWT->CYCCNT-last_read)>=LINE_MUX_SETTLE_US*(SystemCoreClock/1000000));
  assert(test_irq_mask==0);
  last_read=DWT->CYCCNT;++channels;
  return (expected&(1U<<channel))?GPIO_PIN_SET:GPIO_PIN_RESET;
}
int main(void)
{
  assert(LineSensor_ReadRaw()==0xA5 && channels==8);
  channels=0;expected=0x5A;
  DWT->CYCCNT=UINT32_MAX-200;last_read=DWT->CYCCNT;
  assert(LineSensor_ReadRaw()==0x5A && channels==8);
  puts("PASS: sensor address sequence, 50us settling, IRQ availability and DWT wrap");
  return 0;
}
