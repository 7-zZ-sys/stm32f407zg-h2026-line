#include "line_sensor.h"
#include "line_config.h"
#include "main.h"

static void settle_us(uint32_t microseconds)
{
  uint32_t start=DWT->CYCCNT;
  uint32_t cycles=(SystemCoreClock/1000000U)*microseconds;
  /* No interrupt masking: an IRQ can lengthen settling but never shorten it. */
  while ((uint32_t)(DWT->CYCCNT-start)<cycles) { __NOP(); }
}

uint8_t LineSensor_ReadRaw(void)
{
  uint8_t raw=0U;
  for (uint32_t channel=0; channel<8; ++channel) {
    uint32_t select=channel;
    uint32_t pins=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIOE->BSRR=((pins & ~select)<<16)|select;
    settle_us(LINE_MUX_SETTLE_US);
    if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==GPIO_PIN_SET) raw|=(uint8_t)(1U<<channel);
  }
  return raw;
}
