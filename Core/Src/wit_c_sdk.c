/* NORMAL UART subset of the WITMOTION STM32 SDK; provenance is documented. */
#include "wit_c_sdk.h"
#include <stddef.h>
#include <string.h>

#define FRAME_SIZE 11U

static SerialWrite s_write;
static RegUpdateCb s_update;
static uint8_t s_frame[FRAME_SIZE];
static uint32_t s_count, s_read_reg;
int16_t sReg[REGSIZE];

int32_t WitInit(uint32_t protocol, uint8_t addr)
{
  (void)addr;
  if (protocol != WIT_PROTOCOL_NORMAL) return WIT_HAL_INVAL;
  s_count = 0U;
  s_read_reg = 0U;
  return WIT_HAL_OK;
}

int32_t WitSerialWriteRegister(SerialWrite write_func)
{
  if (write_func == NULL) return WIT_HAL_INVAL;
  s_write = write_func;
  return WIT_HAL_OK;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
  if (update_func == NULL) return WIT_HAL_INVAL;
  s_update = update_func;
  return WIT_HAL_OK;
}

static void CopeWitData(uint8_t type, const uint16_t values[4])
{
  uint32_t reg, count;
  switch (type)
  {
    case WIT_GYRO: reg = GX; count = 3U; break;
    case WIT_ANGLE: reg = Roll; count = 3U; break;
    case WIT_REGVALUE: reg = s_read_reg; count = 4U; break;
    default: return;
  }
  if (reg > REGSIZE - count) return;
  memcpy(&sReg[reg], values, count * sizeof(values[0]));
  s_update(reg, count);
}

void WitSerialDataIn(uint8_t data)
{
  uint8_t sum = 0U;
  uint16_t values[4];
  if (s_update == NULL) return;
  s_frame[s_count++] = data;
  if (s_frame[0] != 0x55U)
  {
    --s_count;
    memmove(s_frame, &s_frame[1], s_count);
    return;
  }
  if (s_count < FRAME_SIZE) return;
  for (uint32_t i = 0U; i < FRAME_SIZE - 1U; ++i)
    sum = (uint8_t)(sum + s_frame[i]);
  if (sum != s_frame[FRAME_SIZE - 1U])
  {
    --s_count;
    memmove(s_frame, &s_frame[1], s_count);
    return;
  }
  for (uint32_t i = 0U; i < 4U; ++i)
    values[i] = (uint16_t)((uint16_t)s_frame[2U + 2U * i] |
                          ((uint16_t)s_frame[3U + 2U * i] << 8));
  s_count = 0U;
  CopeWitData(s_frame[1], values);
}

int32_t WitWriteReg(uint32_t reg, uint16_t value)
{
  uint8_t command[5];
  if (reg >= REGSIZE) return WIT_HAL_INVAL;
  if (s_write == NULL) return WIT_HAL_EMPTY;
  command[0] = 0xffU;
  command[1] = 0xaaU;
  command[2] = (uint8_t)reg;
  command[3] = (uint8_t)value;
  command[4] = (uint8_t)(value >> 8);
  s_write(command, sizeof(command));
  return WIT_HAL_OK;
}

int32_t WitReadReg(uint32_t reg, uint32_t count)
{
  if (count != 4U || reg > REGSIZE - 4U) return WIT_HAL_INVAL;
  s_read_reg = reg;
  return WitWriteReg(READADDR, (uint16_t)reg);
}
