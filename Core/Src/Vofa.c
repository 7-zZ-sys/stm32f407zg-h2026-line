/*
 * MIT License
 * Copyright (c) 2021 Jelin
 */

#include "Vofa.h"

#include <stdarg.h>
#include <stdio.h>

static const uint8_t cmdTail[] = VOFA_CMD_TAIL;
static const uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f};

void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode)
{
  if (handle == NULL)
  {
    return;
  }

  handle->rxBuffer.rp = handle->rxBuffer.buffer;
  handle->rxBuffer.wp = handle->rxBuffer.buffer;
  handle->rxBuffer.overflow = false;
  handle->mode = mode;
}

void Vofa_SendData(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t num)
{
  if ((handle != NULL) && (data != NULL) && (num > 0U))
  {
    Vofa_SendDataCallBack(handle, data, num);
  }
}

void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num)
{
  if ((handle == NULL) || (data == NULL) || (num == 0U))
  {
    return;
  }

  Vofa_SendDataCallBack(handle, (uint8_t *)data,
                        (uint16_t)(num * sizeof(float)));
  Vofa_SendDataCallBack(handle, (uint8_t *)justFloatTail,
                        (uint16_t)sizeof(justFloatTail));
}

void Vofa_Printf(Vofa_HandleTypedef *handle, const char *format, ...)
{
  int written;
  uint16_t length;
  va_list args;

  if ((handle == NULL) || (format == NULL))
  {
    return;
  }

  va_start(args, format);
  written = vsnprintf((char *)handle->txBuffer, VOFA_BUFFER_SIZE, format, args);
  va_end(args);

  if (written <= 0)
  {
    return;
  }

  /* vsnprintf返回完整输出长度；截断时只发送缓冲区中的有效内容。 */
  length = (written < (int)VOFA_BUFFER_SIZE) ? (uint16_t)written :
                                               (uint16_t)(VOFA_BUFFER_SIZE - 1U);
  Vofa_SendDataCallBack(handle, handle->txBuffer, length);
}

void Vofa_ReceiveData(Vofa_HandleTypedef *handle)
{
  uint8_t data;

  if (handle == NULL)
  {
    return;
  }

  data = Vofa_GetDataCallBack(handle);

  if (handle->rxBuffer.overflow &&
      (handle->mode == VOFA_MODE_BLOCK_IF_FIFO_FULL))
  {
    return;
  }

  *handle->rxBuffer.wp = data;
  handle->rxBuffer.wp++;

  if (handle->rxBuffer.wp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
  {
    handle->rxBuffer.wp = handle->rxBuffer.buffer;
  }
  if (handle->rxBuffer.wp == handle->rxBuffer.rp)
  {
    handle->rxBuffer.overflow = true;
  }
}

static uint8_t Vofa_GetByte(Vofa_HandleTypedef *handle, uint8_t *byte)
{
  if ((handle == NULL) || (byte == NULL))
  {
    return false;
  }

  if ((handle->rxBuffer.rp == handle->rxBuffer.wp) &&
      !handle->rxBuffer.overflow)
  {
    return false;
  }

  if (handle->rxBuffer.overflow)
  {
    handle->rxBuffer.overflow = false;
  }

  *byte = *handle->rxBuffer.rp;
  *handle->rxBuffer.rp = 0U;
  handle->rxBuffer.rp++;

  if (handle->rxBuffer.rp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
  {
    handle->rxBuffer.rp = handle->rxBuffer.buffer;
  }

  return true;
}

uint16_t Vofa_ReadCmd(Vofa_HandleTypedef *handle, uint8_t *buffer,
                      uint16_t bufferLen)
{
  uint16_t length = 0U;
  uint16_t tailCount = 0U;

  if ((handle == NULL) || (buffer == NULL))
  {
    return 0U;
  }

  for (uint16_t i = 0U;
       (i < bufferLen) && Vofa_GetByte(handle, &buffer[i]) &&
       (tailCount < sizeof(cmdTail));
       ++i)
  {
    if (buffer[i] == cmdTail[tailCount])
    {
      tailCount++;
    }
    else
    {
      tailCount = 0U;
    }
    length++;
  }

  return length;
}

uint16_t Vofa_ReadLine(Vofa_HandleTypedef *handle, uint8_t *buffer,
                       uint16_t bufferLen)
{
  uint16_t length = 0U;

  if ((handle == NULL) || (buffer == NULL))
  {
    return 0U;
  }

  for (uint16_t i = 0U;
       (i < bufferLen) && Vofa_GetByte(handle, &buffer[i]) &&
       (buffer[i] != '\n');
       ++i)
  {
    length++;
  }

  return length;
}

uint16_t Vofa_ReadData(Vofa_HandleTypedef *handle, uint8_t *buffer,
                       uint16_t bufferLen)
{
  uint16_t length = 0U;

  if ((handle == NULL) || (buffer == NULL))
  {
    return 0U;
  }

  for (uint16_t i = 0U;
       (i < bufferLen) && Vofa_GetByte(handle, &buffer[i]);
       ++i)
  {
    length++;
  }

  return length;
}

#ifdef __GNUC__
__attribute__((weak))
void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data,
                           uint16_t length)
{
  (void)handle;
  (void)data;
  (void)length;
}

__attribute__((weak))
uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
  (void)handle;
  return 0U;
}
#endif
