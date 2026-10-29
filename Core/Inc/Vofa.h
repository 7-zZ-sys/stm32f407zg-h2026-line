/*
 * MIT License
 * Copyright (c) 2021 Jelin
 */

#ifndef VOFA_H
#define VOFA_H

#define VOFA_BUFFER_SIZE  256U
#define VOFA_CMD_TAIL     {0xAF, 0xFA}

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t buffer[VOFA_BUFFER_SIZE];
  uint8_t *rp;
  uint8_t *wp;
  bool overflow;
} Vofa_FIFOTypeDef;

typedef enum
{
  VOFA_MODE_SKIP,
  VOFA_MODE_BLOCK_IF_FIFO_FULL
} Vofa_ModeTypeDef;

typedef struct
{
  uint8_t txBuffer[VOFA_BUFFER_SIZE];
  Vofa_FIFOTypeDef rxBuffer;
  Vofa_ModeTypeDef mode;
} Vofa_HandleTypedef;

void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode);

/* 串口端口层。VofaPort_Init() 必须在调度器启动前调用一次，
 * VofaPort_ReadByte() 供任务上下文的指令解析使用。 */
void VofaPort_Init(void);
uint8_t VofaPort_ReadByte(uint8_t *byte);
#define VOFA_PORT_COUNT 2U
#define VOFA_RX_EVENT_RESET 1U
#define VOFA_RX_EVENT_FLUSH 2U
/* 每个事件保留来源；RESET 表示丢字节，解析器必须作废当前行直到换行。 */
uint8_t VofaPort_ReadEvent(uint8_t *port, uint8_t *byte, uint8_t *flags);
void VofaPort_DiscardPendingInput(void);
typedef struct {
  uint32_t rx_overflows[VOFA_PORT_COUNT];
  uint32_t rx_errors[VOFA_PORT_COUNT];
  uint32_t tx_errors[VOFA_PORT_COUNT];
} VofaPortStats;
void VofaPort_GetStats(VofaPortStats *stats);

void Vofa_SendData(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t num);
void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num);
void Vofa_Printf(Vofa_HandleTypedef *handle, const char *format, ...);

void Vofa_ReceiveData(Vofa_HandleTypedef *handle);
uint16_t Vofa_ReadCmd(Vofa_HandleTypedef *handle, uint8_t *buffer,
                      uint16_t bufferLen);
uint16_t Vofa_ReadLine(Vofa_HandleTypedef *handle, uint8_t *buffer,
                       uint16_t bufferLen);
uint16_t Vofa_ReadData(Vofa_HandleTypedef *handle, uint8_t *buffer,
                       uint16_t bufferLen);

void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data,
                           uint16_t length);
uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
