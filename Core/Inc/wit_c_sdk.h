/* WITMOTION STM32 SDK subset: NORMAL UART only. See WIT-SDK-PROVENANCE.md. */
#ifndef WIT_C_SDK_H
#define WIT_C_SDK_H

#include <stdint.h>
#include "REG.h"

#define WIT_HAL_OK          0
#define WIT_HAL_EMPTY      (-5)
#define WIT_HAL_INVAL      (-6)
#define WIT_PROTOCOL_NORMAL 0U

typedef void (*SerialWrite)(uint8_t *data, uint32_t length);
typedef void (*RegUpdateCb)(uint32_t reg, uint32_t count);

/* The NORMAL protocol has no address field; addr retains the SDK signature. */
int32_t WitInit(uint32_t protocol, uint8_t addr);
int32_t WitSerialWriteRegister(SerialWrite write_func);
int32_t WitRegisterCallBack(RegUpdateCb update_func);
void WitSerialDataIn(uint8_t data);
int32_t WitWriteReg(uint32_t reg, uint16_t value);
/* NORMAL reads always return a four-register block. */
int32_t WitReadReg(uint32_t reg, uint32_t count);

extern int16_t sReg[REGSIZE];

#endif
