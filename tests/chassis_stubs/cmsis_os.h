#ifndef TEST_OS_H
#define TEST_OS_H
#include <stdint.h>
int32_t osDelay(uint32_t ticks);
uint32_t osKernelGetTickFreq(void);
#endif
