#ifndef RUNTIME_DIAGNOSTICS_H
#define RUNTIME_DIAGNOSTICS_H
#include <stdint.h>

/* 调试器实时观察，不往20通道FireWater中混入额外文本。时间单位ms。 */
typedef struct {
  volatile uint32_t control_events;
  volatile uint32_t control_runs;
  volatile uint32_t missed_events;
  volatile uint32_t late_wakes;
  volatile uint32_t max_interval_ms;
  volatile uint32_t max_execution_ms;
  volatile uint32_t max_execution_us; /* Cortex-M4 DWT周期计数，细粒度耗时。 */
  volatile uint32_t telemetry_sequence;
  volatile uint32_t telemetry_sample_ms;
  /* 顺序：control / communication / safety / feedback / imu，单位字节。 */
  volatile uint32_t stack_free_min[5];
} RuntimeDiagnostics;
extern RuntimeDiagnostics g_runtime_diagnostics;
#endif
