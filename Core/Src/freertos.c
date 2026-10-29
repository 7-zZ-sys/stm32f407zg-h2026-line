/* H2026 line-following application. The control task exclusively owns motion. */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "line_app.h"
#include "encoder.h"
#include "control_timing.h"
#include "runtime_diagnostics.h"
#include "jy901s.h"
#include "Vofa.h"

#define CONTROL_FLAG_ENCODER_SAMPLE (1UL << 0)
RuntimeDiagnostics g_runtime_diagnostics;
static Vofa_HandleTypedef s_vofa;
osThreadId_t controlTaskHandle, feedbackTaskHandle, imuTaskHandle;
static void StartControlTask(void *argument);
static void StartFeedbackTask(void *argument);
static void StartImuTask(void *argument);
static const osThreadAttr_t control_attributes={.name="controlTask",.stack_size=4096,.priority=osPriorityHigh};
static const osThreadAttr_t feedback_attributes={.name="feedbackTask",.stack_size=3072,.priority=osPriorityNormal};
static const osThreadAttr_t imu_attributes={.name="imuTask",.stack_size=2048,.priority=osPriorityAboveNormal};

void MX_FREERTOS_Init(void)
{
  Vofa_Init(&s_vofa,VOFA_MODE_SKIP);
  VofaPort_Init();
  controlTaskHandle=osThreadNew(StartControlTask,NULL,&control_attributes);
  feedbackTaskHandle=osThreadNew(StartFeedbackTask,NULL,&feedback_attributes);
  imuTaskHandle=osThreadNew(StartImuTask,NULL,&imu_attributes);
  if (!controlTaskHandle || !feedbackTaskHandle || !imuTaskHandle) Error_Handler();
}

static void StartControlTask(void *argument)
{
  (void)argument;
  ControlTiming timing={0};
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  if (Chassis_Init()!=HAL_OK) Error_Handler();
  LineApp_Init();
  for (;;) {
    uint32_t flags=osThreadFlagsWait(CONTROL_FLAG_ENCODER_SAMPLE,osFlagsWaitAny,osWaitForever);
    if ((flags & osFlagsError)!=0U) continue;
    uint32_t start=HAL_GetTick(), cycles=DWT->CYCCNT;
    ControlTimingResult result=ControlTiming_Update(&timing,g_runtime_diagnostics.control_events,start);
    if (result==CONTROL_TIMING_NONE) continue;
    g_runtime_diagnostics.max_interval_ms=timing.max_interval;
    g_runtime_diagnostics.missed_events=timing.missed_events;
    g_runtime_diagnostics.late_wakes=timing.late_wakes;
    if (result==CONTROL_TIMING_FAULT) {
      LineApp_Fault(LINE_TIMING_FAULT);
      Chassis_StopWithReason(CHASSIS_STOP_CONTROL_TIMING);
      Encoder_Reset();
    } else {
      LineApp_Step();
      Chassis_ControlStep();
      ++g_runtime_diagnostics.control_runs;
    }
    uint32_t us=(DWT->CYCCNT-cycles)/(SystemCoreClock/1000000U);
    if (us>g_runtime_diagnostics.max_execution_us) g_runtime_diagnostics.max_execution_us=us;
    uint32_t ms=HAL_GetTick()-start;
    if (ms>g_runtime_diagnostics.max_execution_ms) g_runtime_diagnostics.max_execution_ms=ms;
  }
}

static void StartFeedbackTask(void *argument)
{
  (void)argument;
  for (;;) {
    ChassisFeedback f;
    JY901S_Feedback imu;
    LineFeedback line;
    Chassis_GetFeedback(&f);
    JY901S_GetFeedback(&imu);
    LineApp_GetFeedback(&line);
    ++g_runtime_diagnostics.telemetry_sequence;
    g_runtime_diagnostics.telemetry_sample_ms=HAL_GetTick();
    g_runtime_diagnostics.stack_free_min[0]=osThreadGetStackSpace(controlTaskHandle);
    g_runtime_diagnostics.stack_free_min[3]=osThreadGetStackSpace(feedbackTaskHandle);
    g_runtime_diagnostics.stack_free_min[4]=osThreadGetStackSpace(imuTaskHandle);
    /* Channels 1..33 retain their order; 34..36 capture short invalid-line events. */
    Vofa_Printf(&s_vofa,
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%u,%u,%u,%lu,%u,%u,%u,%u,%ld,%ld,%lu,%ld,%u,%u,%ld,%u,%lu,%u,%lu,%u\r\n",
      f.target_mm_s[0],f.target_mm_s[1],f.target_mm_s[2],f.target_mm_s[3],
      f.measured_mm_s[0],f.measured_mm_s[1],f.measured_mm_s[2],f.measured_mm_s[3],
      f.pwm[0],f.pwm[1],f.pwm[2],f.pwm[3],
      (long)(f.heading_target_deg*100),(long)(f.heading_measured_deg*100),
      (long)(imu.yaw_rate_dps*100),(long)(f.heading_output_dps*100),
      (unsigned)imu.status,(unsigned)imu.calibration_samples,(unsigned)f.heading_active,(unsigned long)imu.error_count,
      (unsigned)line.sample.raw,(unsigned)line.sample.mask,(unsigned)line.controller.state,
      (unsigned)line.controller.fault,(long)(line.sample.error*1000),
      (long)line.controller.travel_mm,(unsigned long)line.controller.elapsed_ms,
      (long)(line.controller.turned_deg*100),(unsigned)line.config.armed,(unsigned)Chassis_IsStopped(),
      (long)line.output.speed_mm_s,(unsigned)imu.fault_reason,(unsigned long)imu.uart_error,
      (unsigned)line.controller.speed_reason,(unsigned long)line.controller.invalid_events,
      (unsigned)line.controller.last_invalid_mask);
    uint32_t delay=(Chassis_GetTelemetryPeriodMs()*osKernelGetTickFreq()+999U)/1000U;
    osDelay(delay ? delay : 1U);
  }
}

static void StartImuTask(void *argument)
{
  (void)argument;
  JY901S_Feedback imu;
  const uint32_t period=(osKernelGetTickFreq()+199U)/200U;
  for (;;) {
    if (JY901S_Init()==HAL_OK) {
      uint32_t next=osKernelGetTickCount();
      for (;;) {
        JY901S_Poll();
        JY901S_GetFeedback(&imu);
        if (imu.status==JY901S_FAULT) break;
        next+=period;
        if ((int32_t)(next-osKernelGetTickCount())<=0) next=osKernelGetTickCount()+period;
        (void)osDelayUntil(next);
      }
    }
    /* Control owner observes stale IMU and latches the stop on the next 10 ms step. */
    osDelay(osKernelGetTickFreq());
  }
}

void Chassis_ControlPeriodElapsedCallback(void)
{
  if (controlTaskHandle!=NULL) {
    ++g_runtime_diagnostics.control_events;
    (void)osThreadFlagsSet(controlTaskHandle,CONTROL_FLAG_ENCODER_SAMPLE);
  }
}
