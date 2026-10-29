#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t raw, mask, count, groups;
  float error; /* -1 left, +1 right; use only when valid, including compact single gaps */
  bool valid, marker;
} LineSample;

typedef enum { LINE_IDLE, LINE_RUNNING, LINE_STOPPING, LINE_DONE, LINE_FAULT } LineState;
typedef enum {
  LINE_SPEED_NORMAL, LINE_SPEED_CURVE, LINE_SPEED_LOST,
  LINE_SPEED_RECOVERY, LINE_SPEED_APPROACH, LINE_SPEED_OFFSET, LINE_SPEED_STOP,
  LINE_SPEED_SAMPLE_HOLD
} LineSpeedReason;
typedef enum {
  LINE_OK, LINE_NOT_ARMED, LINE_IMU_FAULT, LINE_NO_LINE, LINE_LOST,
  LINE_TIMING_FAULT, LINE_OVERTIME, LINE_DISTANCE_FAULT, LINE_STALLED,
  LINE_BRAKE_FAULT, LINE_USER_STOP, LINE_DRIVE_FAULT
} LineFault;
typedef struct {
  float kp, kd, test_speed, test_min_speed, lap_speed, curve_speed, stop_offset_mm;
  uint8_t active_high, reverse, marker_count;
  bool armed;
} LineConfig;
typedef struct {
  uint32_t now_ms;
  float distance_mm, yaw_deg, yaw_rate_dps, max_wheel_speed_mm_s;
  bool imu_ready;
  LineSample sample;
} LineInput;
typedef struct { float speed_mm_s, angular_centirad_s; bool hard_stop; } LineOutput;
typedef struct {
  LineState state;
  LineFault fault;
  bool lap, left_start, narrow_seen, lost, marker_pending, still_pending;
  uint32_t start_ms, last_ms, lost_ms, marker_ms, brake_ms, still_ms, progress_ms, elapsed_ms;
  float origin_mm, travel_mm, last_yaw, turned_deg, last_error, derivative;
  float progress_mm, marker_at_mm, stop_at_mm;
  uint32_t speed_reduced_ms;
  float speed_target_mm_s;
  float last_angular_centirad_s;
  LineSpeedReason speed_reason;
  uint32_t invalid_events;
  uint8_t last_invalid_mask;
} LineController;

LineConfig LineConfig_Default(void);
LineSample Line_Decode(uint8_t raw, const LineConfig *config);
void Line_Init(LineController *controller);
bool Line_Active(const LineController *controller);
bool Line_Start(LineController *controller, const LineConfig *config, const LineInput *input, bool lap);
void Line_Abort(LineController *controller, uint32_t now, LineFault fault);
LineOutput Line_Step(LineController *controller, const LineConfig *config, const LineInput *input);
#endif
