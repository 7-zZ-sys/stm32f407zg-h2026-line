/* Adapted from local firmware/tests/stm32f407_jy901s/test_cascade.c.
 * Exercises production movement code with simulated JY901S UART/DMA frames. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "chassis_control.h"
#include "heading_pid.h"
#include "jy901s.h"
#include "speed_pid.h"
#include "usart.h"
#include "tim.h"
#include "motor_driver.h"
#include "encoder.h"
#include "Vofa.h"

uint32_t test_irq_mask;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim7;
static uint32_t now_ms;
/* Virtual motion in old MPU units (65.5 counts/(deg/s)); emitted as WIT frames. */
static int16_t gyro[3];
static bool data_ready = true, io_fail;
static float module_yaw, module_roll, module_pitch;
static uint16_t sensor_regs[0x100] = {[2]=0x0c, [3]=9, [4]=6, [0x24]=1};
static const uint32_t sensor_bauds[] = {0,4800,9600,19200,38400,57600,115200,230400};
static uint8_t *dma_bytes;
static uint16_t dma_size, dma_position;
static bool sensor_unlocked, reject_writes, rx_missing, gyro_enabled = true, angle_enabled = true;
static uint32_t save_count;
static unsigned drop_read_reg, drop_read_count;
static uint32_t stream_pause_ms;
static int16_t pwm_output[4];
static float speed_feedback[4];
static int64_t total_counts[4];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t osKernelGetTickFreq(void) { return 1000U; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{ (void)timer; return HAL_OK; }

static void FeedBytes(const uint8_t *bytes, unsigned length)
{
  if (dma_bytes == NULL || huart3.Init.BaudRate != sensor_bauds[sensor_regs[4]]) return;
  for (unsigned i = 0; i < length; ++i) {
    dma_bytes[dma_position++] = bytes[i];
    if (dma_position == dma_size/2 || dma_position == dma_size)
      HAL_UARTEx_RxEventCallback(&huart3, dma_position);
    if (dma_position == dma_size) dma_position = 0;
  }
  if (dma_position != 0) HAL_UARTEx_RxEventCallback(&huart3, dma_position);
}

static void Frame(uint8_t type, int16_t x, int16_t y, int16_t z, int16_t extra)
{
  uint8_t bytes[11] = {0x55, type};
  int16_t values[4] = {x,y,z,extra};
  unsigned sum = 0;
  for (unsigned i = 0; i < 4; ++i) {
    bytes[2+2*i] = (uint8_t)values[i];
    bytes[3+2*i] = (uint8_t)((uint16_t)values[i] >> 8);
  }
  for (unsigned i = 0; i < 10; ++i) sum += bytes[i];
  bytes[10] = (uint8_t)sum;
  FeedBytes(bytes, sizeof(bytes));
}

static void Stream(void)
{
  module_yaw = HeadingPID_WrapDegrees(module_yaw + (float)gyro[2]/65.5f*0.01f);
  if (!data_ready || io_fail || stream_pause_ms!=0U) return;
  if (gyro_enabled && (sensor_regs[2] & 4))
    Frame(0x52, (int16_t)((float)gyro[0]/65.5f*32768.0f/2000.0f),
          (int16_t)((float)gyro[1]/65.5f*32768.0f/2000.0f),
          (int16_t)((float)gyro[2]/65.5f*32768.0f/2000.0f), 0);
  if (angle_enabled && (sensor_regs[2] & 8))
    Frame(0x53, (int16_t)(module_roll*32768.0f/180.0f),
          (int16_t)(module_pitch*32768.0f/180.0f),
          (int16_t)(module_yaw*32768.0f/180.0f), 18150);
}

int32_t osDelay(uint32_t ticks)
{
  for (uint32_t i = 0; i < ticks; ++i) {
    ++now_ms;
    if (stream_pause_ms!=0U) --stream_pause_ms;
    if (now_ms % 10U == 0U && dma_bytes != NULL) Stream();
  }
  return 0;
}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart)
{ assert(uart == &huart3); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ assert(uart == &huart3); dma_bytes = NULL; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                            uint8_t *data, uint16_t length)
{
  assert(uart == &huart3); dma_bytes = data; dma_size = length; dma_position = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes,
                                   uint16_t length, uint32_t timeout)
{
  unsigned reg;
  uint16_t value;
  (void)timeout;
  assert(uart == &huart3 && length == 5 && bytes[0] == 0xff && bytes[1] == 0xaa);
  if (io_fail) return HAL_ERROR;
  if (rx_missing || uart->Init.BaudRate != sensor_bauds[sensor_regs[4]]) return HAL_OK;
  reg = bytes[2]; value = (uint16_t)((uint16_t)bytes[3] | ((uint16_t)bytes[4] << 8));
  if (reg == 0x27) {
    assert(value < 0xfc);
    if (value==drop_read_reg && drop_read_count!=0U) { --drop_read_count; return HAL_OK; }
    Frame(0x5f, (int16_t)sensor_regs[value], (int16_t)sensor_regs[value+1],
          (int16_t)sensor_regs[value+2], (int16_t)sensor_regs[value+3]);
  } else if (reg == 0x69) sensor_unlocked = (value == 0xb588);
  else if (sensor_unlocked && !reject_writes) {
    if (reg == 0) ++save_count;
    else sensor_regs[reg] = value;
  }
  return HAL_OK;
}

HAL_StatusTypeDef MotorDriver_Init(void) { memset(pwm_output, 0, sizeof(pwm_output)); return HAL_OK; }
void MotorDriver_StopAll(void) { memset(pwm_output, 0, sizeof(pwm_output)); }
void MotorDriver_SetSignedPwm(const int16_t pwm[4]) { memcpy(pwm_output, pwm, sizeof(pwm_output)); }
HAL_StatusTypeDef Encoder_Start(void) { return HAL_OK; }
void Encoder_Stop(void) {}
void Encoder_Sample10ms(void) {}
void Encoder_GetSpeedsRpm(float speed[4]) { memcpy(speed, speed_feedback, sizeof(speed_feedback)); }
void Encoder_GetTotalCounts(int64_t counts[4]) { memcpy(counts,total_counts,sizeof(total_counts)); }

/* 串口在线调参入口（真身在 vofa_port.c，会拖进 usart.c 和 HAL UART）。
 * 这里做成一个测试用的字节队列，让 TestCommandIdleTimeout() 能把指令"喂"进
 * Chassis_ProcessCommands()；不喂时队列为空，等价于原来那句"没有收到指令"。 */
static uint8_t rx_queue[512];
static uint8_t rx_ports[512], rx_flags[512];
static uint8_t rx_push_port, rx_push_flags;
static uint16_t rx_head;
static uint16_t rx_tail;

static void RxPush(const char *text)
{
  while (*text != '\0')
  {
    assert((uint16_t)(rx_head - rx_tail) < (uint16_t)sizeof(rx_queue));
    rx_queue[rx_head % (uint16_t)sizeof(rx_queue)] = (uint8_t)*text;
    rx_ports[rx_head % (uint16_t)sizeof(rx_queue)] = rx_push_port;
    rx_flags[rx_head % (uint16_t)sizeof(rx_queue)] = rx_push_flags;
    ++rx_head;
    ++text;
  }
}

static void RxClear(void) { rx_head = 0U; rx_tail = 0U; }
void VofaPort_DiscardPendingInput(void) { rx_tail = rx_head; }

uint8_t VofaPort_ReadByte(uint8_t *byte)
{
  if (rx_tail == rx_head) return 0U;
  *byte = rx_queue[rx_tail % (uint16_t)sizeof(rx_queue)];
  ++rx_tail;
  return 1U;
}

uint8_t VofaPort_ReadEvent(uint8_t *port, uint8_t *byte, uint8_t *flags)
{
  if (rx_tail == rx_head) return 0U;
  *port = rx_ports[rx_tail % sizeof(rx_queue)];
  *flags = rx_flags[rx_tail % sizeof(rx_queue)];
  return VofaPort_ReadByte(byte);
}

static JY901S_Feedback IMU(void)
{
  JY901S_Feedback imu;
  JY901S_GetFeedback(&imu);
  return imu;
}

static ChassisFeedback Chassis(void)
{
  ChassisFeedback feedback;
  Chassis_GetFeedback(&feedback);
  return feedback;
}

static void Sample(void) {
  (void)osDelay(10U);
  if (io_fail) JY901S_UartError(&huart3);
  JY901S_Poll();
}
static void AssertStopped(void)
{
  ChassisFeedback feedback = Chassis();
  for (unsigned wheel = 0; wheel < 4; ++wheel)
  {
    assert(pwm_output[wheel] == 0 && feedback.pwm[wheel] == 0);
    assert(feedback.target_mm_s[wheel] == 0);
  }
  assert(!feedback.heading_active);
}

/*
 * 新增的给定整形把"普通停车"做成约 0.4 s 的柔和减速（不再是瞬间切断），
 * 所以固定跑 30 拍不再足以归零。这里改成"跑到停稳为止"，并用拍数上限把
 * "停不下来 / 停得太慢"也变成一条断言——它检查的是行为，而不是某个斜坡速率。
 * 需要零延迟停车的路径（急停、命令超时、IMU 失效）不经过整形，不受影响。
 */
#define STOP_SETTLE_MAX_SAMPLES 80U

static bool ChassisStopped(void)
{
  ChassisFeedback feedback = Chassis();
  for (unsigned wheel = 0; wheel < 4; ++wheel)
  {
    if ((pwm_output[wheel] != 0) || (feedback.pwm[wheel] != 0) ||
        (feedback.target_mm_s[wheel] != 0))
    {
      return false;
    }
  }
  return !feedback.heading_active;
}

static void SettleToStop(void)
{
  unsigned samples = 0U;

  while ((samples < STOP_SETTLE_MAX_SAMPLES) && !ChassisStopped())
  {
    Sample();
    Chassis_ControlStep();
    ++samples;
  }
  assert(samples < STOP_SETTLE_MAX_SAMPLES); /* 没在限期内停稳 */
  AssertStopped();
}

static void Calibrate(void)
{
  gyro[0] = 0; gyro[1] = 0; gyro[2] = 0;
  data_ready = true; io_fail = false; gyro_enabled = angle_enabled = true;
  module_yaw = module_roll = module_pitch = 0;
  rx_missing = reject_writes = false;
  assert(JY901S_Init() == HAL_OK);
  for (unsigned sample = 0; sample < JY901S_CALIBRATION_SAMPLES; ++sample) Sample();
  assert(IMU().status == JY901S_READY);
}

/*
 * 推进 now_ms 指定的时长，但一路用 Sample() 以 10 ms 为步长轮询 IMU。
 *
 * 不能直接 now_ms += ms：不生成新的串口帧会触发JY901S超时保护，
 * 一旦 FAULT，Chassis_ControlStep() 开头的安全检查就会直接急停返回，
 * 后面的断言全部"假通过"，测试失去意义。
 * 实车上 IMU 由独立的 imuTask 轮询，不存在这个问题，纯粹是测试替身要照顾。
 */
static void Advance(uint32_t ms)
{
  for (uint32_t done = 0U; done < ms; done += 10U)
  {
    Sample();
  }
}

static void TestHeadingPID(void)
{
  HeadingPID pid;
  float output;
  HeadingPID_Init(&pid);
  assert(fabsf(HeadingPID_WrapDegrees(-358.0f) - 2.0f) < 0.001f);
  assert(fabsf(HeadingPID_WrapDegrees(358.0f) + 2.0f) < 0.001f);
  output = HeadingPID_Calculate(&pid, -179, 179, 0, 0, 0.02f, 90);
  assert(output > 0 && output < 10); /* 过±180度边界走2度而非358度。 */
  assert(HeadingPID_Calculate(&pid, 0, 10, 0, 0, 0.02f, 90) < 0);
  assert(HeadingPID_Calculate(&pid, 0, 0, 10, 0, 0.02f, 90) < 0);
  pid.ki = 1.0f;
  for (unsigned i = 0; i < 1000; ++i)
    assert(HeadingPID_Calculate(&pid, 90, 0, 0, 0, 0.02f, 20) == 20);
  assert(pid.integral_dps == 0); /* 饱和时不能积累积分。 */
  pid.integral_dps = 5;
  pid.kp = 0;
  (void)HeadingPID_Calculate(&pid, -10, 0, 0, 0, 0.02f, 20);
  assert(pid.integral_dps < 5);
  assert(HeadingPID_Calculate(&pid, NAN, 0, 0, 0, 0.02f, 20) == 0);
  assert(pid.integral_dps == 0);
  assert(HeadingPID_Calculate(&pid, 90, 0, 0, 0, 0.02f, 0) == 0);
}

static void TestSensor(void)
{
  JY901S_Feedback imu;
  /* Factory default baud/rate/mode and disabled output are handled by reads. */
  sensor_regs[2] = 0; sensor_regs[3] = 6; sensor_regs[4] = 2; sensor_regs[0x24] = 0;
  save_count = 0;
  Calibrate();
  assert(sensor_regs[2] == 0x0c && sensor_regs[3] == 9 && sensor_regs[4] == 6);
  assert(sensor_regs[0x24] == 1 && save_count == 1);
  Calibrate(); assert(save_count == 1); /* No flash write on unchanged boot. */
  drop_read_reg=0x24; drop_read_count=1;
  Calibrate(); assert(drop_read_count==0 && save_count==1);
  drop_read_reg=2; drop_read_count=1;
  Calibrate(); assert(drop_read_count==0 && save_count==1);
  assert(JY901S_Init()==HAL_OK);
  stream_pause_ms=100;
  for (unsigned i=0;i<230;++i) Sample();
  assert(IMU().status==JY901S_READY && IMU().fault_reason==JY901S_DIAG_NONE);
  drop_read_reg=0x24; drop_read_count=3;
  assert(JY901S_Init()==HAL_ERROR && IMU().fault_reason==JY901S_DIAG_CONFIG_READ);
  assert(JY901S_Init()==HAL_OK);
  assert(IMU().fault_reason==JY901S_DIAG_CONFIG_READ); /* Visible through retry/calibration. */
  for (unsigned i=0;i<210;++i) Sample();
  assert(IMU().status==JY901S_READY && IMU().fault_reason==JY901S_DIAG_NONE);
  huart3.ErrorCode=8U;
  JY901S_UartError(&huart3); JY901S_Poll();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_UART);
  assert(IMU().uart_error==8U);
  Calibrate();
  assert(JY901S_Init()==HAL_OK);
  data_ready=false;
  for (unsigned i=0;i<6;++i) Sample();
  assert(IMU().status==JY901S_CALIBRATING);
  for (unsigned i=0;i<45;++i) Sample();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_ANGLE_TIMEOUT);
  Calibrate();
  for (uint16_t baud_index = 1; baud_index <= 7; ++baud_index) {
    sensor_regs[4] = baud_index;
    Calibrate();
    assert(sensor_regs[4] == 6 && huart3.Init.BaudRate == 115200);
  }
  gyro[2] = 655;
  for (unsigned i = 0; i < 100; ++i) Sample();
  imu = IMU();
  assert(imu.yaw_deg > 9.8f && imu.yaw_deg < 10.2f);
  assert(imu.yaw_rate_dps > 9.9f && imu.yaw_rate_dps < 10.1f);
  assert(JY901S_IsFresh(&imu, now_ms));
  assert(!JY901S_IsFresh(&imu, now_ms+51));
  io_fail = true; Sample(); assert(IMU().status == JY901S_FAULT);
  Calibrate();
  gyro_enabled = false;
  for (unsigned i = 0; i < 6; ++i) Sample();
  assert(IMU().status == JY901S_FAULT); /* Angle alone must not refresh gyro. */
  assert(IMU().fault_reason==JY901S_DIAG_GYRO_TIMEOUT);
  Calibrate(); angle_enabled = false;
  for (unsigned i = 0; i < 6; ++i) Sample();
  assert(IMU().status == JY901S_FAULT);
  assert(IMU().fault_reason==JY901S_DIAG_ANGLE_TIMEOUT);
  Calibrate();
  gyro[2] = -655;
  for (unsigned i = 0; i < 100; ++i) Sample();
  assert(IMU().yaw_deg < -9.8f);
}

static void TestJY901SProtocol(void)
{
  uint32_t sample_ms;
  Calibrate();
  sample_ms = IMU().last_sample_ms;
  (void)osDelay(1U);
  Frame(0x52,0,0,0,0); /* A damaged/unsupported angle must not complete this pair. */
  Frame(0x51,0,0,2048,0);
  JY901S_Poll();
  assert(IMU().last_sample_ms == sample_ms);
  { /* Noise, checksum failure, signed values, DMA wrap and resynchronization. */
    uint8_t bad[13] = {0x23,0x55,0x53,0,0,0,0,0,0,0,0,0,0x11};
    FeedBytes(bad,sizeof(bad));
    JY901S_Poll();
    assert(IMU().last_sample_ms == sample_ms);
  }
  module_yaw = 90; /* A different module angle with zero gyro: no host integration. */
  Sample(); assert(fabsf(IMU().yaw_deg - 90) < 0.02f);
  module_yaw = 179; Sample();
  module_yaw = -179; Sample();
  assert(fabsf(IMU().yaw_deg + 179) < 0.02f);
  /* Only old queued bytes must never resurrect an IMU after a task stall. */
  Stream(); data_ready = false; (void)osDelay(80);
  JY901S_Poll(); assert(IMU().status == JY901S_FAULT);
  Calibrate();
  /* Overflow is detected; do not silently consume a truncated ring. */
  for (unsigned i=0;i<30;++i) Stream();
  JY901S_Poll(); assert(IMU().status == JY901S_FAULT);
  assert(IMU().fault_reason==JY901S_DIAG_QUEUE);
  Calibrate();
  rx_missing = true;
  assert(JY901S_Init() == HAL_ERROR); /* TX disconnected: stream alone is insufficient. */
  assert(IMU().fault_reason==JY901S_DIAG_PROBE);
  rx_missing = false;
  sensor_regs[0x24] = 0; reject_writes = true;
  assert(JY901S_Init() == HAL_ERROR); /* Reject config without matching readback. */
  assert(IMU().fault_reason==JY901S_DIAG_CONFIG_VERIFY);
  reject_writes = false;
  Calibrate();
  gyro[2] = 655;
  assert(JY901S_Init() == HAL_OK);
  for (unsigned i=0;i<210;++i) Sample();
  assert(IMU().status == JY901S_CALIBRATING && IMU().calibration_samples == 0);
  Calibrate();
  /* Split packets over polling calls; a DMA IDLE does not mark a full frame. */
  data_ready = false;
  {
    const uint8_t packet[11] = {0x55,0x53,0,0,0,0,0,0xc0,0,0,0x68};
    Frame(0x52,0,0,0,0);
    FeedBytes(packet,4); JY901S_Poll();
    FeedBytes(packet+4,7); JY901S_Poll();
    assert(fabsf(IMU().yaw_deg + 90.0f) < 0.02f);
  }
  Calibrate();
  module_roll = 20;
  assert(JY901S_Init() == HAL_OK);
  for (unsigned i=0;i<210;++i) Sample();
  assert(IMU().status == JY901S_CALIBRATING && IMU().calibration_samples == 0);
  Calibrate();
  /* Timestamp arithmetic and stream freshness survive the 32-bit tick wrap. */
  now_ms = UINT32_MAX - 1000U;
  Calibrate();
  assert(JY901S_IsFresh(&(JY901S_Feedback){.status=JY901S_READY,
    .last_angle_ms=UINT32_MAX-10U,.last_gyro_ms=UINT32_MAX-10U,
    .last_sample_ms=UINT32_MAX-10U},5U));
}

static void TestCalibrationStreamGaps(void)
{
  Calibrate();
  assert(JY901S_Init()==HAL_OK);
  for (unsigned i=0;i<199;++i) Sample();
  assert(IMU().status==JY901S_CALIBRATING && IMU().calibration_samples==199);
  /* A startup interruption resets the stationary window, without reconfiguring. */
  data_ready=false;
  for (unsigned i=0;i<10;++i) Sample();
  assert(IMU().status==JY901S_CALIBRATING && IMU().calibration_samples==0);
  assert(!JY901S_IsFresh(&(JY901S_Feedback){.status=JY901S_CALIBRATING},now_ms));
  data_ready=true; Sample();
  assert(IMU().status==JY901S_CALIBRATING && IMU().calibration_samples==1);
  for (unsigned i=0;i<199;++i) Sample();
  assert(IMU().status==JY901S_READY);

  /* Also reject a gap when no Poll ran during it, even if fresh frames follow. */
  assert(JY901S_Init()==HAL_OK);
  for (unsigned i=0;i<199;++i) Sample();
  data_ready=false; (void)osDelay(100U);
  data_ready=true; Sample();
  assert(IMU().status==JY901S_CALIBRATING && IMU().calibration_samples==1);
  data_ready=false;
  for (unsigned i=0;i<51;++i) Sample();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_ANGLE_TIMEOUT);

  Calibrate(); assert(JY901S_Init()==HAL_OK); Sample();
  gyro_enabled=false;
  for (unsigned i=0;i<10;++i) Sample();
  assert(IMU().status==JY901S_CALIBRATING && IMU().calibration_samples==0);
  for (unsigned i=0;i<41;++i) Sample();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_GYRO_TIMEOUT);

  /* Neither UART errors in calibration nor data loss in READY get this grace. */
  Calibrate(); assert(JY901S_Init()==HAL_OK);
  huart3.ErrorCode=2U; JY901S_UartError(&huart3); JY901S_Poll();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_UART);
  assert(IMU().uart_error==2U);
  Calibrate(); data_ready=false;
  for (unsigned i=0;i<6;++i) Sample();
  assert(IMU().status==JY901S_FAULT && IMU().fault_reason==JY901S_DIAG_ANGLE_TIMEOUT);
  Calibrate();
}

static void TestChassis(void)
{
  ChassisFeedback feedback;
  Calibrate();
  assert(Chassis_Init() == HAL_OK);
  Chassis_SetVelocity(200, 0);
  for (unsigned sample = 0; sample < 20; ++sample)
  { Sample(); Chassis_ControlStep(); }
  feedback = Chassis();
  assert(feedback.heading_active);
  assert(feedback.target_mm_s[0] == feedback.target_mm_s[2]);
  assert(pwm_output[0] > 0 && pwm_output[2] > 0);

  /* 已锁定0度后人为向左偏，应该右侧目标低于左侧以向右纠偏。 */
  gyro[2] = 655;
  for (unsigned sample = 0; sample < 20; ++sample)
  { Sample(); Chassis_ControlStep(); }
  feedback = Chassis();
  assert(feedback.heading_output_dps < 0);
  assert(feedback.target_mm_s[0] > feedback.target_mm_s[2]);

  Chassis_SetVelocity(0, 0);
  SettleToStop(); /* 普通停车不应受漂移驱动自转；减速受整形限幅，故留出整定时间。 */

  Chassis_SetHeading(0, HeadingPID_WrapDegrees(IMU().yaw_deg + 30));
  for (unsigned sample = 0; sample < 6; ++sample)
  { Sample(); Chassis_ControlStep(); }
  feedback = Chassis();
  assert(feedback.heading_output_dps > 0);
  assert(feedback.target_mm_s[0] < 0 && feedback.target_mm_s[2] > 0);
  assert(pwm_output[0] < 0 && pwm_output[2] > 0);

  io_fail = true;
  Sample(); Chassis_ControlStep();
  AssertStopped();
  Calibrate();
  Chassis_ControlStep();
  AssertStopped(); /* 恢复不能重新执行故障前的转向命令。 */

  Chassis_SetVelocity(200, 0);
  Sample(); Chassis_ControlStep();
  assert(pwm_output[0] != 0);
  now_ms += 51;
  Chassis_SafetyStep();
  AssertStopped(); /* 停止轮询时仍由安全任务撤销PWM。 */

  Calibrate();
  Chassis_SetVelocity(200, 0);
  for (unsigned sample = 0; sample < 51; ++sample)
  { Sample(); Chassis_ControlStep(); }
  AssertStopped(); /* 有新IMU数据但控制命令超时。 */

  Chassis_SetVelocity(0, 100);
  for (unsigned sample = 0; sample < 10; ++sample)
  { Sample(); Chassis_ControlStep(); }
  assert(Chassis().heading_target_deg > IMU().yaw_deg);
  assert(pwm_output[0] < 0 && pwm_output[2] > 0);
  Chassis_EmergencyStop();
  AssertStopped();
  assert(test_irq_mask == 0U);
}

/*
 * 航向目标不允许"自己推自己"。
 *
 * 背景：整形改造时曾把航向目标的累加从"指令角速度"换成"整形后的实际差速"。
 * 但那个值里含外环自己的修正量，于是构成正反馈：
 *     误差↑ → 修正差速↑ → 目标被推得更远 → 误差↑ → …
 * 误差永远收不回来，目标一路跑到 ±180° 回绕边界后符号翻转，实车上表现为
 * A/B 与 C/D 两组轮子交替快慢（实测目标跑到 −169°，而角速度指令是 0）。
 *
 * 这类 bug 只在长时间运行后暴露，原来那几条瞬时断言抓不到，所以单独加一条：
 * 人为制造一个持续扰动让外环输出修正差速，然后要求目标航向纹丝不动——
 * 因为角速度指令是 0，目标就不该被推进哪怕 0.01°。
 */
static void TestHeadingTargetDoesNotSelfDrive(void)
{
  ChassisFeedback feedback;

  Calibrate();
  assert(Chassis_Init() == HAL_OK);

  /* 车体被持续左转扰动：外环必然输出一个负的修正差速。 */
  gyro[2] = 655; /* 10 deg/s */
  Chassis_SetVelocity(200, 0);
  Sample();
  Chassis_ControlStep();
  /* 起步瞬间目标航向会锁存为当时的实测航向，把它记下来当基准。 */
  const float target_at_start = Chassis().heading_target_deg;

  for (unsigned sample = 0; sample < 100; ++sample)
  {
    Chassis_SetVelocity(200, 0);
    Sample();
    Chassis_ControlStep();
  }
  feedback = Chassis();

  /* 先确认这个场景确实产生了足够的修正差速，否则测试没有意义。 */
  assert(feedback.heading_active);
  assert(feedback.heading_output_dps < -5.0f);

  /* 角速度指令为 0 → 目标航向必须停在起步锁定值上，一位都不能被推进。
   * 有 bug 时这里会跑掉几十度。 */
  assert(fabsf(feedback.heading_target_deg - target_at_start) < 0.01f);

  Chassis_EmergencyStop();
}

/*
 * 指令的行结束符兜底（2026-10-13）。
 *
 * 背景：VOFA+ 发送框旁那个"追加换行"下拉，某些情况下控件点不动/失效，于是发出去
 * 的指令末尾既没有 \r 也没有 \n。而 Chassis_ProcessCommands() 原本只认 \r/\n 作为
 * 终止符 —— 指令会永远攒在缓冲里等不到结算，实车上表现为"在 VOFA 里发指令车不动"。
 *
 * 现在补了一条空闲超时兜底。这条测试锁住三件事：
 *   1) 带换行的指令仍然立刻生效（不能因为加了兜底反而变慢）
 *   2) 不带换行时，超时之前一个字节都不能生效
 *   3) 被链路拆成两半、间隔小于阈值的指令，不能被"腰斩执行"
 *
 * 观测量用 heading_active：它由指令模式直接决定（V=0 → false，V≠0 → true），
 * 不经过给定整形，所以不受斜坡影响，可以逐拍断言。
 */
static void TestCommandIdleTimeout(void)
{
  /* 必须先让 IMU 就绪并保持新鲜：Chassis_ControlStep() 开头有安全检查
   * （IMU 数据不新鲜或指令超时都会直接急停返回），若不喂 Sample()，
   * 它会提前 return，heading_active 永远为 false，断言会"假通过"。 */
  Calibrate();
  assert(Chassis_Init() == HAL_OK);

  /* 基准：压到停车状态 */
  RxClear();
  Chassis_SetVelocity(0, 0);
  Sample();
  Chassis_ProcessCommands();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);

  /* --- 1) 带换行：立刻生效，不必等超时 --- */
  RxPush("V=200\n");
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(Chassis().heading_active);

  /* --- 2) 不带换行：超时之前绝不能生效 --- */
  RxClear();
  Chassis_SetVelocity(0, 0);
  Sample();
  Chassis_ProcessCommands();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);

  RxPush("V=-150");
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);   /* 刚收到，不能执行 */

  Advance(100U);                       /* 总空闲约 110 ms < 150 ms 阈值 */
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);   /* 仍然不能执行 */

  /* 注意不能用 +1000：那会超过指令 500 ms 超时，触发急停，
   * heading_active 会因为急停而不是因为指令生效变成 false/true，测试失真。
   * +200 既越过 150 ms 空闲阈值，又留在 500 ms 指令有效期之内。 */
  Advance(200U);
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(Chassis().heading_active);    /* 这时才结算执行 */

  /* --- 3) 半条指令在超时内被补齐，不能被腰斩 --- */
  RxClear();
  Chassis_SetVelocity(0, 0);
  Sample();
  Chassis_ProcessCommands();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);

  RxPush("V=25");
  Chassis_ProcessCommands();
  Advance(100U);                       /* 小于阈值 */
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);   /* 不能被当成 V=25 提前执行 */

  RxPush("0");                         /* 补成 V=250，同时重置空闲计时 */
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(!Chassis().heading_active);   /* 仍没有换行，还没到时候 */

  Advance(200U);
  Chassis_ProcessCommands();
  Sample();
  Chassis_ControlStep();
  assert(Chassis().heading_active);    /* 结算，V=250 生效 */

  Chassis_EmergencyStop();
  RxClear();
}

/* 验证上电默认值与本轮选定的在线参数产生相同的起步、转弯、停车输出。 */
static void RecordTunedTrajectory(ChassisFeedback trace[140])
{
  Calibrate();
  assert(Chassis_Init() == HAL_OK);
  for (unsigned sample = 0; sample < 140; ++sample)
  {
    if (sample < 50) Chassis_SetVelocity(200, 0);
    else if (sample < 90) Chassis_SetVelocity(200, 50);
    else Chassis_SetVelocity(0, 0);
    Sample();
    Chassis_ControlStep();
    trace[sample] = Chassis();
  }
}

static void TestTunedBootDefaults(void)
{
  ChassisFeedback boot_trace[140];
  ChassisFeedback explicit_trace[140];

  /* 必须是本进程第一个底盘测试，才能验证尚未被串口修改的静态默认值。 */
  RecordTunedTrajectory(boot_trace);
  assert(boot_trace[4].pwm[0] >= 671); /* 离开零速死区后起步含8%前馈。 */
  RxPush("FF=8\nLACC=600\nLJRK=4000\nDACC=700\nDJRK=6000\nRESP=0.15\n");
  Chassis_ProcessCommands();
  RecordTunedTrajectory(explicit_trace);
  for (unsigned sample = 0; sample < 140; ++sample)
  {
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
      assert(boot_trace[sample].target_mm_s[wheel] ==
             explicit_trace[sample].target_mm_s[wheel]);
      assert(boot_trace[sample].pwm[wheel] == explicit_trace[sample].pwm[wheel]);
    }
    assert(fabsf(boot_trace[sample].heading_output_dps -
                 explicit_trace[sample].heading_output_dps) < 0.001f);
  }
  Chassis_EmergencyStop();
  RxClear();
}

static unsigned aux_seen;
static bool TestAuxHandler(const char *command)
{
  ++aux_seen;
  return strcmp(command, "PAN_US=1520") == 0;
}

static void TestAuxCommandRouter(void)
{
  RxClear();
  Chassis_SetAuxCommandHandler(TestAuxHandler);
  RxPush("PAN_US=1520\n");
  Chassis_ProcessCommands();
  assert(aux_seen == 1U);
  RxPush("TRATE=123\n");
  Chassis_ProcessCommands();
  assert(aux_seen == 2U && Chassis_GetTelemetryPeriodMs() == 123U);
  Chassis_SetAuxCommandHandler(NULL);
  RxPush("TRATE=100\n");
  Chassis_ProcessCommands();
  assert(aux_seen == 2U && Chassis_GetTelemetryPeriodMs() == 100U);
  RxClear();
}

/* RK3588脚本的真实文本帧：持续20ms刷新、停止刷新、ZERO急停。 */
static void TestGamepadTextSession(void)
{
  Calibrate();
  assert(Chassis_Init() == HAL_OK);
  RxClear();
  RxPush("\nTRATE=100\nHOLD=0\nZERO\n");
  Chassis_ProcessCommands();
  AssertStopped();

  for (unsigned sample = 0; sample < 80; ++sample)
  {
    if ((sample % 2U) == 0U) RxPush("HOLD=0\nV=200\nW=25\n");
    Chassis_ProcessCommands();
    Chassis_RefreshCommand();
    Sample();
    Chassis_ControlStep();
  }
  assert(pwm_output[0] != 0 || pwm_output[2] != 0);
  assert(Chassis().heading_target_deg > IMU().yaw_deg);

  /* 模拟kill -9/USB断线，IMU仍正常，communicationTask仍调用Refresh。 */
  for (unsigned sample = 0; sample < 51; ++sample)
  {
    Chassis_RefreshCommand();
    Sample();
    Chassis_ControlStep();
    Chassis_SafetyStep();
  }
  assert(IMU().status == JY901S_READY);
  assert(Chassis_GetLastStopReason() == CHASSIS_STOP_COMMAND_TIMEOUT);
  AssertStopped();

  for (unsigned sample = 0; sample < 20; ++sample)
  {
    if ((sample % 2U) == 0U) RxPush("HOLD=0\nV=200\nW=0\n");
    Chassis_ProcessCommands();
    Sample();
    Chassis_ControlStep();
  }
  assert(pwm_output[0] != 0);
  RxPush("\nHOLD=0\nZERO\n");
  Chassis_ProcessCommands();
  AssertStopped(); /* 不等减速斜坡、不等下一拍控制。 */
  Chassis_RefreshCommand();
  AssertStopped();
  RxClear();
  RxPush("HOLD=1\n"); /* 还原其他测试/调参使用的默认行为。 */
  Chassis_ProcessCommands();
}

static void TestTrackingModeAndStopLatch(void)
{
  Calibrate();
  assert(Chassis_Init()==HAL_OK);
  RxClear(); RxPush("STOP\n");
  Chassis_StopWithReason(CHASSIS_STOP_HANDOVER);
  assert(rx_head!=rx_tail); /* Start/handover must not erase a following STOP. */
  Chassis_ProcessCommands();
  for (unsigned i=0;i<200;i++) {
    Chassis_SetTrackingVelocity(200,50);
    Sample(); Chassis_ControlStep();
  }
  assert(fabsf(Chassis().heading_target_deg-IMU().yaw_deg)<0.01f);
  assert(Chassis().target_mm_s[0]<Chassis().target_mm_s[2]);
  gyro[2]=3275; /* 50 deg/s exceeds the 28.65 deg/s command. */
  Sample(); Chassis_SetTrackingVelocity(200,50); Chassis_ControlStep();
  assert(Chassis().heading_output_dps<28.65f);
  for (unsigned i=0;i<4;i++) speed_feedback[i]=50;
  Chassis_EmergencyStop();
  for (unsigned i=0;i<5;i++) { Sample(); Chassis_ControlStep(); AssertStopped(); }
  for (unsigned i=0;i<4;i++) { speed_feedback[i]=0; total_counts[i]=1320; }
  assert(fabsf(Chassis_GetDistanceMm()-65.5f*3.1415926f)<0.01f);
}

int main(void)
{
  TestTunedBootDefaults();
  TestHeadingPID();
  TestSensor();
  TestJY901SProtocol();
  TestCalibrationStreamGaps();
  TestChassis();
  TestHeadingTargetDoesNotSelfDrive();
  TestCommandIdleTimeout();
  TestAuxCommandRouter();
  TestGamepadTextSession();
  TestTrackingModeAndStopLatch();
  puts("JY901S UART/DMA + heading/speed cascade tests passed.");
  return 0;
}
