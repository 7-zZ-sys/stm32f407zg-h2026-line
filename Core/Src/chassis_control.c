#include "chassis_control.h"

#include <limits.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "encoder.h"
#include "motor_driver.h"
#include "speed_pid.h"
#include "motion_profile.h"
#include "tim.h"
#include "heading_pid.h"
#include "jy901s.h"
#include "Vofa.h"
#include "line_config.h"

static ChassisAuxCommandHandler s_aux_command_handler;

void Chassis_SetAuxCommandHandler(ChassisAuxCommandHandler handler)
{
  s_aux_command_handler = handler;
}

/*
 * 单轮最大线速度。按标定后的轮径 65.5 mm：250 RPM × π × 65.5 ÷ 60 = 857.4 mm/s，
 * 取 851（沿用值）：851 ÷ (π × 65.5 ÷ 60) ≈ 248 RPM，仍在 250 RPM 以内，够用。
 *
 * 注意这个常量是"单轮"上限：差速分配后 |linear| + |角速度分量| 不会超过它，
 * 所以它同时决定了航向环有多少差速余量（见 Chassis_ControlStep 里的 angular_limit_dps）。
 * 原来是 600，正好等于旧的设计线速度，导致满速时差速余量为 0、航向环失去修正能力；
 * 抬到 851 之后 600 mm/s 处仍有 (851−600)×2÷250×57.3 ≈ 115 dps 的几何余量，
 * 再被 HEADING_MAX_RATE_DPS(=60 dps) 封顶，实际留给纠偏的是 60 dps。
 *
 * 注：这里原来写的是"约 144 dps、30 dps 封顶"，两个数都是旧值。轮距从物理中心距
 * 170 mm 换成标定后的有效值 250 mm 之后，几何余量从 169 dps 变成 115 dps —— 
 * 仍然远高于 60 dps 的限幅，所以真正生效的一直是限幅。
 */
#define CHASSIS_MAX_LINEAR_SPEED_MM_S   851
/* 轮距：直接进差速公式 v左/右 = v ∓ ω·W/2 的那个 W。
 *
 * ⚠️ 这里要填的是「有效轮距」，不是量出来的物理中心距！
 * 滑移转向靠轮胎横向刮擦来转向，同样的轮速差下实际转过的角度比纯滚动几何
 * 小，等效于"轮距更大"。所以有效值通常明显大于物理值（这辆车大 47%）。
 *
 * 标定方法（只用遥测，不需要外部测量）：左右轮实测速度差 = ω实测(rad/s) × W有效
 *   即  W有效 = [(C+D)/2 − (A+B)/2] ÷ (yaw_rate_dps × 0.01745329)
 * 实测（2026-10-13，硬质地板上，轮径按 65.5 标定后）：
 *   原地转   V=0   W=50 → 248 mm（21 帧）
 *   边走边转 V=200 W=50 → 254 mm（22 帧）
 * 两种工况几乎一致，取 250 mm。
 * 物理中心距是 170 mm —— 填 170 会让外环一直带着约 8° 的稳态误差去补前馈
 * 的缺口（输出 41 dps 而前馈只需要 28.65），白白多打 43% 的差速。
 * 运行时可改：TRACK=<mm> */
#define CHASSIS_TRACK_WIDTH_MM_DEFAULT   250.0f
#define CHASSIS_COMMAND_TIMEOUT_MS      500U
/* 轮径：按实车标定（2026-10-13）。
 *
 * 标定法：跑一条已知长度的直线，量用时。
 *   轮径新 = 旧轮径 × (实测速度 ÷ 指令速度)
 * 实测：V=200 跑 4000 mm 用时 19.85 s → 201.5 mm/s
 *       → 65.0 × 201.5 / 200 = 65.5 mm（只差 0.8%，说明原沿用值本来就很准）
 *
 * 注意：这个值同时吸收了「编码器每圈脉冲数」的误差，是整条 mm/s ↔ RPM 链的
 * 总比例。所以不用再单独校 1320 counts/rev。
 * 运行时可改：DIAM=<mm> */
#define CHASSIS_WHEEL_DIAMETER_MM_DEFAULT 65.5f
#define CHASSIS_PI_F                    3.1415926f

/* 上面两个是运行时变量（DIAM=/TRACK= 可改），所以不能用 #define：
 * 写入在 communicationTask，读取在 controlTask，故加 volatile。 */
static volatile float s_track_width_mm = CHASSIS_TRACK_WIDTH_MM_DEFAULT;
static volatile float s_wheel_diameter_mm = CHASSIS_WHEEL_DIAMETER_MM_DEFAULT;

/*
 * 遥测周期（ms）。
 *
 * 默认 100：无线链路（ESP8266 透传）在 50 ms 下会丢字节，100 ms 才稳。
 * 但整形的加减速过程通常不到1秒，100 ms 采样点较少，看不清 S 形、
 * 也量不了响应时间。所以做成在线可调：精细分析时用 TRATE=20 提高分辨率。
 *
 * ⚠️ 无线在 20~50 ms 下仍会丢字节，精细分析请插 USB 走 USART1。
 *
 * 它本质是遥测/UI 的事，放在这个文件只是因为串口指令解析器在这里。
 */
#define CHASSIS_TELEMETRY_PERIOD_DEFAULT_MS  100U
#define CHASSIS_TELEMETRY_PERIOD_MIN_MS       20U
#define CHASSIS_TELEMETRY_PERIOD_MAX_MS     1000U
static volatile uint32_t s_telemetry_period_ms = CHASSIS_TELEMETRY_PERIOD_DEFAULT_MS;

uint32_t Chassis_GetTelemetryPeriodMs(void)
{
  return s_telemetry_period_ms;
}

#define CHASSIS_STOP_THRESHOLD_RPM      0.05f
/* 控制周期。整个文件（编码器测速、航向外环 0.020 s、给定整形）都按固定的
 * 10 ms 设计，与 TIM7 的 Prescaler/Period 对应，所以这里用常量而不是实测 dt。 */
#define CHASSIS_CONTROL_PERIOD_S        0.010f
/* PWM占空比标定：TIM3的ARR=8399（Prescaler=0 → 10 kHz，商家建议值）。
 * 速度环输出为±100%，换算成占空比计数±8399，与MOTOR_PWM_LIMIT保持一致。 */
#define CHASSIS_PWM_LIMIT               8399.0f
#define CHASSIS_PWM_PER_PERCENT         (CHASSIS_PWM_LIMIT / 100.0f)
#define CHASSIS_HEADING_PERIOD_S        0.020f
#define CHASSIS_DEG_TO_RAD              0.01745329252f
#define CHASSIS_RAD_TO_DEG              57.29577951f

/* ================================================================== */
/* 给定整形：让家用行驶更平稳                                          */
/* ================================================================== */
/*
 * 这一组参数取代了原来"每 10 ms 每个轮子最多变化 4 RPM 的线性斜坡"。
 * 原来的做法有三个问题，每一个都直接表现为"不够平稳"：
 *
 * 1) 斜坡做在"轮空间"。左右轮从同一速度起步、以相同斜率爬升，差速必须等
 *    前一个轮子先到顶之后才拉得开。以 (200 mm/s, 左转 0.5 rad/s) 为例：
 *      Δ = 0.5 × 170 / 2 = 42.5 mm/s，左轮 46.3 RPM、右轮 71.4 RPM，
 *      两轮都以 4 RPM/步 从 0 爬升 → 前 11 步（约 116 ms）两轮同速、车在
 *      直行，之后 6 步（63 ms）才把整个转向补出来。曲率与指令完全不符，
 *      航向外环被迫去纠正这个结构性误差。
 *    → 现在在 (线速度 v, 差速 Δ) 空间联合规划，四轮目标恒等于 v ± Δ，
 *      整段检查轮速可行性。暂态曲率会变化，不宣称全过程等于指令曲率。
 *
 * 2) 加速度是突变的。原来斜率恒定 4 RPM/10 ms = 1361 mm/s²，起步瞬间从
 *    0 跳到该值，加加速度(jerk)无穷大 → 起步"点头"、停车"顿一下"。
 *    → 现在用三次速度轨迹，保留起始速度/加速度，终点加速度为0；
 *      验证整段加速度与jerk极值，不能通过末端吸附绕过约束。
 *
 * 3) 低速没有静摩擦补偿。实测约 20 RPM（≈68 mm/s）以下有卡顿门槛；纯 PI
 *    要靠积分慢慢把力矩"憋"过静摩擦，一旦冲开就窜出去，表现为低速一顿一顿。
 *    → 现在有静摩擦前馈直接给出启动力矩（幅值需实测，见 FF= 指令）。
 *
 * 全部参数都可经串口在线调整，不必重新编译：
 *   RESP=<s>    轨迹规划的最短时间尺度（默认 0.15），实际时长受约束拉长
 *   LACC=<v>    线加速度上限 mm/s²（默认 600）
 *   LJRK=<v>    线加加速度上限 mm/s³（默认 4000）
 *   DACC=<v>    差速加速度上限 mm/s²（默认 700）
 *   DJRK=<v>    差速加加速度上限 mm/s³（默认 6000）
 *   FF=<%>      静摩擦前馈幅值，0 = 关闭（默认 8，经用户实车选定）
 *
 * 默认参数下纯直行0→200 mm/s，浮点给定约0.55秒到达；不是车体实测耗时。
 * 在线换目标继承当前速度与加速度；不可行时继续已验证的旧轨迹并逐拍重试，
 * 不硬裁剪状态。降低限值也在可行时生效，不能瞬间把已有加速度变小。
 *
 * 普通零速度给定经过柔和减速；循迹STOP/ZERO由应用层立即锁存停机。
 *    需要"立即停"的场合——急停、命令超时、IMU 失效——都直接走
 *    Chassis_EmergencyStop()，不经过整形，所以不受影响。
 */
/* 2026-10-14：固化用户本轮实车选定值；在线修改仍只影响RAM，重启恢复此组。 */
#define CHASSIS_SHAPE_RESPONSE_DEFAULT  0.15f
#define CHASSIS_LINEAR_ACCEL_DEFAULT    600.0f
#define CHASSIS_LINEAR_JERK_DEFAULT     4000.0f
#define CHASSIS_DIFF_ACCEL_DEFAULT      700.0f
#define CHASSIS_DIFF_JERK_DEFAULT       6000.0f
#define CHASSIS_FRICTION_FF_DEFAULT     8.0f


/* 串口在线调参的范围钳位，防止误输入把整形打坏（例如 RESP=0 → ωn 无穷大）。 */
#define CHASSIS_SHAPE_RESPONSE_MIN_S    0.05f
#define CHASSIS_SHAPE_RESPONSE_MAX_S    2.0f
#define CHASSIS_SHAPE_ACCEL_MAX         20000.0f
#define CHASSIS_SHAPE_JERK_MAX          200000.0f
#define CHASSIS_FRICTION_FF_MAX         30.0f

/* 单次 32 位读写在 Cortex-M4 上是原子的，所以这几项不需要临界区保护。 */
static volatile float s_shape_response_s = CHASSIS_SHAPE_RESPONSE_DEFAULT;
static volatile float s_shape_linear_accel = CHASSIS_LINEAR_ACCEL_DEFAULT;
static volatile float s_shape_linear_jerk = CHASSIS_LINEAR_JERK_DEFAULT;
static volatile float s_shape_diff_accel = CHASSIS_DIFF_ACCEL_DEFAULT;
static volatile float s_shape_diff_jerk = CHASSIS_DIFF_JERK_DEFAULT;
static volatile float s_shape_friction_ff = CHASSIS_FRICTION_FF_DEFAULT;

/* 串口在线调参的增益范围钳位。与520台架工程保持一致，
 * 防止文本框误输入（例如KP=999）把环路直接打到发散。 */
#define CHASSIS_SPEED_KP_MAX            10.0f
#define CHASSIS_SPEED_KI_MAX            1.0f
#define CHASSIS_SPEED_KD_MAX            10.0f
#define CHASSIS_HEADING_KP_MAX          10.0f
#define CHASSIS_HEADING_KI_MAX          1.0f
#define CHASSIS_HEADING_KD_MAX          10.0f


typedef enum { COMMAND_VELOCITY = 0, COMMAND_HEADING = 1, COMMAND_TRACKING = 2 } CommandMode;

typedef struct
{
  volatile int16_t linear_mm_s;
  volatile int16_t angular_centirad_s;
  volatile uint32_t last_command_tick;
  volatile CommandMode command_mode;
  volatile float commanded_heading_deg;
  HeadingPID heading_pid;
  float heading_target_deg;
  float heading_measured_deg;
  float heading_output_dps;
  bool heading_active;
  uint8_t heading_divider;
  CommandMode previous_mode;
  int16_t previous_linear;
  float reference_rpm[CHASSIS_WHEEL_COUNT];
  float previous_reference_rpm[CHASSIS_WHEEL_COUNT];
  /* 给定整形状态：在 (线速度, 差速) 两个通道上各自保存"值"和"加速度"。
   * 每个轮子的给定由这两项组合而成（v ± Δ），不再逐轮做斜坡。 */
  float ramp_linear_mm_s;
  float ramp_linear_accel;
  float ramp_diff_mm_s;
  float ramp_diff_accel;
  float measured_rpm[CHASSIS_WHEEL_COUNT];
  int16_t target_mm_s[CHASSIS_WHEEL_COUNT];
  int16_t measured_mm_s[CHASSIS_WHEEL_COUNT];
  int16_t pwm[CHASSIS_WHEEL_COUNT];
} ChassisState;

static ChassisState s_chassis;
static MotionProfile s_profile;
static volatile uint32_t s_motion_epoch;
static bool s_stop_latched;
/* 当前停机条件与首次记录原因分开：已超时停车时出现IMU故障仍是新事件。 */
static ChassisStopReason s_stop_condition;
static volatile ChassisStopReason s_last_stop_reason;
static void Chassis_CancelMotionIntent(bool invalidate_partial);

static bool Chassis_ImuReady(void)
{
  JY901S_Feedback imu;
  JY901S_GetFeedback(&imu);
  return JY901S_IsFresh(&imu, HAL_GetTick());
}

static float Chassis_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

/*
 * 静摩擦前馈。只要给定非零就叠加一个固定力矩，让轮子立刻越过静摩擦门槛，
 * 不再靠积分慢慢"憋"出来。幅值 magnitude_percent 必须实测：
 * 给一个很小的 V= 指令，看轮子刚开始转动时 VOFA 通道 9~12 的 PWM 读数。
 *
 * 前馈交给 SpeedPID 合成后统一限幅，条件积分检查包含前馈的真实输出。
 */
static float Chassis_FrictionFeedforward(float reference_rpm, float magnitude_percent)
{
  if (magnitude_percent <= 0.0f)
  {
    return 0.0f;
  }
  if (Chassis_AbsFloat(reference_rpm) <= CHASSIS_STOP_THRESHOLD_RPM)
  {
    return 0.0f;
  }
  return (reference_rpm > 0.0f) ? magnitude_percent : -magnitude_percent;
}

static bool Chassis_IsDirectionReversed(float previous, float current)
{
  return ((current > CHASSIS_STOP_THRESHOLD_RPM &&
           previous < -CHASSIS_STOP_THRESHOLD_RPM) ||
          (current < -CHASSIS_STOP_THRESHOLD_RPM &&
           previous > CHASSIS_STOP_THRESHOLD_RPM));
}

static int16_t Chassis_ClampSpeed(int32_t speed)
{
  if (speed > CHASSIS_MAX_LINEAR_SPEED_MM_S)
  {
    speed = CHASSIS_MAX_LINEAR_SPEED_MM_S;
  }
  else if (speed < -CHASSIS_MAX_LINEAR_SPEED_MM_S)
  {
    speed = -CHASSIS_MAX_LINEAR_SPEED_MM_S;
  }

  return (int16_t)speed;
}

static float Chassis_MmSToRpm(float speed_mm_s)
{
  return (speed_mm_s * 60.0f) /
         (CHASSIS_PI_F * s_wheel_diameter_mm);
}

static float Chassis_RpmToMmS(float speed_rpm)
{
  return (speed_rpm * CHASSIS_PI_F * s_wheel_diameter_mm) / 60.0f;
}

static int16_t Chassis_FloatToInt16(float value)
{
  if (value > (float)INT16_MAX)
  {
    return INT16_MAX;
  }
  if (value < (float)INT16_MIN)
  {
    return INT16_MIN;
  }

  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t Chassis_PercentToPwm(float output_percent)
{
  float pwm = output_percent * CHASSIS_PWM_PER_PERCENT;

  if (pwm > CHASSIS_PWM_LIMIT)
  {
    pwm = CHASSIS_PWM_LIMIT;
  }
  else if (pwm < -CHASSIS_PWM_LIMIT)
  {
    pwm = -CHASSIS_PWM_LIMIT;
  }

  return Chassis_FloatToInt16(pwm);
}

HAL_StatusTypeDef Chassis_Init(void)
{
  s_stop_latched = false;
  s_stop_condition = CHASSIS_STOP_NONE;
  s_last_stop_reason = CHASSIS_STOP_NONE;
  Chassis_CancelMotionIntent(true);
  MotionProfile_Reset(&s_profile);
  SpeedPID_Init();
  HeadingPID_Init(&s_chassis.heading_pid);
  s_chassis.command_mode = COMMAND_VELOCITY;
  s_chassis.commanded_heading_deg = 0.0f;
  s_chassis.heading_active = false;
  s_chassis.heading_divider = 0U;
  s_chassis.heading_output_dps = 0.0f;
  s_chassis.ramp_linear_mm_s = 0.0f;
  s_chassis.ramp_linear_accel = 0.0f;
  s_chassis.ramp_diff_mm_s = 0.0f;
  s_chassis.ramp_diff_accel = 0.0f;

  s_chassis.linear_mm_s = 0;
  s_chassis.angular_centirad_s = 0;
  s_chassis.last_command_tick = HAL_GetTick();
  for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
  {
    s_chassis.reference_rpm[wheel] = 0.0f;
    s_chassis.previous_reference_rpm[wheel] = 0.0f;
    s_chassis.measured_rpm[wheel] = 0.0f;
    s_chassis.target_mm_s[wheel] = 0;
    s_chassis.measured_mm_s[wheel] = 0;
    s_chassis.pwm[wheel] = 0;
  }

  if (MotorDriver_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (Encoder_Start() != HAL_OK)
  {
    MotorDriver_StopAll();
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
    Encoder_Stop();
    MotorDriver_StopAll();
    return HAL_ERROR;
  }

  return HAL_OK;
}

void Chassis_SetVelocity(int16_t linear_mm_s, int16_t angular_centirad_s)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if ((linear_mm_s != 0 || angular_centirad_s != 0) && !Chassis_ImuReady())
  {
    Chassis_StopWithReason(CHASSIS_STOP_IMU);
    if (primask == 0U) __enable_irq();
    return;
  }

  s_chassis.linear_mm_s = Chassis_ClampSpeed(linear_mm_s);
  if (linear_mm_s != 0 || angular_centirad_s != 0) s_stop_latched = false;
  s_chassis.angular_centirad_s = angular_centirad_s;
  s_chassis.command_mode = COMMAND_VELOCITY;
  s_chassis.last_command_tick = HAL_GetTick();

  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Chassis_SetHeading(int16_t linear_mm_s, float heading_deg)
{
  uint32_t primask;
  if (!isfinite(heading_deg)) return;
  primask = __get_PRIMASK();
  __disable_irq();
  if (!Chassis_ImuReady())
  {
    Chassis_StopWithReason(CHASSIS_STOP_IMU);
    if (primask == 0U) __enable_irq();
    return;
  }
  s_chassis.linear_mm_s = Chassis_ClampSpeed(linear_mm_s);
  s_stop_latched = false;
  s_chassis.angular_centirad_s = 0;
  s_chassis.commanded_heading_deg = HeadingPID_WrapDegrees(heading_deg);
  s_chassis.command_mode = COMMAND_HEADING;
  s_chassis.last_command_tick = HAL_GetTick();
  if (primask == 0U) __enable_irq();
}


/* Autonomous steering owns the yaw-rate reference, not an integrated heading. */
void Chassis_SetTrackingVelocity(int16_t linear_mm_s, int16_t angular_centirad_s)
{
  Chassis_SetVelocity(linear_mm_s, angular_centirad_s);
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  if (!s_stop_latched) s_chassis.command_mode = COMMAND_TRACKING;
  if (saved == 0U) __enable_irq();
}

bool Chassis_IsStopped(void) { return s_stop_latched; }

float Chassis_GetDistanceMm(void)
{
  int64_t counts[4];
  Encoder_GetTotalCounts(counts);
  double sum = (double)counts[0] + (double)counts[1] + (double)counts[2] + (double)counts[3];
  return (float)(sum * 0.25 * s_wheel_diameter_mm * CHASSIS_PI_F /
                 ENCODER_COUNTS_PER_REVOLUTION);
}

void MotorControl(uint8_t angular_direction, uint16_t angular_speed,
                  uint8_t motion_direction, uint16_t motion_speed)
{
  int16_t signed_linear;
  int16_t signed_angular;

  if (motion_speed > CHASSIS_MAX_LINEAR_SPEED_MM_S)
  {
    motion_speed = CHASSIS_MAX_LINEAR_SPEED_MM_S;
  }

  signed_linear = (motion_direction == 0U) ? (int16_t)motion_speed :
                                             -(int16_t)motion_speed;
  signed_angular = (angular_direction == 0U) ? (int16_t)angular_speed :
                                               -(int16_t)angular_speed;
  Chassis_SetVelocity(signed_linear, signed_angular);
}

void Chassis_ControlStep(void)
{
  float desired_mm_s[CHASSIS_WHEEL_COUNT];
  float output_percent;
  int16_t linear;
  int16_t angular;
  float diff_target_mm_s;
  float commanded_heading;
  float angular_limit_dps;
  float feedforward_dps;
  MotionLimits limits;
  float shape_response_s;
  float shape_linear_accel;
  float shape_linear_jerk;
  float shape_diff_accel;
  float shape_diff_jerk;
  float shape_friction_ff;
  CommandMode command_mode;
  JY901S_Feedback imu;
  uint32_t last_command_tick;
  uint32_t primask;

  Encoder_Sample10ms();
  SpeedPID_ApplyPendingGains();
  Encoder_GetSpeedsRpm(s_chassis.measured_rpm);
  for (uint32_t wheel = 0; wheel < 4; ++wheel)
    s_chassis.measured_mm_s[wheel] = Chassis_FloatToInt16(Chassis_RpmToMmS(s_chassis.measured_rpm[wheel]));
  JY901S_GetFeedback(&imu);
  s_chassis.heading_measured_deg = imu.yaw_deg;
  /* Keep encoder/yaw telemetry live, but never re-energize a latched stop. */
  if (s_stop_latched) { MotorDriver_StopAll(); return; }

  /* 失效时立即撤销PWM和旧命令；恢复后须收到新的命令才可运动。 */
  if (!JY901S_IsFresh(&imu, HAL_GetTick()))
  {
    Chassis_StopWithReason(CHASSIS_STOP_IMU);
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  linear = s_chassis.linear_mm_s;
  angular = s_chassis.angular_centirad_s;
  last_command_tick = s_chassis.last_command_tick;
  commanded_heading = s_chassis.commanded_heading_deg;
  command_mode = s_chassis.command_mode;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((HAL_GetTick() - last_command_tick) > CHASSIS_COMMAND_TIMEOUT_MS)
  {
    Chassis_StopWithReason(CHASSIS_STOP_COMMAND_TIMEOUT);
    return;
  }

  /* 读取在线可调的整形参数。单次 32 位读写在 Cortex-M4 上是原子的，不必进
   * 临界区；这里读一次到局部量，保证本周期内的取值一致。 */
  shape_response_s = s_shape_response_s;
  if (shape_response_s < CHASSIS_SHAPE_RESPONSE_MIN_S)
  {
    shape_response_s = CHASSIS_SHAPE_RESPONSE_MIN_S;
  }
  shape_linear_accel = s_shape_linear_accel;
  shape_linear_jerk = s_shape_linear_jerk;
  shape_diff_accel = s_shape_diff_accel;
  shape_diff_jerk = s_shape_diff_jerk;
  shape_friction_ff = s_shape_friction_ff;

  /* 停车不进行航向纠偏，防止静止时因陀螺仪漂移自行转动。 */
  if (linear == 0 && angular == 0 && command_mode != COMMAND_HEADING)
  {
    s_chassis.heading_active = false;
    s_chassis.heading_divider = 0U;
    s_chassis.heading_output_dps = 0.0f;
    HeadingPID_Reset(&s_chassis.heading_pid);
  }
  else if (command_mode == COMMAND_TRACKING)
  {
    /* The line loop already measures absolute lateral error: do not integrate yaw targets. */
    float requested = (float)angular * 0.01f * CHASSIS_RAD_TO_DEG;
    float correction = LINE_YAW_RATE_KP * (requested - imu.yaw_rate_dps);
    s_chassis.heading_target_deg = imu.yaw_deg;
    s_chassis.heading_active = true;
    s_chassis.heading_divider = 0U;
    HeadingPID_Reset(&s_chassis.heading_pid);
    s_chassis.heading_output_dps = fmaxf(-HEADING_MAX_RATE_DPS,
        fminf(HEADING_MAX_RATE_DPS, requested + correction));
  }
  else
  {
    if (!s_chassis.heading_active || command_mode != s_chassis.previous_mode)
    {
      s_chassis.heading_target_deg = imu.yaw_deg;
      s_chassis.heading_output_dps = 0.0f;
      s_chassis.heading_divider = 0U;
      HeadingPID_Reset(&s_chassis.heading_pid);
    }
    if ((linear > 0 && s_chassis.previous_linear < 0) ||
        (linear < 0 && s_chassis.previous_linear > 0))
      HeadingPID_Reset(&s_chassis.heading_pid);
    s_chassis.heading_active = true;

    /* 预留左右轮速度差的空间，外环限幅和最终轮速限幅保持一致。 */
    angular_limit_dps = (CHASSIS_MAX_LINEAR_SPEED_MM_S - fabsf((float)linear)) *
                        2.0f / s_track_width_mm * CHASSIS_RAD_TO_DEG;
    if (angular_limit_dps > HEADING_MAX_RATE_DPS)
      angular_limit_dps = HEADING_MAX_RATE_DPS;
    /* 前馈取"指令角速度"，不是整形后的实际值：否则起步瞬间前馈为 0，外环没有
     * 初始激励；而直行时目标航向又不累加，误差恒为 0 → 根本转不起来。 */
    feedforward_dps = (float)angular * 0.01f * CHASSIS_RAD_TO_DEG;
    if (feedforward_dps > angular_limit_dps) feedforward_dps = angular_limit_dps;
    if (feedforward_dps < -angular_limit_dps) feedforward_dps = -angular_limit_dps;

    /* 角度外环20ms；四轮速度内环仍为10ms。 */
    if (s_chassis.heading_divider == 0U)
    {
      if (command_mode == COMMAND_HEADING)
        s_chassis.heading_target_deg = commanded_heading;
      else
        /* 目标航向只能由"角速度指令"（前馈）推进，绝不能跟着整形后的实际
         * 差速走——那个值里含外环自己的修正量，会构成正反馈：
         *   误差↑ → 修正差速↑ → 目标被推得更远 → 误差↑ → …
         * 误差永远收不回来，目标一路跑到 ±180° 回绕边界后符号翻转，表现为
         * A/B 与 C/D 两组轮子交替快慢（实测目标曾跑到 −169°，而当时角速度
         * 指令是 0）。回归测试见 test_cascade.c 的
         * TestHeadingTargetDoesNotSelfDrive()。 */
        s_chassis.heading_target_deg = HeadingPID_WrapDegrees(
            s_chassis.heading_target_deg + feedforward_dps * CHASSIS_HEADING_PERIOD_S);
      s_chassis.heading_output_dps = HeadingPID_Calculate(&s_chassis.heading_pid,
          s_chassis.heading_target_deg, imu.yaw_deg, imu.yaw_rate_dps,
          feedforward_dps, CHASSIS_HEADING_PERIOD_S, angular_limit_dps);
    }
    /* 两次外环之间若线速度改变，仍立即限制角速度输出。 */
    if (s_chassis.heading_output_dps > angular_limit_dps)
      s_chassis.heading_output_dps = angular_limit_dps;
    if (s_chassis.heading_output_dps < -angular_limit_dps)
      s_chassis.heading_output_dps = -angular_limit_dps;
    s_chassis.heading_divider ^= 1U;
  }
  s_chassis.previous_mode = command_mode;
  s_chassis.previous_linear = linear;

  /* ---------- 给定整形：在 (线速度 v, 差速 Δ) 空间做 ---------- */
  diff_target_mm_s = s_chassis.heading_output_dps * CHASSIS_DEG_TO_RAD *
                     s_track_width_mm * 0.5f;

  limits = (MotionLimits){shape_response_s, shape_linear_accel, shape_linear_jerk,
                         shape_diff_accel, shape_diff_jerk,
                         (float)CHASSIS_MAX_LINEAR_SPEED_MM_S};
  if (command_mode == COMMAND_TRACKING) {
    limits.response_s = LINE_SHAPE_RESPONSE_S;
    limits.diff_accel = LINE_DIFF_ACCEL_MM_S2;
    limits.diff_jerk = LINE_DIFF_JERK_MM_S3;
  }
  MotionProfile_Step(&s_profile, (float)linear, diff_target_mm_s,
                     &limits, CHASSIS_CONTROL_PERIOD_S);
  s_chassis.ramp_linear_mm_s = s_profile.value[0];
  s_chassis.ramp_linear_accel = s_profile.accel[0];
  s_chassis.ramp_diff_mm_s = s_profile.value[1];
  s_chassis.ramp_diff_accel = s_profile.accel[1];

  /* 联合规划验证整个过渡中的 v±Δ，不仅限制最终指令。 */
  desired_mm_s[0] = s_chassis.ramp_linear_mm_s - s_chassis.ramp_diff_mm_s;
  desired_mm_s[1] = desired_mm_s[0];
  desired_mm_s[2] = s_chassis.ramp_linear_mm_s + s_chassis.ramp_diff_mm_s;
  desired_mm_s[3] = desired_mm_s[2];

  for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
  {
    bool stopping;
    float friction_ff;

    s_chassis.reference_rpm[wheel] = Chassis_MmSToRpm(desired_mm_s[wheel]);
    stopping = Chassis_AbsFloat(s_chassis.reference_rpm[wheel]) <=
               CHASSIS_STOP_THRESHOLD_RPM;

    /* 整形后的给定停在零点上、或穿过零点换向时清积分，
     * 既避免换向残留，也避免静止时积分慢慢把车顶动。 */
    if (stopping ||
        Chassis_IsDirectionReversed(s_chassis.previous_reference_rpm[wheel],
                                    s_chassis.reference_rpm[wheel]))
    {
      SpeedPID_Reset(wheel);
    }
    s_chassis.previous_reference_rpm[wheel] = s_chassis.reference_rpm[wheel];

    friction_ff = Chassis_FrictionFeedforward(s_chassis.reference_rpm[wheel],
                                              shape_friction_ff);
    output_percent = SpeedPID_CalculateWithFeedforward(wheel,
        s_chassis.measured_rpm[wheel], s_chassis.reference_rpm[wheel],
        !stopping, friction_ff);

    /* 给定与实测都到零：彻底断电，不留残余力矩。 */
    if (stopping &&
        (Chassis_AbsFloat(s_chassis.measured_rpm[wheel]) <=
         CHASSIS_STOP_THRESHOLD_RPM))
    {
      output_percent = 0.0f;
      SpeedPID_Reset(wheel);
    }

    s_chassis.target_mm_s[wheel] =
        Chassis_FloatToInt16(Chassis_RpmToMmS(s_chassis.reference_rpm[wheel]));
    s_chassis.measured_mm_s[wheel] =
        Chassis_FloatToInt16(Chassis_RpmToMmS(s_chassis.measured_rpm[wheel]));
    s_chassis.pwm[wheel] = Chassis_PercentToPwm(output_percent);
  }

  MotorDriver_SetSignedPwm(s_chassis.pwm);
}

void Chassis_SafetyStep(void)
{
  JY901S_Feedback imu;
  JY901S_GetFeedback(&imu);
  if (!JY901S_IsFresh(&imu, HAL_GetTick()))
    Chassis_StopWithReason(CHASSIS_STOP_IMU);
  else if ((HAL_GetTick() - s_chassis.last_command_tick) > CHASSIS_COMMAND_TIMEOUT_MS)
    Chassis_StopWithReason(CHASSIS_STOP_COMMAND_TIMEOUT);
}

void Chassis_EmergencyStop(void)
{
  Chassis_StopWithReason(CHASSIS_STOP_USER);
}

ChassisStopReason Chassis_GetLastStopReason(void) { return s_last_stop_reason; }

void Chassis_StopWithReason(ChassisStopReason reason)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  bool invalidate_partial = !s_stop_latched || reason == CHASSIS_STOP_USER ||
      (reason != s_stop_condition && reason != CHASSIS_STOP_COMMAND_TIMEOUT);
  /* 已停住后仅发现旧命令计时超时，不是新的运动故障；否则ZERO后的
   * 第一条分段命令也会被随后的超时检查误作废。IMU/时序等新故障仍作废。 */
  if (!s_stop_latched || reason == CHASSIS_STOP_USER) s_last_stop_reason = reason;
  /* 只在新停机事件/明确ZERO时清队列、更新代次。重复的10ms/20ms检查
   * 仍保持PWM为零，但不能作废停机后正在分段接收或等待150ms解析的新命令。 */
  /* Handover must preserve a STOP already queued behind RUN/LAP in the same burst. */
  if (invalidate_partial && reason != CHASSIS_STOP_HANDOVER) VofaPort_DiscardPendingInput();
  s_stop_latched = true;
  s_stop_condition = reason;
  /* 每次仍清上层缓存，防止未就绪时的HEAD等请求留下可重放状态。 */
  Chassis_CancelMotionIntent(invalidate_partial);
  MotionProfile_Reset(&s_profile);

  s_chassis.linear_mm_s = 0;
  s_chassis.angular_centirad_s = 0;
  s_chassis.command_mode = COMMAND_VELOCITY;
  s_chassis.heading_active = false;
  s_chassis.heading_divider = 0U;
  s_chassis.heading_output_dps = 0.0f;
  s_chassis.previous_linear = 0;
  /* 急停不走整形：直接把整形状态清零、立即断电，
   * 不受加速度/加加速度限幅影响（限幅只管正常行驶的过渡）。 */
  s_chassis.ramp_linear_mm_s = 0.0f;
  s_chassis.ramp_linear_accel = 0.0f;
  s_chassis.ramp_diff_mm_s = 0.0f;
  s_chassis.ramp_diff_accel = 0.0f;
  HeadingPID_Reset(&s_chassis.heading_pid);
  for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
  {
    s_chassis.reference_rpm[wheel] = 0.0f;
    s_chassis.previous_reference_rpm[wheel] = 0.0f;
    s_chassis.target_mm_s[wheel] = 0;
    s_chassis.pwm[wheel] = 0;
  }

  /* 与速度任务互斥地清状态并撤销PWM，避免中途被控制任务抢占。 */
  SpeedPID_ResetAll();
  MotorDriver_StopAll();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Chassis_GetFeedback(ChassisFeedback *feedback)
{
  uint32_t primask;

  if (feedback == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
  {
    feedback->target_mm_s[wheel] = s_chassis.target_mm_s[wheel];
    feedback->measured_mm_s[wheel] = s_chassis.measured_mm_s[wheel];
    feedback->pwm[wheel] = s_chassis.pwm[wheel];
  }
  feedback->heading_target_deg = s_chassis.heading_target_deg;
  feedback->heading_measured_deg = s_chassis.heading_measured_deg;
  feedback->heading_error_deg = HeadingPID_WrapDegrees(
      s_chassis.heading_target_deg - s_chassis.heading_measured_deg);
  feedback->heading_output_dps = s_chassis.heading_output_dps;
  feedback->heading_active = s_chassis.heading_active;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* ================================================================== */
/* 串口在线调参                                                       */
/* ================================================================== */
/*
 * 在VOFA控件里发送文本指令即可在线改速度和增益，不必重新编译下载。
 *
 * 规则：
 *   - 一行一条指令，结尾必须带 \n 或 \r\n；
 *   - 必须从第0列开始，关键字大小写不敏感，参数不能有空格；
 *   - 解析在communicationTask里执行，绝不在中断里跑。
 *
 * 指令表：
 *   V=<mm/s>        线速度（正值前进），保留当前角速度
 *   W=<0.01rad/s>   角速度（正值左转），保留当前线速度
 *   SPEED=<mm/s>    线速度且角速度清零
 *   HEAD=<deg>      绝对航向目标，按当前线速度行驶
 *   KP/KI/KD=<v>    四轮速度环增益一起改
 *   KP1..KP4=<v>    只改某一个轮（KI1..4、KD1..4同理）
 *   HKP/HKI/HKD=<v> 航向环增益（别名 PKP/PKI/PKD）
 *   RESP=<s>        给定整形响应时间尺度，越大越柔（默认0.15）
 *   LACC/LJRK=<v>   线加速度/加加速度上限（默认600 / 4000）
 *   DACC/DJRK=<v>   差速加速度/加加速度上限（默认700 / 6000）
 *   FF=<%>          静摩擦前馈幅值，0=关闭（默认8，用户实车选定）
 *   HOLD=0/1        关闭/开启指令自保持
 *   STOP            停车（目标清零）
 *   ZERO            急停并清空所有控制器状态
 *   MODE=...        兼容旧写法，只吞掉不处理
 */

#define CHASSIS_COMMAND_BUFFER_SIZE  40U

typedef enum
{
  CHASSIS_CMD_VELOCITY = 0,
  CHASSIS_CMD_HEADING = 1
} ChassisCommandMode;

static int16_t s_command_linear_mm_s;
static int16_t s_command_angular_centirad_s;
static float s_command_heading_deg;
static ChassisCommandMode s_command_mode;
static bool s_command_valid;

/*
 * 指令自保持。默认开启：收到一条指令后由communicationTask持续重发，
 * 因此不会撞上CHASSIS_COMMAND_TIMEOUT_MS的500ms超时，可以安心调参。
 * 注意：IMU失效急停是独立于命令超时的另一道保护，开自保持不会削弱它。
 * 需要严格的"失联即停"语义时发送 HOLD=0。
 */
static bool s_command_hold = false;

static char Chassis_UpperAscii(char character)
{
  if ((character >= 'a') && (character <= 'z'))
  {
    return (char)(character - ('a' - 'A'));
  }
  return character;
}

static char *Chassis_FindCommandValue(char *command, const char *name)
{
  uint8_t index = 0U;

  while (name[index] != '\0')
  {
    if (Chassis_UpperAscii(command[index]) != Chassis_UpperAscii(name[index]))
    {
      return NULL;
    }
    index++;
  }

  if ((command[index] != '=') && (command[index] != ':'))
  {
    return NULL;
  }
  return &command[index + 1U];
}

/* 匹配行首关键字，后面可以是结尾/赋值号/空白，用于STOP这类无参指令。 */
static bool Chassis_CommandMatches(char *command, const char *name)
{
  uint8_t index = 0U;
  char tail;

  while (name[index] != '\0')
  {
    if (Chassis_UpperAscii(command[index]) != Chassis_UpperAscii(name[index]))
    {
      return false;
    }
    index++;
  }

  tail = command[index];
  return ((tail == '\0') || (tail == '=') || (tail == ':') ||
          (tail == ' ') || (tail == '\t'));
}

static bool Chassis_ReadFloatCommand(char *command, const char *name, float *value)
{
  char *value_text = Chassis_FindCommandValue(command, name);
  char *end;
  float parsed;

  if (value_text == NULL)
  {
    return false;
  }

  parsed = strtof(value_text, &end);
  if ((end == value_text) || (parsed != parsed) ||
      (parsed > 1.0e6f) || (parsed < -1.0e6f))
  {
    return false;
  }

  while ((*end == ' ') || (*end == '\t'))
  {
    end++;
  }
  if (*end != '\0')
  {
    return false;
  }

  *value = parsed;
  return true;
}

/* 读取 KP1..KP4 这类带序号的关键字，避免引入snprintf。 */
static bool Chassis_ReadIndexedCommand(char *command, const char *base,
                                       float *value, uint8_t *wheel)
{
  char key[8];
  uint8_t base_length = 0U;
  uint8_t index;

  while ((base[base_length] != '\0') && (base_length < 4U))
  {
    key[base_length] = base[base_length];
    base_length++;
  }

  for (index = 1U; index <= CHASSIS_WHEEL_COUNT; ++index)
  {
    key[base_length] = (char)('0' + (char)index);
    key[base_length + 1U] = '\0';
    if (Chassis_ReadFloatCommand(command, key, value))
    {
      *wheel = (uint8_t)(index - 1U);
      return true;
    }
  }
  return false;
}

static int16_t Chassis_ClampCommandSpeed(float value)
{
  if (value > (float)CHASSIS_MAX_LINEAR_SPEED_MM_S)
  {
    return (int16_t)CHASSIS_MAX_LINEAR_SPEED_MM_S;
  }
  if (value < -(float)CHASSIS_MAX_LINEAR_SPEED_MM_S)
  {
    return (int16_t)(-CHASSIS_MAX_LINEAR_SPEED_MM_S);
  }
  return (int16_t)value;
}

static int16_t Chassis_ClampCommandAngular(float value)
{
  if (value > 1000.0f)
  {
    return 1000;
  }
  if (value < -1000.0f)
  {
    return -1000;
  }
  return (int16_t)value;
}

static void Chassis_ApplyVelocityCommand(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!Chassis_ImuReady())
  {
    Chassis_StopWithReason(CHASSIS_STOP_IMU);
    if (primask == 0U) __enable_irq();
    return;
  }

  s_command_mode = CHASSIS_CMD_VELOCITY;
  s_command_valid = true;
  Chassis_SetVelocity(s_command_linear_mm_s, s_command_angular_centirad_s);
  if (primask == 0U) __enable_irq();
}

/* 增益统一按 [0, maximum] 钳位。 */
static float Chassis_ClampGain(float value, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < 0.0f)
  {
    return 0.0f;
  }
  return value;
}

static void Chassis_SetOneGain(uint32_t wheel, uint32_t which, float value)
{
  float kp;
  float ki;
  float kd;

  if (wheel >= CHASSIS_WHEEL_COUNT)
  {
    return;
  }

  SpeedPID_GetGains(wheel, &kp, &ki, &kd);

  if (which == 0U)
  {
    kp = Chassis_ClampGain(value, CHASSIS_SPEED_KP_MAX);
  }
  else if (which == 1U)
  {
    ki = Chassis_ClampGain(value, CHASSIS_SPEED_KI_MAX);
  }
  else
  {
    kd = Chassis_ClampGain(value, CHASSIS_SPEED_KD_MAX);
  }

  /* 注意：SpeedPID_SetGains会清该轮积分，改增益后有一次短暂重建，属预期。 */
  SpeedPID_SetGains(wheel, kp, ki, kd);
}

/*
 * 航向环（外环）只有一个控制器，没有分轮概念。
 * 改增益保留积分；HeadingPID_Reset则明确清除integral_dps，二者不同。
 */
static void Chassis_SetHeadingGain(uint32_t which, float value)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (which == 0U)
  {
    s_chassis.heading_pid.kp = Chassis_ClampGain(value, CHASSIS_HEADING_KP_MAX);
  }
  else if (which == 1U)
  {
    s_chassis.heading_pid.ki = Chassis_ClampGain(value, CHASSIS_HEADING_KI_MAX);
  }
  else
  {
    s_chassis.heading_pid.kd = Chassis_ClampGain(value, CHASSIS_HEADING_KD_MAX);
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void Chassis_SetAllGains(uint32_t which, float value)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
  {
    Chassis_SetOneGain(wheel, which, value);
  }
  if (primask == 0U) __enable_irq();
}

/*
 * 给定整形与静摩擦前馈的在线整定（家用平稳性）。
 * which: 0=RESP 1=LACC 2=LJRK 3=DACC 4=DJRK 5=FF
 * 全部做范围钳位：误输入会直接毁掉整形（例如 RESP=0 会让 ωn 变成无穷大）。
 */
static void Chassis_SetShapeParameter(uint32_t which, float value)
{
  uint32_t primask = __get_PRIMASK();

  /* 0不能表示“关闭限幅”：会让规划无法执行，包括停车。拒绝非正约束。 */
  if (which >= 1U && which <= 4U && value < 1.0f) return;

  __disable_irq();

  if (which == 0U)
  {
    if (value < CHASSIS_SHAPE_RESPONSE_MIN_S)
    {
      value = CHASSIS_SHAPE_RESPONSE_MIN_S;
    }
    else if (value > CHASSIS_SHAPE_RESPONSE_MAX_S)
    {
      value = CHASSIS_SHAPE_RESPONSE_MAX_S;
    }
    s_shape_response_s = value;
  }
  else if (which == 1U)
  {
    s_shape_linear_accel = Chassis_ClampGain(value, CHASSIS_SHAPE_ACCEL_MAX);
  }
  else if (which == 2U)
  {
    s_shape_linear_jerk = Chassis_ClampGain(value, CHASSIS_SHAPE_JERK_MAX);
  }
  else if (which == 3U)
  {
    s_shape_diff_accel = Chassis_ClampGain(value, CHASSIS_SHAPE_ACCEL_MAX);
  }
  else if (which == 4U)
  {
    s_shape_diff_jerk = Chassis_ClampGain(value, CHASSIS_SHAPE_JERK_MAX);
  }
  else
  {
    s_shape_friction_ff = Chassis_ClampGain(value, CHASSIS_FRICTION_FF_MAX);
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

/*
 * 标定量：轮径（which=0）与有效轮距（which=1）。
 *
 * 做成运行时可改，是为了让标定能"边测边调"——这两个值每改一次都要重新编译
 * 烧写的话，反复试值会非常痛苦。
 *
 * 范围钳位防止误输入把运动学搞崩（例如轮径填 0 会让 mm/s→RPM 除零）。
 */
#define CHASSIS_WHEEL_DIAMETER_MIN_MM   20.0f
#define CHASSIS_WHEEL_DIAMETER_MAX_MM   200.0f
#define CHASSIS_TRACK_WIDTH_MIN_MM      50.0f
#define CHASSIS_TRACK_WIDTH_MAX_MM      600.0f

static void Chassis_SetGeometryParameter(uint32_t which, float value)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  if (which == 0U)
  {
    if (value < CHASSIS_WHEEL_DIAMETER_MIN_MM)
    {
      value = CHASSIS_WHEEL_DIAMETER_MIN_MM;
    }
    else if (value > CHASSIS_WHEEL_DIAMETER_MAX_MM)
    {
      value = CHASSIS_WHEEL_DIAMETER_MAX_MM;
    }
    s_wheel_diameter_mm = value;
  }
  else
  {
    if (value < CHASSIS_TRACK_WIDTH_MIN_MM)
    {
      value = CHASSIS_TRACK_WIDTH_MIN_MM;
    }
    else if (value > CHASSIS_TRACK_WIDTH_MAX_MM)
    {
      value = CHASSIS_TRACK_WIDTH_MAX_MM;
    }
    s_track_width_mm = value;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 遥测周期，TRATE=<ms>。范围钳位：低于 20 ms 会把两路 UART 都压到接近饱和。 */
static void Chassis_SetTelemetryPeriod(float value)
{
  uint32_t primask;

  if (value < (float)CHASSIS_TELEMETRY_PERIOD_MIN_MS)
  {
    value = (float)CHASSIS_TELEMETRY_PERIOD_MIN_MS;
  }
  else if (value > (float)CHASSIS_TELEMETRY_PERIOD_MAX_MS)
  {
    value = (float)CHASSIS_TELEMETRY_PERIOD_MAX_MS;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  s_telemetry_period_ms = (uint32_t)value;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void Chassis_CancelMotionIntent(bool invalidate_partial)
{
  s_command_valid = false;
  s_command_linear_mm_s = 0;
  s_command_angular_centirad_s = 0;
  s_command_heading_deg = 0;
  if (invalidate_partial) ++s_motion_epoch;
}

static void Chassis_ApplyCommand(char *command, uint32_t epoch)
{
  float value = 0.0f;
  uint8_t wheel = 0U;

  if (epoch != s_motion_epoch &&
      !Chassis_CommandMatches(command, "STOP") && !Chassis_CommandMatches(command, "ZERO")) return;
  if (s_aux_command_handler != NULL && s_aux_command_handler(command)) return;

  if (Chassis_FindCommandValue(command, "MODE") != NULL)
  {
    return; /* 兼容旧写法，本工程模式由命令本身决定。 */
  }

  if (Chassis_ReadFloatCommand(command, "V", &value))
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (epoch == s_motion_epoch) {
      s_command_linear_mm_s = Chassis_ClampCommandSpeed(value);
      Chassis_ApplyVelocityCommand();
    }
    if (primask == 0U) __enable_irq();
    return;
  }
  if (Chassis_ReadFloatCommand(command, "W", &value))
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (epoch == s_motion_epoch) {
      s_command_angular_centirad_s = Chassis_ClampCommandAngular(value);
      Chassis_ApplyVelocityCommand();
    }
    if (primask == 0U) __enable_irq();
    return;
  }
  if (Chassis_ReadFloatCommand(command, "SPEED", &value))
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (epoch == s_motion_epoch) {
      s_command_linear_mm_s = Chassis_ClampCommandSpeed(value);
      s_command_angular_centirad_s = 0;
      Chassis_ApplyVelocityCommand();
    }
    if (primask == 0U) __enable_irq();
    return;
  }
  if (Chassis_ReadFloatCommand(command, "HEAD", &value))
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (epoch != s_motion_epoch) {
      if (primask == 0U) __enable_irq();
      return;
    }

    s_command_heading_deg = value;
    s_command_mode = CHASSIS_CMD_HEADING;
    s_command_valid = true;
    Chassis_SetHeading(s_command_linear_mm_s, value);
    if (primask == 0U) __enable_irq();
    return;
  }

  if (Chassis_ReadFloatCommand(command, "KP", &value))
  {
    Chassis_SetAllGains(0U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "KI", &value))
  {
    Chassis_SetAllGains(1U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "KD", &value))
  {
    Chassis_SetAllGains(2U, value);
    return;
  }
  if (Chassis_ReadIndexedCommand(command, "KP", &value, &wheel))
  {
    Chassis_SetOneGain(wheel, 0U, value);
    return;
  }
  if (Chassis_ReadIndexedCommand(command, "KI", &value, &wheel))
  {
    Chassis_SetOneGain(wheel, 1U, value);
    return;
  }
  if (Chassis_ReadIndexedCommand(command, "KD", &value, &wheel))
  {
    Chassis_SetOneGain(wheel, 2U, value);
    return;
  }

  /*
   * 航向环（外环）增益。外环只有一个控制器，所以没有 KP1~KP4 那样的序号形式。
   * 同时接受 PKP/PKI/PKD 作为别名——那是原520工程位置环的命名，
   * 如果你VOFA里已经有按这个命名的控件，不用改动就能直接复用。
   */
  if (Chassis_ReadFloatCommand(command, "HKP", &value) ||
      Chassis_ReadFloatCommand(command, "PKP", &value))
  {
    Chassis_SetHeadingGain(0U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "HKI", &value) ||
      Chassis_ReadFloatCommand(command, "PKI", &value))
  {
    Chassis_SetHeadingGain(1U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "HKD", &value) ||
      Chassis_ReadFloatCommand(command, "PKD", &value))
  {
    Chassis_SetHeadingGain(2U, value);
    return;
  }

  /*
   * 给定整形与静摩擦前馈（家用平稳性）。名字不用单字母，避免和上面的
   * V/W/KP 等冲突，也方便在VOFA文本框里一眼看清改的是哪一项。
   */
  if (Chassis_ReadFloatCommand(command, "RESP", &value))
  {
    Chassis_SetShapeParameter(0U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "LACC", &value))
  {
    Chassis_SetShapeParameter(1U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "LJRK", &value))
  {
    Chassis_SetShapeParameter(2U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "DACC", &value))
  {
    Chassis_SetShapeParameter(3U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "DJRK", &value))
  {
    Chassis_SetShapeParameter(4U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "FF", &value))
  {
    Chassis_SetShapeParameter(5U, value);
    return;
  }

  /* 标定量：在线改轮径 / 有效轮距，省得标定时反复编译烧写。
   * 例：DIAM=65.5   TRACK=250 */
  if (Chassis_ReadFloatCommand(command, "DIAM", &value))
  {
    Chassis_SetGeometryParameter(0U, value);
    return;
  }
  if (Chassis_ReadFloatCommand(command, "TRACK", &value))
  {
    Chassis_SetGeometryParameter(1U, value);
    return;
  }

  /* 遥测周期，TRATE=<ms>。100 = 无线稳态；20~30 = 精细波形分析（请插 USB）。 */
  if (Chassis_ReadFloatCommand(command, "TRATE", &value))
  {
    Chassis_SetTelemetryPeriod(value);
    return;
  }

  if (Chassis_ReadFloatCommand(command, "HOLD", &value))
  {
    s_command_hold = (value != 0.0f);
    return;
  }

  if (Chassis_CommandMatches(command, "STOP"))
  {
    Chassis_CancelMotionIntent(true);
    s_command_linear_mm_s = 0;
    s_command_angular_centirad_s = 0;
    Chassis_ApplyVelocityCommand();
    return;
  }
  if (Chassis_CommandMatches(command, "ZERO"))
  {
    Chassis_CancelMotionIntent(true);
    s_command_valid = false;
    s_command_linear_mm_s = 0;
    s_command_angular_centirad_s = 0;
    Chassis_EmergencyStop();
  }
}

/*
 * 指令空闲超时。
 *
 * 正常上位机会在指令末尾追加 \r 或 \n，那就靠终止符结算。但有些发送框
 * （例如 VOFA+ 的"无追加"档，而且那个下拉控件还可能点不动）什么都不追加，
 * 指令就会永远攒在缓冲里等不到终止符 —— 表现为"发指令车不动"。
 *
 * 所以补一条兜底：收到最后一个字节之后静默这么久，就把已攒的内容当成
 * 一条完整指令执行。正常带换行的指令先被终止符结算掉，不受影响。
 *
 * 取值权衡：太短会把"被链路拆开的半条指令"提前执行（例如 V=200 被拆成
 * V=2 + 00，会先按 V=2 执行）；太长则手敲指令有明显延迟。150 ms 对敲命令
 * 感觉不出来，又能容忍 ESP 转发时的短暂停顿。
 */
#define CHASSIS_COMMAND_IDLE_MS         150U

void Chassis_ProcessCommands(void)
{
  typedef struct {
    char command[CHASSIS_COMMAND_BUFFER_SIZE];
    uint8_t length, discard;
    uint32_t last_byte_tick, epoch;
  } Parser;
  static Parser parser[VOFA_PORT_COUNT];
  uint8_t byte, port, flags;
  bool received[VOFA_PORT_COUNT] = {false};
  /* 轮询有界，串口持续输入也不能占住 communicationTask。 */
  for (unsigned n = 0; n < 256U && VofaPort_ReadEvent(&port, &byte, &flags); ++n) {
    if (port >= VOFA_PORT_COUNT) continue;
    Parser *p = &parser[port];
    received[port] = true;
    p->last_byte_tick = HAL_GetTick();
    if ((flags & VOFA_RX_EVENT_RESET) != 0U) {
      p->length = 0U; p->discard = 1U;
      continue;
    }
    if ((flags & VOFA_RX_EVENT_FLUSH) != 0U) {
      p->length = 0U; /* 保留溢出导致的discard，只有换行可以解除。 */
      continue;
    }
    if (byte == '\r' || byte == '\n') {
      if (p->length != 0U && p->discard == 0U) {
        p->command[p->length] = '\0';
        Chassis_ApplyCommand(p->command, p->epoch);
      }
      p->length = 0U; p->discard = 0U;
    } else if (!p->discard) {
      if (p->length == 0U) p->epoch = s_motion_epoch;
      if (p->length < CHASSIS_COMMAND_BUFFER_SIZE - 1U)
        p->command[p->length++] = (char)byte;
      else { p->length = 0U; p->discard = 1U; }
    }
  }
  for (port = 0; port < VOFA_PORT_COUNT; ++port) {
    Parser *p = &parser[port];
    if (!received[port] && !p->discard && p->length != 0U &&
        HAL_GetTick() - p->last_byte_tick >= CHASSIS_COMMAND_IDLE_MS) {
      p->command[p->length] = '\0';
      Chassis_ApplyCommand(p->command, p->epoch);
      p->length = 0U;
    }
  }
}

void Chassis_RefreshCommand(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!s_command_valid || !s_command_hold)
  {
    if (primask == 0U) __enable_irq();
    return;
  }

  if (s_command_mode == CHASSIS_CMD_HEADING)
  {
    Chassis_SetHeading(s_command_linear_mm_s, s_command_heading_deg);
  }
  else
  {
    Chassis_SetVelocity(s_command_linear_mm_s, s_command_angular_centirad_s);
  }
  if (primask == 0U) __enable_irq();
}
