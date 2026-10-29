/*
 * speed_pid.c — 四个轮子各自独立的速度环 PI(D) 控制器
 *
 * ============================ 它在这条链路里的位置 ============================
 *
 *   communicationTask            下发目标（mm/s 或串口指令）
 *          ↓
 *   Chassis_SetVelocity()        只把目标记到 s_chassis，不动电机
 *          ↓
 *   Chassis_ControlStep()        每 10 ms 跑一次（TIM7 唤醒 controlTask）
 *          ├─ 把 mm/s 换算成 RPM，并做斜坡限幅 → reference_rpm
 *          ├─ SpeedPID_Calculate()   ← 本文件：RPM 误差 → 输出百分比
 *          └─ Chassis_PercentToPwm() 把百分比换算成 PWM 计数（±8399）
 *          ↓
 *   MotorDriver_SetSignedPwm()   写 CCR + 两个方向电平脚 → 电机
 *
 *
 * ============================== 量纲约定 ==============================
 * 读这个文件最容易搞错的就是单位，先把三者的量纲钉死：
 *
 *   measured / target : RPM（转/分钟），来自编码器实测与斜坡后的给定
 *   返回值 output     : 百分比 %，范围 ±output_limit（默认 ±100）
 *                       正数 = 前进，负数 = 后退
 *                       由 chassis_control.c 的 Chassis_PercentToPwm()
 *                       乘上 CHASSIS_PWM_PER_PERCENT(= 8399/100) 变成PWM计数
 *
 * ============================== 调用周期 ==============================
 * 由 TIM7 每 **10 ms** 调用一次。这个周期直接决定了 Ki/Kd 的实际大小：
 *
 *   积分：integral += error         ← 注意**没有乘 dt**
 *         每 10 ms 累加一次 → 每秒累加 100 次
 *         等效积分增益 = ki / 0.01 = **ki × 100**（每秒）
 *         ki=0.04 实际相当于 4 /s
 *
 *   微分：kd * (error - previous_error)   ← 一阶差分，也**没有除 dt**
 *         差分间隔就是一个调用周期 0.01 s
 *         等效微分增益 = kd × 0.01 = **kd / 100**
 *
 *   ⚠️ 两者方向相反，而且都强依赖调用周期。
 *      如果以后改了控制周期（例如改成 5 ms），必须按比例重算 Ki/Kd，
 *      否则积分作用会悄悄变成两倍/一半，而代码看起来毫无变化。
 *
 *   （对比：heading_pid.c 的外环用的是 error * dt_s，是"真实秒单位"的积分，
 *     所以那两个文件的 Ki 数值不能互相套用，含义不同。）
 *
 * ================================ 算法 ================================
 * 位置式 PI(D)，带**条件积分**抗饱和：
 *
 *   error  = target - measured
 *   output = FF + Kp·error + Ki·Σerror + Kd·(error - error_prev)
 *
 * "条件积分"的意思是：不是一饱和就停止积分，而是只在
 * **"接受了这次积分会更饱和，而且误差还在往同方向推"** 时才拒绝累加。
 * 这样既防积分饱和，又允许积分在误差反向时正常退饱和。
 *
 * 状态是"每个轮子一份"（s_controller[4]），四路互不干扰。
 *
 * ============================== 默认增益 ==============================
 * 沿用已调好的 520 电机参考工程：Kp=0.5、Ki=0.04、Kd=0。
 * 可通过串口指令在线修改（见 chassis_control.c 的 KP=/KI=/KD=/KP1..4=）。
 */

#include "speed_pid.h"
#include "stm32f4xx_hal.h"

#include <stddef.h> /* 提供 NULL，供 SpeedPID_GetGains 的可选输出参数使用 */

/* ============================ 编译期默认参数 ============================ */

/* 比例增益。误差单位 RPM，输出单位 %，所以量纲是 %/RPM。
 * 0.5 表示"误差 1 RPM 就输出 0.5%"（对应 0.5% × 83.99 ≈ 42 个 PWM 计数）。 */
#define SPEED_PID_DEFAULT_KP     0.5f
/* 积分增益（每次调用累加一次误差，未乘 dt，等效 ki×100 /s，见文件头说明）。 */
#define SPEED_PID_DEFAULT_KI     0.04f
/* 微分增益。默认 0 = 微分项完全不生效；改大可以抑制超调，
 * 但一阶差分对反馈噪声很敏感（编码器量化台阶会被放大）。 */
#define SPEED_PID_DEFAULT_KD     0.0f
/* 输出限幅，单位 %。100 表示允许输出到 ±100% 占空比。
 * 换算到 PWM 计数就是 ±8399（= TIM3 的 ARR）。 */
#define SPEED_PID_OUTPUT_LIMIT   100.0f

/* ============================== 内部状态 ============================== */

/* 四个电机分别保存独立的 PI/PID 状态（增益 + 积分 + 上次误差 + 上次输出）。
 * 数组下标 = 轮子序号：0=A左前、1=B左后、2=C右前、3=D右后，
 * 与 encoder.c 的轮序、motor_driver.c 的 A/B/C/D 一一对应。
 * static 表示外部只能通过下面这些函数访问，不能直接改。 */
static SpeedPID_Controller s_controller[SPEED_PID_WHEEL_COUNT];
static float s_requested_gains[SPEED_PID_WHEEL_COUNT][3];
static volatile uint32_t s_pending_gains;

/* ============================== 内部工具 ============================== */

/**
 * @brief  把浮点数限制在 [minimum, maximum] 区间内。
 * @param  value    待限制的值。
 * @param  minimum  下限（必须 <= maximum）。
 * @param  maximum  上限。
 * @retval 限制后的值。value 在区间内则原样返回。
 * @note   本文件自用的小工具，为什么不直接用 fminf/fmaxf：
 *         避免为两个比较操作引入 libm 依赖。static 不外露。
 */
static float SpeedPID_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}

/* ============================== 公开接口 ============================== */

/**
 * @brief  把所有速度环的增益恢复成编译期默认值，并清零全部内部状态。
 * @param  无
 * @retval 无
 * @note   由 Chassis_Init() 在调度器启动后调用一次。
 *         每次调用都会覆盖掉通过串口在线改过的增益，所以运行中途不要调用。
 */
void SpeedPID_Init(void)
{
  s_pending_gains = 0U;
  for (uint32_t wheel = 0U; wheel < SPEED_PID_WHEEL_COUNT; ++wheel)
  {
    s_controller[wheel].kp = SPEED_PID_DEFAULT_KP;
    s_controller[wheel].ki = SPEED_PID_DEFAULT_KI;
    s_controller[wheel].kd = SPEED_PID_DEFAULT_KD;
    s_controller[wheel].output_limit = SPEED_PID_OUTPUT_LIMIT;
    SpeedPID_Reset(wheel); /* 清积分、清上次误差、清上次输出 */
  }
}

/**
 * @brief  清除**某一个**轮子的控制器历史状态（增益不动）。
 * @param  wheel  轮子序号 0~3。越界时什么都不做（不报错、不写内存）。
 * @retval 无
 * @note   清掉的是三样东西：
 *           - integral      积分累加值（防止残留积分把轮子顶住）
 *           - previous_error 上次误差（防止下一次算微分时产生跳变）
 *           - output        上次输出（仅用于记录，不参与计算）
 *         增益 kp/ki/kd/output_limit **不受影响**。
 *
 *         chassis_control.c 在四种"历史不再适用"的场合调用它：
 *           1. 收到停车命令；
 *           2. 目标速度方向反转（比如 +500 直接跳到 −500）；
 *           3. 斜坡给定实际穿过零点；
 *           4. 急停/故障。
 *         SpeedPID_ApplyPendingGains() 在控制周期边界提交增益时也会调用它。
 */
void SpeedPID_Reset(uint32_t wheel)
{
  if (wheel < SPEED_PID_WHEEL_COUNT)
  {
    s_controller[wheel].previous_error = 0.0f;
    s_controller[wheel].integral = 0.0f;
    s_controller[wheel].output = 0.0f;
  }
}

/**
 * @brief  清除**全部四个**轮子的控制器历史状态（等价于对 0~3 各调一次 Reset）。
 * @param  无
 * @retval 无
 * @note   由 Chassis_EmergencyStop() 调用，保证急停后四个环都是干净的。
 */
void SpeedPID_ResetAll(void)
{
  for (uint32_t wheel = 0U; wheel < SPEED_PID_WHEEL_COUNT; ++wheel)
  {
    SpeedPID_Reset(wheel);
  }
}

/**
 * @brief  缓存某个轮子的PID增益请求；控制任务在周期边界统一提交并清历史。
 * @param  wheel  轮子序号 0~3。越界时什么都不做。
 * @param  kp     比例增益，单位 %/RPM。**调用方负责钳位**，本函数不检查范围。
 * @param  ki     积分增益（每次累加，未乘 dt）。
 * @param  kd     微分增益（一阶差分，未除 dt）。
 * @retval 无
 * @note   ⚠️ 提交时调用 SpeedPID_Reset()，所以**改增益后积分会归零**，
 *         输出会短暂由纯 P 项主导，约几十毫秒内重新建立积分。
 *         这是有意的（换一套增益后旧积分不再适用），但在线调参时要知道，
 *         否则会误以为"一改增益车就顿一下"是 bug。
 *
 *         增益的上下限钳位在 chassis_control.c 的 Chassis_ClampGain() 里做
 *         （速度环上限 Kp≤10、Ki≤1、Kd≤10，下限 0）。
 */
void SpeedPID_SetGains(uint32_t wheel, float kp, float ki, float kd)
{
  if (wheel < SPEED_PID_WHEEL_COUNT)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_requested_gains[wheel][0] = kp;
    s_requested_gains[wheel][1] = ki;
    s_requested_gains[wheel][2] = kd;
    s_pending_gains |= 1UL << wheel;
    if (primask == 0U) __enable_irq();
  }
}

/**
 * @brief  读出某一个轮子当前的三个增益。
 * @param  wheel  轮子序号 0~3。越界时直接返回，不写任何输出参数。
 * @param  kp     [out] 比例增益；**传 NULL 表示不读这一项**。
 * @param  ki     [out] 积分增益；传 NULL 表示不读。
 * @param  kd     [out] 微分增益；传 NULL 表示不读。
 * @retval 无
 * @note   存在的原因：串口在线调参时常常只想改一个增益（比如 `KP1=0.8`），
 *         这时必须先把另外两个读出来、再一起写回去，否则会被覆盖成 0。
 *         chassis_control.c 的 Chassis_SetOneGain() 就是这么用的。
 */
void SpeedPID_GetGains(uint32_t wheel, float *kp, float *ki, float *kd)
{
  if (wheel >= SPEED_PID_WHEEL_COUNT)
  {
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool pending = (s_pending_gains & (1UL << wheel)) != 0U;
  if (kp != NULL)
  {
    *kp = pending ? s_requested_gains[wheel][0] : s_controller[wheel].kp;
  }
  if (ki != NULL)
  {
    *ki = pending ? s_requested_gains[wheel][1] : s_controller[wheel].ki;
  }
  if (kd != NULL)
  {
    *kd = pending ? s_requested_gains[wheel][2] : s_controller[wheel].kd;
  }
  if (primask == 0U) __enable_irq();
}

void SpeedPID_ApplyPendingGains(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t wheel = 0; wheel < SPEED_PID_WHEEL_COUNT; ++wheel) {
    if ((s_pending_gains & (1UL << wheel)) == 0U) continue;
    s_controller[wheel].kp = s_requested_gains[wheel][0];
    s_controller[wheel].ki = s_requested_gains[wheel][1];
    s_controller[wheel].kd = s_requested_gains[wheel][2];
    SpeedPID_Reset(wheel);
  }
  s_pending_gains = 0U;
  if (primask == 0U) __enable_irq();
}

/**
 * @brief  计算某一个轮子的速度环输出（位置式 PI(D)，带条件积分抗饱和）。
 *
 * @param  wheel            轮子序号 0~3（0=A左前 1=B左后 2=C右前 3=D右后）。
 *                          **越界时直接返回 0.0f**，不写任何内部状态。
 * @param  measured         实测转速，单位 **RPM**（来自编码器，已经过一阶低通）。
 * @param  target           目标转速，单位 **RPM**。
 *                          注意传进来的是 chassis_control.c 里**斜坡限幅之后**的
 *                          reference_rpm（联合加速度/jerk约束后），不是原始目标，
 *                          所以这里的阶跃冲击已经被上游软化过了。
 * @param  integral_enable  是否允许积分：
 *                            true  = 正常累加积分；
 *                            false = **把积分清零**（不是冻结！），
 *                                    本次及以后输出退化成纯 P(+D)。
 *                          调用方在"目标接近零 / 正在停车"时给 false，
 *                          避免停车过程里残留的积分把轮子顶住不放。
 *
 * @return 输出百分比，范围 ±output_limit（默认 ±100）。
 *         正=前进，负=后退。调用方用 Chassis_PercentToPwm() 换算成 PWM 计数。
 *
 * @note   三个实现细节，读代码时值得逐个想清楚：
 *
 *   1) previous_error 是**无条件更新**的（在积分分支之前），
 *      所以即使 integral_enable=false，微分项也不会在恢复积分时产生跳变。
 *
 *   2) 抗饱和用的是"候选积分"试算法：先假设接受这次积分，算出 candidate_output，
 *      只有在 **"已经饱和 且 误差还在把输出往同方向推"** 时才拒绝累加。
 *      误差反向时照常累加，积分才能正常退饱和。
 *
 *   3) 积分累加的是 error 而不是 error*dt（见文件头的"调用周期"一节），
 *      所以 Ki 的实际强弱由 TIM7 的 10 ms 周期决定。
 */
float SpeedPID_Calculate(uint32_t wheel, float measured, float target,
                         bool integral_enable)
{
  return SpeedPID_CalculateWithFeedforward(wheel, measured, target, integral_enable, 0.0f);
}

float SpeedPID_CalculateWithFeedforward(uint32_t wheel, float measured, float target,
                                        bool integral_enable, float feedforward)
{
  SpeedPID_Controller *controller;
  float error;  //误差
  float proportional; //比例项
  float derivative; //微分项
  float candidate_integral; //候选积分
  float candidate_output; //候选输出

  if (wheel >= SPEED_PID_WHEEL_COUNT)
  {
    return 0.0f;
  }

  controller = &s_controller[wheel];
  SpeedPID_ApplyPendingGains();

  /* --- 误差与 P、D 项 --------------------------------------------------- */
  error = target - measured;                       /* 正误差 = 转得太慢，要加速 */
  proportional = controller->kp * error;
  /* D 项用"误差的一阶差分"（误差微分），间隔就是一个调用周期 10 ms。
   * 默认 kd=0 时这一项恒为 0。若调大要小心：编码器量化台阶会被直接放大。 */
  derivative = controller->kd * (error - controller->previous_error);
  controller->previous_error = error;              /* 无条件更新，见 @note 1 */

  /* --- I 项：条件积分 --------------------------------------------------- */
  if ((controller->ki != 0.0f) && integral_enable)
  {
    /* 先假装接受这次积分，算出"如果接受了会是多少输出" */
    candidate_integral = controller->integral + error;
    candidate_output = feedforward + proportional +
                       controller->ki * candidate_integral +
                       derivative;

    /* 输出饱和且误差仍推动输出继续饱和时，暂停积分（拒绝这次累加，
     * 保留旧的 integral）。反之——包括"误差反向、有助于退饱和"的情况——
     * 就接受这次累加。 */
    if (!((candidate_output > controller->output_limit && error > 0.0f) ||
          (candidate_output < -controller->output_limit && error < 0.0f)))
    {
      controller->integral = candidate_integral;
    }
  }
  else
  {
    /* 两种情况走到这里：
     *   - 调用方明确要求关闭积分（integral_enable=false）→ 清零；
     *   - ki 被设成了 0 → 顺带把可能残留的积分也清掉，保持状态干净。 */
    controller->integral = 0.0f;
  }

  /* --- 合成输出并限幅 --------------------------------------------------- */
  controller->output = feedforward + proportional +
                       controller->ki * controller->integral +
                       derivative;
  controller->output = SpeedPID_Clamp(controller->output,
                                      -controller->output_limit,
                                      controller->output_limit);
  return controller->output;
}
