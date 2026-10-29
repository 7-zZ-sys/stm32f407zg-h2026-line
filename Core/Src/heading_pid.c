/*
 * heading_pid.c — 航向外环（角度环）PID
 *
 * ============================ 它在这条链路里的位置 ============================
 *
 *   目标航向角 ──┐
 *                ├─ HeadingPID_Calculate()  ← 本文件
 *   实测航向角 ──┘        ↓  输出"目标角速度"(deg/s)
 *                    左右轮差速分配（chassis_control.c）
 *                         ↓
 *                    四个速度内环（speed_pid.c，每轮一份）
 *                         ↓
 *                        PWM
 *
 * 这是**串级控制的外环**：它不直接控制电机，而是给内环下"转速差"的指令。
 * 外环每 20 ms 跑一次（两个控制周期），内环每 10 ms 跑一次。
 *
 * ============================== 量纲约定 ==============================
 *   输入：角度 deg、角速度 deg/s、时间 s
 *   输出：角速度 deg/s（正值 = 左转 / 逆时针）
 *   ⚠️ 和 speed_pid.c 的 RPM / % 完全不是一套，两者的 Ki 数值不能互相套用。
 *
 * ========================= 和 speed_pid.c 的三处关键差异 =========================
 *
 *   1) 积分用**真实秒**：`Ki · error · dt_s`
 *      而速度环是 `integral += error`（没乘 dt，强依赖 10 ms 周期）。
 *      所以两个文件里 Ki=0.04 的实际强弱完全不同。
 *
 *   2) 微分项**不是对误差做数值差分**，而是用陀螺仪实测角速度：
 *      `Kd · (feedforward_dps − measured_rate_dps)`。见 Calculate 的 @note。
 *
 *   3) 多了一个**前馈**输入：期望的转向速率直接加到输出上，PI 只负责修正残差。
 *      而且前馈和反馈**共用同一个限幅额度**，这是最容易踩的设计约束，
 *      详见 heading_pid.h 里 HEADING_MAX_RATE_DPS 的说明。
 *
 * 默认增益（heading_pid.h）：Kp=1.5、Ki=0、Kd=0.04。
 * 因为 Ki=0，默认实际是个 **PD 控制器**（Calculate 里会强制把积分清零）。
 */

#include "heading_pid.h"

#include <math.h> /* fmodf / isfinite */

/**
 * @brief  把浮点数限制在 [-limit, +limit] 内。
 * @param  value  待限制的值。
 * @param  limit  单侧限幅（必须为正）。
 * @retval 限制后的值。
 * @note   本文件自用。注意它**只有一个 limit 参数**、默认对称限幅，
 *         和 speed_pid.c 里带上下限的 SpeedPID_Clamp() 签名不同。
 */
static float Clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

/**
 * @brief  把任意角度归一到 **[-180, +180)** 区间（注意是左闭右开）。
 *
 * @param  degrees  任意角度，单位 deg。可以是一圈以外的累加值（如 540、-358）。
 * @return 与之等效、且落在 [-180, +180) 内的角度。
 *         **非有限值（NaN / ±Inf）返回 0.0f**，不会把坏值传播出去。
 *
 * @note  为什么这个函数是航向控制的地基：**角度差必须走最短路径。**
 *
 *   假设实测 179°、目标 −179°。这两个方向其实只差 2°，但直接相减：
 *       target - measured = -179 - 179 = -358
 *   控制系统会以为要往一个方向转 358°，于是**命令车几乎转一整圈**。
 *   先归一化再算误差就没这个问题：WrapDegrees(-358) = +2 → 只转 2°。
 *
 * @note  实现是三步（可用下面的例子逐步验算）：
 *           1. `+180` 把目标区间 [-180, 180) 平移到 [0, 360)
 *           2. `fmodf(..., 360)` 取余，把任意圈数折到一圈以内。
 *              注意 C 的 fmodf **结果符号跟被除数一致**，所以可能是负数；
 *              而 if 分支里的 `+= 360` 就是把这个负数搬回 [0, 360)。
 *           3. `-180` 再平移回 [-180, 180)
 *
 *         为什么不直接用 `fmodf(degrees, 360)`：那样得到的是 (-360, 360)，
 *         不满足"最短路径"要求（179 和 −179 会得到 179 和 −179 两个不同结果）。
 *
 * @note  边界与特殊值（都是有意为之，不是巧合）：
 *
 *   | 输入   | 输出    | 说明 |
 *   |--------|---------|------|
 *   | 0      | 0       | |
 *   | 179    | 179     | |
 *   | **180**  | **−180** | 区间是左闭右开，"正好半圈"统一归到 −180 |
 *   | 181    | −179    | 绕过去了 |
 *   | 358    | −2      | 最短路径 |
 *   | **−358** | **+2**  | 上面的关键用例：目标 −179、实测 179 |
 *   | 540    | −180    | 一圈半 |
 *   | NaN/Inf| **0**   | 提前返回，不污染下游 |
 *
 * @note  本工程的全部调用点（都在用它解回绕）：
 *
 *   | 调用点 | 用途 |
 *   |---|---|
 *   | 本文件 HeadingPID_Calculate | 把角度差归一成最短转向方向 |
 *   | jy901s.c | 模块姿态减去主控参考后把 yaw 压回 ±180 |
 *   | chassis_control.c `Chassis_SetHeading()` | 归一外部传入的目标角 |
 *   | chassis_control.c 速度模式分支 | 目标角按前馈速率**持续累加**后归一——正因为有这一步，才能连续转过 360° 而不受 ±180 限制（路径测试画整圈就是靠它） |
 *   | chassis_control.c `Chassis_GetFeedback()` | 算出 `heading_error_deg` 发给 VOFA 第 13/14 通道 |
 *   | chassis_control.c 路径测试 | 用**逐次增量**累加实测转角来解回绕，比"终点减起点"正确 |
 */
float HeadingPID_WrapDegrees(float degrees)
{
  /* 先挡住 NaN / ±Inf：一旦漏出去，后面的 PID 会算出 NaN 并一路污染到 PWM。 */
  if (!isfinite(degrees)) return 0.0f;
  degrees = fmodf(degrees + 180.0f, 360.0f); /* → 一圈以内，可能为负 */
  if (degrees < 0.0f) degrees += 360.0f;     /* → [0, 360) */
  return degrees - 180.0f;                   /* → [-180, 180) */
}

/**
 * @brief  把一个航向 PID 的增益设为编译期默认值，并清积分。
 * @param  pid  控制器状态指针（**本函数不检查 NULL**）。
 * @retval 无
 * @note   由 Chassis_Init() 调用一次。会覆盖掉通过串口在线改过的增益。
 */
void HeadingPID_Init(HeadingPID *pid)
{
  pid->kp = HEADING_PID_DEFAULT_KP;
  pid->ki = HEADING_PID_DEFAULT_KI;
  pid->kd = HEADING_PID_DEFAULT_KD;
  HeadingPID_Reset(pid);
}

/**
 * @brief  清除航向 PID 的历史状态。
 * @param  pid  控制器状态指针（**本函数不检查 NULL**）。
 * @retval 无
 * @note   只清 `integral_dps` 一个字段——因为 D 项用的是陀螺仪实测值、
 *         不存 `previous_error`，所以这个控制器**只有积分一个历史量**。
 *         这和 speed_pid.c 要清三个字段不同。
 *         增益不受影响，所以改增益不清积分（见 heading_pid.h 的说明）。
 */
void HeadingPID_Reset(HeadingPID *pid)
{
  pid->integral_dps = 0.0f;
}

/**
 * @brief  计算航向外环输出（PD/PID，带前馈，带条件积分抗饱和）。
 *
 * @param  pid                控制器状态。**全工程只有一个**（不像速度环有 4 份），
 *                            传 `&s_chassis.heading_pid`。
 *                            ⚠️ 本函数**不检查 pid 是否为 NULL**。
 * @param  target_deg         目标航向角，deg，相对上电校准零点，正值=左转。
 *                            速度模式下是"按前馈速率持续累加"的目标；
 *                            航向模式（`HEAD=`）下是外部给的绝对目标。
 * @param  measured_deg       实测航向角，deg，来自 JY901S 的直接姿态角。
 * @param  measured_rate_dps  实测角速度，deg/s，来自陀螺仪（已零偏补偿 + 20 ms 低通）。
 *                            **D 项用的就是它**，不是对角度做差分。
 * @param  feedforward_dps    前馈角速度，deg/s，即"期望的转向速率"。
 *                            由 `Chassis_SetVelocity()` 的第二个参数换算而来；
 *                            `Chassis_SetHeading()` 会把它置 0。
 * @param  dt_s               距上次调用的时间，秒。**只用于积分项**（真实秒单位）。
 *                            当前由 chassis_control.c 传常量 0.020f。
 * @param  output_limit_dps   输出限幅，deg/s。由线速度算出的几何限幅再被
 *                            `HEADING_MAX_RATE_DPS` 封顶得到。
 *
 * @return 目标角速度，deg/s，范围 ±output_limit_dps。正值 = 左转。
 *         调用方把它换算成左右轮差速：
 *         `angular_component = 输出 × π/180 × 轮距 / 2`。
 *
 * @note  **【最重要】前馈和反馈共用同一个限幅额度。**
 *   最后一行是 `Clamp(base + 积分, ±output_limit_dps)`，也就是说
 *   **前馈 + Kp·误差 + Kd 项 + 积分 是打包在一起限幅的**。后果是：
 *
 *     如果想转得快（前馈大），前馈就会把额度吃光，留给纠偏的余量趋近于零，
 *     外环退化成开环 —— 航向误差会一直累积而环路追不回来。
 *
 *   这不是理论担心，2026-10-11 就撞上过：路径测试原本用 ω=0.5 rad/s=28.65 dps，
 *   而当时 `HEADING_MAX_RATE_DPS` 只有 30 dps，只剩 **1.35 dps** 余量，
 *   轨迹根本不可能闭合。把限幅提到 60 之后余量变成 31.35 dps 才跑通。
 *   **所以想提高转向速率，必须同时放大这个限幅。**
 *
 * @note  **D 项不是误差的微分，而是 `feedforward − 实测角速度`。**
 *
 *   数学上它是等价的：目标角按前馈速率前进时，
 *       d(误差)/dt = d(目标)/dt − d(实测)/dt ≈ feedforward − measured_rate
 *   所以这**确实**是误差的微分，只是换了获取方式。好处有两个：
 *
 *     - **没有微分冲击（derivative kick）**：若对误差做数值差分，
 *       `HEAD=90` 这种目标突变会让 D 项瞬间爆出一个巨大尖峰；
 *       用 `feedforward − rate` 就完全不会，因为两个量本身都是平滑的。
 *     - **噪声更小**：陀螺仪直接给 deg/s，比"差分角度估计值"干净得多。
 *
 *   但要注意语义会随模式变化：`HEAD=` 模式下 `Chassis_SetHeading()` 把
 *   angular 清零 → `feedforward_dps = 0` → D 项退化成 `−Kd × measured_rate`，
 *   即**纯阻尼**（转向越快、往回拉的力越大），不再严格等于误差微分。
 *   这仍然有用，只是含义从"误差微分"变成了"负反馈阻尼"。
 *
 *   另外：**D 项不使用 `dt_s`** —— 角速度已经是 deg/s，不需要再除以时间。
 *   所以我们**没有** `previous_error` 这个成员。
 *
 * @note  **积分用真实秒 `Ki·error·dt_s`**，和速度环的 `integral += error` 不同，
 *   两个文件的 Ki 数值不能互相套用。另注意 `dt_s` 传进来的是**常量 0.020f**
 *   而不是实测周期，所以积分强度仍然依赖外环真的每 20 ms 跑一次。
 *
 * @note  **输入校验与安全回退**：下面那一长串条件在挡三类坏输入 ——
 *   非有限值（NaN/Inf，可能是传感器异常或异常浮点）、
 *   `dt_s <= 0` 或 `dt_s > 0.1`（循环被卡住、时间戳异常；用这种 dt 积分会算错，
 *   0.1 s 是根保险丝）、`output_limit_dps <= 0`（没有额度可用）。
 *   任一命中就**清积分并返回 0**（不转向）——"宁可不动，也不要乱动"。
 *
 * @note  **双层限幅**：积分项 `candidate` 自己先被限幅到 ±output_limit_dps，
 *   然后 `base + 积分` 再被限幅一次。所以积分单独不会超过满量程。
 *
 * @note  调用节奏：由 chassis_control.c 用 `heading_divider` 二分频，
 *   **每 2 个控制周期（20 ms）调用一次**，所以 `dt_s` 传 0.020f。
 *   两次调用之间如果线速度变了，`chassis_control.c` 还会对
 *   `heading_output_dps` 再补一次几何限幅。
 */
float HeadingPID_Calculate(HeadingPID *pid, float target_deg, float measured_deg,
                           float measured_rate_dps, float feedforward_dps,
                           float dt_s, float output_limit_dps)
{
  float error;
  float base;
  float candidate;

  /* --- 输入校验：任一项不合法就清积分、不转向（见上面的 @note） ---------- */
  if (!isfinite(target_deg) || !isfinite(measured_deg) ||
      !isfinite(measured_rate_dps) || !isfinite(feedforward_dps) ||
      !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f ||
      !isfinite(output_limit_dps) || output_limit_dps <= 0.0f)
  {
    HeadingPID_Reset(pid);
    return 0.0f;
  }

  /* --- 误差走最短路径 --------------------------------------------------- */
  error = HeadingPID_WrapDegrees(target_deg - measured_deg);

  /* --- 不含积分的部分：前馈 + P + D ------------------------------------- */
  /* D 项用陀螺仪角速度而不是误差差分，避免目标角度跳变引起微分冲击
   * （详见函数说明；注意它不吃 dt_s）。 */
  base = feedforward_dps + pid->kp * error +
         pid->kd * (feedforward_dps - measured_rate_dps);

  /* --- 积分项：真实秒单位 + 条件积分抗饱和 -------------------------------- */
  /* Ki=0 时把积分强制清零，保证控制器里不留无效状态（默认就是这个情况）。 */
  if (pid->ki == 0.0f) pid->integral_dps = 0.0f;
  /* 先算出"接受这次积分后"的值，并单独限幅（第一层限幅）。 */
  candidate = Clamp(pid->integral_dps + pid->ki * error * dt_s,
                    output_limit_dps);
  /* 条件积分：饱和时只允许有助于退出饱和的积分方向。
   * 也就是说，误差反向（能帮输出脱离饱和）时照常累加，积分才能正常退饱和。 */
  if (!((base + candidate > output_limit_dps && error > 0.0f) ||
        (base + candidate < -output_limit_dps && error < 0.0f)))
  {
    pid->integral_dps = candidate;
  }
  /* 前馈 + 反馈打包限幅（第二层限幅）——这是"前馈吃额度"的地方。 */
  return Clamp(base + pid->integral_dps, output_limit_dps);
}
