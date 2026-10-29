/*
 * encoder.c — 四路硬件正交编码器：把"计数值"变成"轮速 RPM"
 *
 * ============================ 它在这条链路里的位置 ============================
 *
 *   电机轴上的增量式正交编码器（A/B 两相）
 *          ↓  硬件定时器的编码器模式（TI12，4 倍频）
 *   四个 TIM 的 CNT 寄存器           ← 硬件自己累加，CPU 完全不参与
 *          ↓  Encoder_Sample10ms()  ← 本文件：读 CNT、做差、换算、滤波
 *   s_speed_rpm[4]
 *          ↓  Encoder_GetSpeedsRpm()
 *   SpeedPID_Calculate() 的 measured 参数（速度内环的反馈量）
 *
 * **硬件计时，软件读取**：四个 TIM 工作在编码器模式下，正交信号直接驱动
 * 计数器增/减，CPU 一个中断都不用进。这也是为什么测速这么"便宜"。
 *
 * ============================== 硬件分配 ==============================
 *
 *   | 轮子 | 位置 | 定时器 | 引脚 | 计数器位宽 |
 *   |---|---|---|---|---|
 *   | A | 左前 | TIM2 | PA15 / PB3 | **32 位** |
 *   | B | 左后 | TIM4 | PD12 / PD13 | 16 位 |
 *   | C | 右前 | TIM1 | PE9 / PE11 | 16 位 |
 *   | D | 右后 | TIM8 | PC6 / PC7 | 16 位 |
 *
 * 只有 A 轮落在 32 位定时器上（F4 上只有 TIM2/TIM5 是 32 位）——**这是引脚分配
 * 决定的，不是刻意选择**。正因为位宽不统一，下面才需要两个不同的读函数。
 *
 * 编码器信号还在 tim.c 里做了数字滤波（IC1Filter/IC2Filter = 10），
 * 能滤掉窄于约 1.5~3 µs 的毛刺（TIM1/TIM8 在 APB2→168 MHz 上约 1.5 µs，
 * TIM2/TIM4 在 APB1→84 MHz 上约 3 µs）。这能挡电气毛刺，但挡不住
 * 真正的计数丢失——见文件末尾"分辨率极限"。
 *
 * ============================== 量纲与刻度 ==============================
 *   本文件的输出单位是 **RPM（转/分钟）**，是轮子的转速。
 *   chassis_control.c 再乘 π×轮径/60 换算成 mm/s 给上层用。
 *
 *   每转计数 = ENCODER_COUNTS_PER_REVOLUTION = 1320（TI12 四倍频后的计数）。
 *   换算系数：60 / (1320 × 0.01) = **4.5455 RPM 每计数**
 *
 * ========================= 两个必须实车核对的硬件约定 =========================
 *
 *   1) **编码器方向系数**（ENCODER_x_DIRECTION）
 *      电机极性、编码器 A/B 相接线顺序、减速箱朝向三者共同决定"正转时计数
 *      是增还是减"，**没有可推理的规律**，所以四个轮子各留一个符号常量。
 *      架空测试时若某轮给正目标却反馈负转速，只改对应那个符号即可。
 *      注意它和 motor_driver.c 里的方向电平是**两套独立的约定**，都要对。
 *
 *   2) **每转计数 1320**
 *      工程沿用的 520 电机规格值，尚未在实车上复核（轮径 65 mm 也还没标定）。
 *      文档里给的推行标定法：断电直线推 1 米，读编码器总计数，
 *      轮径 = 1000 × 1320 / 总计数 / π。
 *
 * ============================== 调用上下文 ==============================
 *   本文件所有函数**只被 controlTask 调用**：
 *     Chassis_Init()         → Encoder_Start() → Encoder_Reset()
 *     Chassis_ControlStep()  → Encoder_Sample10ms() → Encoder_GetSpeedsRpm()
 *   也就是说没有跨任务访问。Encoder_GetSpeedsRpm()/Encoder_Reset() 里的
 *   临界区目前是**防御性**的，不是当前必需的——保留成本很低（几微秒），
 *   但如果将来有别的任务要读转速，它才真正开始起作用。
 *   同理 s_speed_rpm 上的 volatile 目前也是防御性的。
 */

#include "encoder.h"

#include "tim.h"

/* 采样周期，秒。**必须和 TIM7 的实际周期一致**：
 * TIM7 = 84 MHz / (8399+1) / (99+1) = 100 Hz → 10 ms。
 * ⚠️ 测速公式把它当分母，所以它写错/控制周期改了这个常量没跟着改，
 *    所有转速会整体按比例错（改 10 ms→5 ms 而这里不动，读数会直接翻倍）。
 *    和 speed_pid.c 里 Ki/Kd 的周期依赖是同一类坑。 */
#define ENCODER_SAMPLE_PERIOD_SECONDS  0.01f

/* 一阶低通滤波系数：y += α·(x − y)，每 10 ms 执行一次。
 * α 越大越跟手但越抖，越小越平滑但越滞后。
 * α=0.35 的等效时间常数 = −Δt/ln(1−α) = −0.01/ln(0.65) ≈ **23 ms**，
 * 等效 −3dB 截止频率约 **7 Hz**。
 * 这 23 ms 的滞后是速度环带宽的上限之一——Kp 调大到一定程度后，
 * 摆动的根源就变成这个滤波滞后，而不是电机本身了。
 * 沿用已调好的 520 电机参考工程。 */
#define SPEED_FILTER_ALPHA             0.35f

/*
 * 车轮顺序与底盘保持一致：A左前、B左后、C右前、D右后。
 * 如果架空测试时某轮正转反馈为负，只修改对应方向系数的正负号。
 *
 * 当前值：A/B = −1，C/D = +1。这只是"本车当前接线"的结论，
 * 换电机、换编码器、调换 A/B 相都会让它变，所以测试时第一个要看的就是它。
 */
#define ENCODER_A_DIRECTION  (-1.0f)
#define ENCODER_B_DIRECTION  (-1.0f)
#define ENCODER_C_DIRECTION  ( 1.0f)
#define ENCODER_D_DIRECTION  ( 1.0f)

/* 上一次采样时读到的计数器值，用来做差得到"这 10 ms 走了多少计数"。
 * 四个轮子各一份。注意即使三个是 16 位计数器，这里也统一用 uint32_t 存，
 * 16 位分支在读取时再强转回去。 */
static uint32_t s_previous_count[ENCODER_WHEEL_COUNT];
/* 滤波后的轮速，单位 RPM。下标 = 轮序（0=A 1=B 2=C 3=D）。 */
static volatile float s_speed_rpm[ENCODER_WHEEL_COUNT];
static int64_t s_total_counts[ENCODER_WHEEL_COUNT];

/* 把四个方向系数做成表，避免在换算循环里写 switch。 */
static const float s_direction[ENCODER_WHEEL_COUNT] = {
  ENCODER_A_DIRECTION,
  ENCODER_B_DIRECTION,
  ENCODER_C_DIRECTION,
  ENCODER_D_DIRECTION
};

/**
 * @brief  读 32 位编码器计数器（A 轮 / TIM2），返回本次与上次的差值。
 * @param  htim   编码器定时器句柄（传 &htim2）。
 * @param  wheel  轮序 0~3，用来定位 s_previous_count 里的那一份。
 * @retval 本次采样周期内的计数增量，**带符号**（正=计数增加，负=减少）。
 *
 * @note  **正向减法天然处理回绕，不需要任何 if 判断。**
 *   这是这段代码最值得理解的地方。无符号整数相减是"模 2³² 运算"：
 *
 *     上次 = 0xFFFF_FF00，本次 = 0x0000_0100（计数器刚绕过 0）
 *     按普通算术应是 +512，但直接减会得到 0xFFFF_FF00 反向 borrow……
 *     实际上 unsigned 减法给出 (0x100 − 0xFFFFFF00) mod 2³² = **0x200 = 512** ✓
 *
 *   也就是说：**只要两次采样之间真实增量不超过 ±2³¹，模运算的结果就是正确的。**
 *   32 位计数器永远不会碰到这个边界（要走 21 亿个计数）。
 *
 * @note  s_previous_count 在返回前就已更新，所以连续调用两次
 *   第二次会返回 0（或极小的残留值）——本函数是"消耗型"的，不要重复调。
 */
static int32_t Encoder_ReadDelta32(TIM_HandleTypeDef *htim, uint32_t wheel)
{
  uint32_t current = __HAL_TIM_GET_COUNTER(htim);
  int32_t delta = (int32_t)(current - s_previous_count[wheel]);
  s_previous_count[wheel] = current;
  return delta;
}

/**
 * @brief  读 16 位编码器计数器（B/C/D 轮），返回本次与上次的差值。
 * @param  htim   编码器定时器句柄（&htim4 / &htim1 / &htim8）。
 * @param  wheel  轮序 0~3。
 * @retval 本次采样周期内的计数增量，带符号。
 *
 * @note  原理和上面的 32 位版完全一样，只是位宽换成 16 位：
 *   两个 uint16_t 相减 → 模 2¹⁶ 的差（自动处理 0xFFFF→0x0000 的回绕），
 *   再**强转成 int16_t**，把大于 0x7FFF 的结果解释成负数。
 *
 *   边界条件：|真实增量| 必须 < 32768，否则符号会判反。
 *   实际余量极大——250 RPM 时每 10 ms 只有 250/60×1320×0.01 ≈ **55 个计数**，
 *   离 32768 差了约 600 倍。所以这个强转在任何工况下都是安全的。
 *
 * @note  注意 `s_previous_count[wheel]` 是 uint32_t，写进去时零扩展、
 *   读出来时先转回 uint16_t，往返不丢信息。
 */
static int32_t Encoder_ReadDelta16(TIM_HandleTypeDef *htim, uint32_t wheel)
{
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
  int16_t delta = (int16_t)(current - (uint16_t)s_previous_count[wheel]);
  s_previous_count[wheel] = current;
  return (int32_t)delta;
}

/**
 * @brief  启动四个编码器定时器并清零全部状态。
 * @param  无
 * @retval HAL_OK 表示四路都启动成功；HAL_ERROR 表示有任一路失败，
 *         此时**已经把已启动的那几路都停掉**再返回，不留半开状态。
 * @note   由 Chassis_Init() 调用（在调度器启动之后）。
 *         失败时 Chassis_Init() 会继续撤销 PWM 并返回 HAL_ERROR，
 *         最终由 controlTask 走 Error_Handler()。
 */
HAL_StatusTypeDef Encoder_Start(void)
{
  Encoder_Reset();

  /* A左前：TIM2，PA15/PB3，32位计数器。 */
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* B左后：TIM4，PD12/PD13，16位计数器。 */
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Encoder_Stop();
    return HAL_ERROR;
  }

  /* C右前：TIM1，PE9/PE11，16位计数器。 */
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Encoder_Stop();
    return HAL_ERROR;
  }

  /* D右后：TIM8，PC6/PC7，16位计数器。 */
  if (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Encoder_Stop();
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief  停止四个编码器定时器。
 * @param  无
 * @retval 无
 * @note   无条件停四路（包括本来就没启动的），所以可以当"全部收尾"用。
 *         返回值被丢弃：停不下来也没有更好的补救办法。
 */
void Encoder_Stop(void)
{
  (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim8, TIM_CHANNEL_ALL);
}

/**
 * @brief  把四个计数器清零，并清掉软件侧的上次计数和滤波后的转速。
 * @param  无
 * @retval 无
 * @note   由 Encoder_Start() 调用。**必须同时清硬件 CNT 和软件 s_previous_count**，
 *         否则第一次做差会得到一个巨大的假增量。
 *         用 __disable_irq() 保护是防御性的（本函数只在 controlTask 里跑）。
 */
void Encoder_Reset(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);

  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    s_previous_count[wheel] = 0U;
    s_total_counts[wheel] = 0;
    s_speed_rpm[wheel] = 0.0f;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief  每 10 ms 调用一次：读四路计数增量 → 换算成瞬时 RPM → 一阶低通滤波。
 * @param  无
 * @retval 无（结果写进内部的 s_speed_rpm[]，用 GetSpeedsRpm 取）
 * @note   由 Chassis_ControlStep() 在每个控制周期开头调用，
 *         是整个速度环的**输入端点**。控制周期不稳，测速就不准。
 *
 * @note  换算公式（以 A 轮为例）：
 *
 *     instantaneous_speed = delta × 方向系数 × 60
 *                           ────────────────────────
 *                           1320 × 0.01
 *
 *   拆开看每一项的来历：
 *     delta     这 10 ms 的计数增量（带符号）
 *     × 方向系数 把"计数增减"翻译成"轮子前进/后退"
 *     × 60      每秒 → 每分钟（RPM 是"每分钟"转数）
 *     ÷ 1320    计数 → 转
 *     ÷ 0.01    每 10 ms → 每秒
 *
 *   合并起来：**1 个计数 = 60/13.2 = 4.5455 RPM**。
 *
 * @note  滤波是标准的一阶 IIR：`y += α·(x − y)`，
 *         等价于 `y[n] = (1−α)·y[n−1] + α·x[n]`。
 *         α=0.35 → 等效时间常数约 23 ms。
 *
 *         ⚠️ 滤波**只能把量化台阶"摊平"，不能消除它**：输入是整数计数，
 *         本身就带 4.55 RPM 的台阶；滤波后的残留纹波（峰峰）≈
 *         α/(2−α) × 4.55 = 0.21 × 4.55 ≈ **0.96 RPM**。
 *         这一路纹波再乘上 Kp=0.5（0.48%）→ 约 **41 个 PWM 计数**
 *         （满量程 8399）。所以 VOFA 上看到的 PWM 细密纹波，
 *         有一部分是这个刻度极限造成的，调增益治不了。
 */
void Encoder_Sample10ms(void)
{
  int32_t delta[ENCODER_WHEEL_COUNT];

  /* 先一次性把四路增量都读出来，避免换算过程中计数器继续变化
   * 导致四个轮子的采样时刻不一致。 */
  delta[0] = Encoder_ReadDelta32(&htim2, 0U);
  delta[1] = Encoder_ReadDelta16(&htim4, 1U);
  delta[2] = Encoder_ReadDelta16(&htim1, 2U);
  delta[3] = Encoder_ReadDelta16(&htim8, 3U);

  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    s_total_counts[wheel] += (s_direction[wheel] > 0.0f) ? delta[wheel] : -(int64_t)delta[wheel];
    float instantaneous_speed =
        ((float)delta[wheel] * s_direction[wheel] * 60.0f) /
        (ENCODER_COUNTS_PER_REVOLUTION * ENCODER_SAMPLE_PERIOD_SECONDS);

    /* 与已调好520电机参考工程一致的一阶低通滤波。 */
    s_speed_rpm[wheel] += SPEED_FILTER_ALPHA *
                          (instantaneous_speed - s_speed_rpm[wheel]);
  }
}

/**
 * @brief  取出四个轮子滤波后的转速。
 * @param  speed_rpm  [out] 长度 4 的数组，单位 RPM。
 *                    下标 = 轮序：0=A左前 1=B左后 2=C右前 3=D右后。
 *                    **传 NULL 直接返回**，不写任何东西。
 * @retval 无
 * @note   由 Chassis_ControlStep() 在 Encoder_Sample10ms() 之后立刻调用，
 *         把结果拷进 s_chassis.measured_rpm，再喂给速度环。
 *         拷贝（而不是返回指针）是为了让调用方拿到一份**一致的快照**。
 *
 * @note   ⚠️ **分辨率极限（本文件最重要的工程结论）**
 *
 *   反馈量的最小刻度是 1 个计数 / 10 ms：
 *
 *   | 量 | 值 |
 *   |---|---|
 *   | 1 计数 / 10 ms | **4.545 RPM** |
 *   | 换算成轮速（π×65mm） | **15.47 mm/s** |
 *   | 200 mm/s 时占目标的 | **7.7%** |
 *   | 500 mm/s 时占目标的 | 3.1% |
 *   | 250 RPM 满速时每 10 ms 的计数 | 约 55 |
 *
 *   也就是说**低速段（比如 50~100 mm/s）一个台阶就占目标的 15~30%**，
 *   速度环在那个区间本质上是"半量化"运行的。想改善只有三条路：
 *   拉长采样窗口（牺牲带宽）、换更高线数的编码器（硬件）、
 *   或者接受它并限制最低工作速度。
 */
void Encoder_GetSpeedsRpm(float speed_rpm[ENCODER_WHEEL_COUNT])
{
  uint32_t primask;

  if (speed_rpm == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    speed_rpm[wheel] = s_speed_rpm[wheel];
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Encoder_GetTotalCounts(int64_t counts[ENCODER_WHEEL_COUNT])
{
  if (counts == NULL) return;
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  for (uint32_t i = 0; i < ENCODER_WHEEL_COUNT; ++i) counts[i] = s_total_counts[i];
  if (saved == 0U) __enable_irq();
}
