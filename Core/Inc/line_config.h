#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

/* YAHBOOM YB-MVX05 八路循迹模块：
 * AD0、AD1、AD2 用于选择 X1～X8 探头，OUT 输出数字高低电平，不是 ADC 模拟量。
 */

/* ==================== 探头采样与异常图样处理 ==================== */

#define LINE_MUX_SETTLE_US          50U
/* 切换探头地址后，等待 50 微秒，再读取 OUT 电平。 */

#define LINE_SINGLE_GAP_MAX_SPAN    4U
/* 单缺口容错允许的最大探头跨度。
 * 命中图样最多跨 4 个探头位置，并且只允许缺失中间一路。
 * 不表示允许任意缺失 4 路。
 */

#define LINE_AMBIGUOUS_HOLD_MS      20U
/* 非零但不可靠的图样，最多维持上一次可靠转向 20 毫秒。
 * 确认期间限制速度、不加速；持续无效达到此时间后降至 60 mm/s。
 * 全白图样不等待确认，立即降至 60 mm/s。
 */

#define LINE_AMBIGUOUS_MAX_SPEED    300.0f
/* 非零无效图样确认期间的目标速度上限，单位 mm/s。
 * 实际取“此前目标速度、当前模式正常弯道下限、本参数”三者最小值。
 */

#define LINE_DEFAULT_ACTIVE_HIGH   1
/* 默认黑线有效电平：1 = 高电平表示黑线，0 = 低电平表示黑线。
 * 可通过串口 LPOL=0/1 修改。
 */

#define LINE_DEFAULT_REVERSE       0
/* 默认探头左右顺序：
 * 0 = 按 X1 到 X8 对应车体左侧到右侧；
 * 1 = 将上述顺序反转。
 * 应以实测为准，可通过串口 LREV=0/1 修改。
 */


/* ==================== 循迹转向控制 ==================== */

#define LINE_DEFAULT_KP            180.0f
/* 默认循迹比例系数 P，可通过 LKP 修改。
 * 根据横向偏差产生纠偏转向；过大可能导致左右摆动。
 */

#define LINE_DEFAULT_KD            6.0f
/* 默认循迹微分系数 D，可通过 LKD 修改。
 * 根据滤波后的偏差变化率产生转向修正，用于抑制摆动。
 * 过大也可能放大探头跳变造成的转向波动。
 */

#define LINE_MAX_ANGULAR            90.0f
/* 循迹 PD 转向请求的绝对值上限，单位 centirad/s。
 * 90 对应 0.9 rad/s，约为 51.6 度/秒，不是 90 度/秒。
 */

#define LINE_YAW_RATE_KP            0.25f
/* IMU 角速度反馈比例系数。
 * 根据要求角速度与实测角速度的差值，补偿底盘转向。
 */

#define LINE_SHAPE_RESPONSE_S       0.05f
/* 循迹模式运动规划的最短响应时间尺度，单位秒。
 * 实际过渡时间还受加速度、加加速度和轮速限制约束。
 */

#define LINE_DIFF_ACCEL_MM_S2       3000.0f
/* 转向差速分量 Δ 的加速度上限，单位 mm/s²。
 * 左轮目标 = 车体速度 - Δ，右轮目标 = 车体速度 + Δ。
 */

#define LINE_DIFF_JERK_MM_S3        30000.0f
/* 转向差速分量 Δ 的加加速度上限，单位 mm/s³。
 * 控制转向差速变化的平滑程度。
 */


/* ==================== 正常循迹速度 ==================== */

#define LINE_DEFAULT_TEST_SPEED    350.0f
/* RUN 连续循迹模式的默认正常速度上限，单位 mm/s。
 * 可通过串口 LSPD 修改。
 */

#define LINE_DEFAULT_TEST_MIN_SPEED 350.0f
/* RUN 模式正常弯道最低目标速度，单位 mm/s。
 * 可通过 LMIN 修改，生效值不超过 LSPD。
 * 无效图样确认、丢线和速度恢复期间，可以低于此值。
 */

#define LINE_DEFAULT_LAP_SPEED     400.0f
/* LAP 单圈模式的默认正常速度上限，单位 mm/s。
 * 可通过串口 LLAP 修改，不控制 RUN 模式。
 */

#define LINE_DEFAULT_CURVE_SPEED   350.0f
/* LAP 模式正常弯道最低目标速度，单位 mm/s。
 * 可通过串口 LCURVE 修改，不控制 RUN 模式。
 */


/* ==================== 降速后的恢复过程 ==================== */

#define LINE_SPEED_RECOVER_MS       100U
/* 降低循迹目标速度后，至少保持 100 毫秒才允许恢复加速。
 * 再次需要降速时重新计时，避免在弯道中反复提速、减速。
 */

#define LINE_SPEED_RECOVER_ACCEL     160.0f
/* 循迹目标速度恢复时的最大增加速率，单位 mm/s²。
 * 160 表示每秒最多增加 160 mm/s。
 * 实际轮速仍由底盘运动规划和电机速度 PI 执行。
 */


/* ==================== LAP 单圈返回与终点接近 ==================== */

#define LINE_APPROACH_SPEED        100.0f
/* LAP 接近终点时的目标速度上限，单位 mm/s。 */

#define LINE_APPROACH_DISTANCE_MM  5900.0f
/* LAP 累计行驶达到 5900 mm 后，开始执行终点接近限速。 */

#define LINE_MIN_LAP_DISTANCE_MM   5000.0f
/* LAP 至少行驶 5000 mm 后，才允许识别返程终点横线。
 * 还必须同时满足离开起点、累计顺时针转角等条件。
 */

#define LINE_MAX_LAP_DISTANCE_MM   7500.0f
/* LAP 超过 7500 mm 仍未完成任务，报告里程异常并停车。 */

#define LINE_MIN_LAP_YAW_DEG       300.0f
/* 允许识别返程横线所需的最小累计顺时针转角。
 * 本程序顺时针为负，因此实际要求累计转角 <= -300 度。
 */

#define LINE_START_CLEAR_MM        250.0f
/* 看见过窄线，并且累计前进至少 250 mm 后，
 * 才认为已离开起点横线，避免起步时误判返回终点。
 */


/* ==================== 横线识别与停车补偿 ==================== */

#define LINE_DEFAULT_MARK_COUNT    3U
/* 横线识别所需的最少命中探头数，可通过 LMARK 修改。
 * 同时要求命中组连续，并且中间两路探头都命中。
 * 单缺口容错图样不作为横线。
 */

#define LINE_MARK_DEBOUNCE_MS      20U
/* 满足返程条件的横线图样持续至少 20 毫秒，才确认横线。 */

#define LINE_DEFAULT_STOP_MM       0.0f
/* 默认停车补偿距离，单位 mm，可通过 LSTOP 修改。
 * 以首次检测返程横线的位置为距离基准：
 * 0 = 横线确认后请求停车；
 * 正数 = 行驶到指定补偿距离后请求停车。
 * 实际停车位置还受检测延迟和惯性滑行影响。
 */


/* ==================== 故障保护与停车确认 ==================== */

#define LINE_LOST_TIMEOUT_MS       120U
/* 从首次无效图样开始，连续 120 毫秒没有可靠线就锁存停车。
 * 包含前面的无效图样确认时间，不是在确认结束后再计时。
 */

#define LINE_STALL_TIMEOUT_MS      1500U
/* 约 1500 毫秒内，里程未累计前进 10 mm，
 * 则判断为无进展并停车；可能是堵转或里程反馈异常。
 */

#define LINE_LAP_TIMEOUT_MS        20000U
/* LAP 单圈任务的总时间上限：20000 毫秒，即 20 秒。
 * RUN 连续循迹模式不受此项限制。
 */

#define LINE_BRAKE_TIMEOUT_MS      1500U
/* 进入停车确认后，超过 1500 毫秒仍未满足停稳条件，
 * 报告停车异常。
 * 停稳条件为各轮速度绝对值均低于 20 mm/s，并持续 100 毫秒。
 */

/* New project only: PE0 AD0, PE1 AD1, PE2 AD2, PE3 OUT.
 * PE4 START (active-low); PE5 STOP (active-low). External keys connect to GND.
 * If the development board already loads these pins, remap GPIO and IOC together.
 */
#endif
