#include "control_task.hpp"

#include "chassis_speed.hpp"
#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;
GimbalData rm_motor_data_x2;
GimbalData rm_motor_data_y2;

// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 0.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.003f, 0.01f, 0.0f, 1.0f, 0.5f, 0.0f, false, true);

// 电机 Y 的两环 PID
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 0.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.003f, 0.01f, 0.0f, 1.0f, 0.5f, 0.0f, false, true);

// 电机 X2 的两环 PID
sp::PID rm_motor_pid_angle_x2(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 0.0f, true, true);
sp::PID rm_motor_pid_speed_x2(0.001f, 0.003f, 0.01f, 0.0f, 1.0f, 0.5f, 0.0f, false, true);

// 电机 Y2 的两环 PID
sp::PID rm_motor_pid_angle_y2(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 0.0f, true, true);
sp::PID rm_motor_pid_speed_y2(0.001f, 0.003f, 0.01f, 0.0f, 1.0f, 0.5f, 0.0f, false, true);

// ==========================================================================================
//  功率限制参数区
// ==========================================================================================
//  裁判系统的 chassis_power_limit（referee_protocol.hpp:251）还没接进本工程
//  （CMakeLists 里没有编译 referee/*.cpp），所以这里先用常量。
//  接上之后把 constexpr 改成 float，在循环里 P_max = 裁判系统读到的值。
// ==========================================================================================
constexpr float P_MAX_W = 80.0f;  // W，底盘功率上限

// ---- 实测功率反馈（功率计 0x301）----
constexpr float P_MARGIN_W = 5.0f;		// W，模型误差余量：模型目标 = P_MAX_W - P_MARGIN_W
constexpr float P_TRIM_TAU_S = 0.3f;	// s，实测修正时间常数（超了快压、低了慢放 4×）
constexpr float P_TRIM_MAX_W = 30.0f;	// W，闭环时最多往下压多少
constexpr float P_TRIM_SAFE_W = 20.0f;	// W，功率计掉线时的固定下调量（没标定可改 30）
// ==========================================================================================

// ==========================================================================================
//  UP 档（自动前进到指定路程后停车）参数区 —— 调参只改这一块
//  ★完整调参说明 + 对照表见：applications/readme.md★
// ==========================================================================================
//  一次动作：S形加速 → 匀速 → S形减速 → 停在 UP_RUN_DIST 处
//  【三个主旋钮】
//    UP_SPEED_MPS  想跑多快（m/s）
//    UP_ACCEL_MS   起步加到满速用多久（ms）—— 越小，曲线越陡、加速越猛
//    UP_DECEL_MS   从满速减到 0 用多久（ms）—— 越小，曲线越陡、停车越猛
//  曲线形状固定（五次 S 曲线），上面两个"时长"只决定它的胖瘦（横向伸缩），形状本身
//  不变；所以无论调多陡，"两端加速度为 0、全程没有加速度阶跃"这个特性都不会丢。
//  长度 m、速度 m/s、时间 ms
// ==========================================================================================
constexpr float UP_SPEED_MPS =
	1.56f;	// 巡航(最大)速度 m/s；想跑更快，必须同时调大UP_ACCEL_MS /UP_DECEL_MS（把 S 形曲线拉胖）否则峰值加速度会超限、起步打滑。
constexpr float UP_RUN_DIST = 4.0f;		   // 目标路程 m（净位移：前进为正，被推后退会减回去）
constexpr uint32_t UP_ACCEL_MS = 1000;	   // 加速段时长 ms（越小越猛）
constexpr uint32_t UP_DECEL_MS = 1000;	   // 减速段时长 ms（越小越猛）
constexpr float UP_STOP_DEADBAND = 0.02f;  // 收尾死区 m：最后这点距离直接归零，避免低速爬行
// ==========================================================================================

// ---- 以下全部由上面推导，不要手改（注释里是当前参数下的实际值）----
constexpr float S_CURVE_PEAK_SLOPE = 1.875f;  // 五次 S 曲线的峰值斜率（在 u=0.5 处）
constexpr float S_CURVE_MEAN = 0.5f;		  // 五次 S 曲线的区间均值 ∫₀¹s du

constexpr float UP_ACCEL_S = UP_ACCEL_MS * 0.001f;				// = 1.000 s
constexpr float UP_DECEL_S = UP_DECEL_MS * 0.001f;				// = 1.000 s
constexpr float UP_SPEED_ROTOR = UP_SPEED_MPS * ROTOR_PER_MPS;	// ≈ 281 转子 rad/s

// 峰值加速度 = 峰值斜率 × 速度 ÷ 时长（由"时长"反推出来，供打滑校验用）
constexpr float UP_ACCEL_PEAK =
	S_CURVE_PEAK_SLOPE * UP_SPEED_MPS / UP_ACCEL_S;	 // ≈ 2.93 m/s²(0.30g)
constexpr float UP_DECEL_PEAK =
	S_CURVE_PEAK_SLOPE * UP_SPEED_MPS / UP_DECEL_S;	 // ≈ 2.93 m/s²(0.30g)

// S 形曲线走完一段的路程 = 速度 × 时长 × 区间均值(0.5)
constexpr float UP_ACCEL_DIST = UP_SPEED_MPS * UP_ACCEL_S * S_CURVE_MEAN;  // = 0.780 m
constexpr float UP_DECEL_DIST = UP_SPEED_MPS * UP_DECEL_S * S_CURVE_MEAN;  // = 0.780 m
constexpr float UP_DECEL_START = UP_RUN_DIST - UP_DECEL_DIST;	  // = 2.220 m（供参考/调试观察）
constexpr float UP_ARRIVE_DIST = UP_RUN_DIST - UP_STOP_DEADBAND;  // = 2.980 m 判定"到达"

// 预计总时长 ≈ 加速时长 + 全程按巡航速度跑完的时间；超时给 2 倍 + 3 s 余量
constexpr float UP_EST_S = UP_ACCEL_S + UP_RUN_DIST / UP_SPEED_MPS;			// = 2.92 s
constexpr uint32_t UP_TIMEOUT_MS = (uint32_t)(UP_EST_S * 2000.0f) + 3000u;	// ≈ 8846 ms

// ---- 配置守卫：调参调坏了，编译期就报错，不会等车在场上打滑 ----
static_assert(UP_ACCEL_MS > 0 && UP_DECEL_MS > 0, "加减速时长必须 > 0");
static_assert(
	UP_DECEL_DIST < UP_RUN_DIST,
	"减速段路程 >= 目标路程：起步时减速限速就已生效，会以高速起步。请调大 UP_RUN_DIST 或调小 "
	"UP_DECEL_MS");
static_assert(
	UP_ACCEL_PEAK < 4.0f && UP_DECEL_PEAK < 4.0f,
	"加减速度超过约 0.4g：麦轮会打滑。请调大 UP_ACCEL_MS / UP_DECEL_MS");

// 五次平滑阶跃：u∈[0,1] → [0,1]；两端一阶导、二阶导都为 0
// ⟹ 加/减速的起止都没有冲击（这是防打滑的关键）
static float smoothstep5(float u)
{
	if (u < 0.0f) u = 0.0f;
	if (u > 1.0f) u = 1.0f;
	float u2 = u * u;
	return u2 * u * (10.0f + u * (-15.0f + 6.0f * u));	// 10u³ − 15u⁴ + 6u⁵
}

// 给 Plotter 用：本帧实际生效的缩放系数、本帧预测功率（不缩放时会用多少瓦）
float power_k = 1.0f;
float power_pin_pred = 0.0f;

// 实测功率反馈：power_trim 是"模型少算了多少瓦"的在线估计
float power_trim = 0.0f;	   // 实测修正量 W（0 = 不修正）
float power_pin_after = 0.0f;  // 修正后模型预测的实际功率 W

static bool power_meter_seen = false;	// 是否收到过功率计
static uint32_t power_meter_ok_ms = 0;	// 最后一次收到功率计的时刻

// 四台电机的目标转矩 → 功率限制 → 下发
// 顺序必须是「四台目标转矩全部算完 → 统一乘 K → 一起 cmd」。
// 算一台发一台是错的：第一台下发时 K 还没定，这一帧就已经不守恒。
static void power_limited_cmd(float t_x, float t_y, float t_x2, float t_y2)
{
	// τ 用即将下发的指令，ω 用反馈转速（转速是缩不了的）
	motor_sums.update_raw(
		t_x, rm_motor_x.speed, t_y, rm_motor_y.speed, t_x2, rm_motor_x2.speed, t_y2,
		rm_motor_y2.speed);

	const uint32_t now = osKernelSysTick();
	const float p_target = P_MAX_W - P_MARGIN_W;  // 75 W

	// ---- 实测功率慢修正：模型管快，实测管准 ----
	// 只有"限幅器真的在决定功率"时，实测才包含模型偏差的信息：
	//   p_in > 修正后目标 → K<1，正在限幅；
	//   实测 > 目标       → 不管模型怎么说，已经超了。
	// 其他时候（巡航、刹车、DOWN）保持不动，避免把"需求功率"当成"模型输出"去学。
	const bool meter_ok = power_meter.is_alive(now);
	if (meter_ok) {
		power_meter_seen = true;
		power_meter_ok_ms = now;

		const bool at_limit = (motor_sums.p_in > (p_target - power_trim));
		const bool over = (power_meter.power > p_target);
		if (at_limit || over) {
			const float dt = 0.001f;						 // 本循环固定 1 ms
			const float err = power_meter.power - p_target;	 // >0：实际超了
			// 超了压得快（τ=P_TRIM_TAU_S），低了放得慢（4×τ）
			power_trim += err * dt / (err > 0.0f ? P_TRIM_TAU_S : P_TRIM_TAU_S * 4.0f);
		}
	}
	else if (!power_meter_seen || (now - power_meter_ok_ms) > 1000u) {
		// 从没收到 / 掉线超过 1 s：退到安全值
		power_trim = P_TRIM_SAFE_W;
	}
	// 掉线 1 s 内保持上次修正量（模型偏差是慢变量）

	if (power_trim < 0.0f) power_trim = 0.0f;
	if (power_trim > P_TRIM_MAX_W) power_trim = P_TRIM_MAX_W;

	const float p_target_eff = p_target - power_trim;

	power_pin_pred = motor_sums.p_in;			// K=1 的预测（只给 plot 看）
	power_k = motor_sums.calc_k(p_target_eff);	// 模型 + 实测修正，[0, 1]

	// 硬兜底：实测真的超过 P_MAX_W，按比例再砍一刀
	if (meter_ok && power_meter.power > P_MAX_W) {
		power_k *= P_MAX_W / power_meter.power;
	}

	// 修正后的预测实际功率：乘完最终 K（含硬兜底）后，模型认为会有多少瓦
	power_pin_after = motor_sums.power_at(power_k);

	rm_motor_x.cmd(t_x * power_k);
	rm_motor_y.cmd(t_y * power_k);
	rm_motor_x2.cmd(t_x2 * power_k);
	rm_motor_y2.cmd(t_y2 * power_k);
}

static uint32_t up_accel_ms = 0;
static float up_distance_m = 0.0f;		 // UP 档已行驶路程(净位移), 单位: m
static bool up_finished = false;		 // 本次动作是否已完成(到点/超时), 锁存防抖
static uint32_t up_start_ms = 0;		 // 本次动作开始时刻, 超时兜底用
static float up_stop_distance_m = 0.0f;	 // 停车瞬间的路程读数, 便于复盘"它以为自己停在哪"

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::DOWN;

	while (true) {
		// 检测拨杆模式变化：进入 UP 档时把上次动作的状态全部清零、重新跑一次；
		// 切出 UP 档时保持现状（up_distance_m 冻结），方便复盘"它停在哪"
		bool mode_changed = (remote.sw_r != last_sw_r);
		if (mode_changed) {
			// 进入 UP：复位本次动作的所有状态
			// up_start_ms 必须在这里赋值——否则它恒为 0，开机超过 UP_TIMEOUT_MS 后
			// 第一次切 UP 就会立刻命中超时（osKernelSysTick() - 0 > UP_TIMEOUT_MS），
			// 表现为 up_finished 马上锁存、车一动不动
			if (remote.sw_r == sp::DBusSwitchMode::UP) {
				up_distance_m = 0.0f;
				up_stop_distance_m = 0.0f;
				up_finished = false;
				up_start_ms = osKernelSysTick();
				up_accel_ms = 0;
			}
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_speed_y.clear();
			rm_motor_pid_speed_x2.clear();
			rm_motor_pid_speed_y2.clear();
		}
		last_sw_r = remote.sw_r;

		//         右拨杆 UP：自动模式
		// ========================================
		if (remote.sw_r == sp::DBusSwitchMode::UP) {
			if (up_accel_ms < UP_ACCEL_MS) {  //加速计时
				up_accel_ms++;
			}
			// ① 起步/匀速段：时间驱动的 S 曲线（u 从 0 涨到 1）
			float s_acc = smoothstep5((float)up_accel_ms / (float)UP_ACCEL_MS);

			// ② 停车段：剩余距离驱动的同一条 S 曲线（剩余距离从 d_dec 降到 0）
			//    剩 d_dec 时 s_dec=1（正好开始减速），剩 0 时 s_dec=0（正好停住）
			float remaining = UP_RUN_DIST - up_distance_m;
			if (remaining < 0.0f) remaining = 0.0f;
			float s_dec = smoothstep5(remaining / UP_DECEL_DIST);

			// ③ 取小 → 自动拼出「S形加速 + 匀速 + S形减速」
			//    两条曲线在切换点都等于 1，所以切换处速度连续，不会有任何突变
			float s = (s_acc < s_dec) ? s_acc : s_dec;

			// ④ 收尾：最后一点距离直接归零（避免低速爬行）；动作完成后锁死
			if (remaining < UP_STOP_DEADBAND || up_finished) {
				s = 0.0f;
			}

			float target_speed_up = UP_SPEED_ROTOR * s;	 //单个轮子目标角速度(转子 rad/s)

			rm_motor_pid_speed_x.calc(-target_speed_up, rm_motor_x.speed);
			float ffm_x = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_x.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			rm_motor_pid_speed_y.calc(target_speed_up, rm_motor_y.speed);
			float ffm_y = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_y.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_y = (rm_motor_pid_speed_y.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			rm_motor_pid_speed_x2.calc(target_speed_up, rm_motor_x2.speed);
			float ffm_x2 = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_x2.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_x2 = (rm_motor_pid_speed_x2.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			rm_motor_pid_speed_y2.calc(-target_speed_up, rm_motor_y2.speed);
			float ffm_y2 = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_y2.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_y2 = (rm_motor_pid_speed_y2.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			power_limited_cmd(
				rm_motor_pid_speed_x.out + ffm_x, rm_motor_pid_speed_y.out + ffm_y,
				rm_motor_pid_speed_x2.out + ffm_x2, rm_motor_pid_speed_y2.out + ffm_y2);
		}

		// ========================================
		//         右拨杆 MID：手动模式
		// ========================================
		else if (remote.sw_r == sp::DBusSwitchMode::MID) {
#if 1  //调试时去掉mid档改为0
			float direction = 0;
			if (remote.ch_lu > 0.3f) {
				direction = -1.0f;
			}
			else if (remote.ch_lu < -0.3f) {
				direction = 1.0f;
			}
			float target_speed_x =
				-200.0f * remote.ch_lv + 200.0f * remote.ch_lh + direction * 63.0f;

			rm_motor_pid_speed_x.calc(target_speed_x, rm_motor_x.speed);
			float ffm_x = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_x.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			float target_speed_y =
				200.0f * remote.ch_lv + 200.0f * remote.ch_lh + direction * 63.0f;

			rm_motor_pid_speed_y.calc(target_speed_y, rm_motor_y.speed);
			float ffm_y = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_y.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_y = (rm_motor_pid_speed_y.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			float target_speed_x2 =
				200.0f * remote.ch_lv - 200.0f * remote.ch_lh + direction * 63.0f;

			rm_motor_pid_speed_x2.calc(target_speed_x2, rm_motor_x2.speed);
			float ffm_x2 = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_x2.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_x2 = (rm_motor_pid_speed_x2.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			float target_speed_y2 =
				-200.0f * remote.ch_lv - 200.0f * remote.ch_lh + direction * 63.0f;

			rm_motor_pid_speed_y2.calc(target_speed_y2, rm_motor_y2.speed);
			float ffm_y2 = 0.0f;
			/*if (fabsf(rm_motor_pid_speed_y2.out) > 0.5f) {  // 目标速度够大才补偿
                ffm_y2 = (rm_motor_pid_speed_y2.out > 0 ? 1.0f : -1.0f) * 0.052f;
            }*/
			power_limited_cmd(
				rm_motor_pid_speed_x.out + ffm_x, rm_motor_pid_speed_y.out + ffm_y,
				rm_motor_pid_speed_x2.out + ffm_x2, rm_motor_pid_speed_y2.out + ffm_y2);
#else
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_x2.cmd(0.0f);
			rm_motor_y2.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			rm_motor_pid_angle_x2.clear();
			rm_motor_pid_speed_x2.clear();
			rm_motor_pid_angle_y2.clear();
			rm_motor_pid_speed_y2.clear();
#endif
		}

		// ========================================
		//         右拨杆 DOWN：禁用
		// ========================================
		else {
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_x2.cmd(0.0f);
			rm_motor_y2.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			rm_motor_pid_angle_x2.clear();
			rm_motor_pid_speed_x2.clear();
			rm_motor_pid_angle_y2.clear();
			rm_motor_pid_speed_y2.clear();
		}

		update_chassis_speed();

		motor_sums.update();  // 反馈版：只给 Plotter/打印看，不参与控制

		// UP 档：用刚更新完的车速积分路程(净位移，前进为正)
		// 循环周期恰好 1 ms(osDelay(1) + 1kHz tick)，所以 dt 直接写 0.001f
		// 电机掉线时车速会停在旧值，用 valid 挡住，避免路程一直虚涨
		if (remote.sw_r == sp::DBusSwitchMode::UP && !up_finished && chassis.valid) {
			up_distance_m += chassis.vx * 0.001f;
		}

		// 到点 / 超时 → 锁存停车
		// 注意：超时判断不能加 valid 条件 —— 掉线时路程不涨，否则车会一直冲出去
		if (remote.sw_r == sp::DBusSwitchMode::UP && !up_finished) {
			if (up_distance_m >= UP_ARRIVE_DIST ||
				(osKernelSysTick() - up_start_ms) > UP_TIMEOUT_MS) {
				up_finished = true;
				up_stop_distance_m = up_distance_m;	 // 记录停车读数，便于复盘
			}
		}

		send_rm_motor();

		osDelay(1);
	}
}