#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#include <cmath>	// std::isfinite / std::sqrt
#include <cstdint>	// uint32_t

#include "cmsis_os.h"  // osKernelSysTick()
#include "motor/rm_motor/rm_motor.hpp"

inline sp::RM_Motor rm_motor_x(1, sp::RM_Motors::M3508);   //右上 /A型
inline sp::RM_Motor rm_motor_y(2, sp::RM_Motors::M3508);   //左上 \B型
inline sp::RM_Motor rm_motor_x2(3, sp::RM_Motors::M3508);  //左下 /A型
inline sp::RM_Motor rm_motor_y2(4, sp::RM_Motors::M3508);  //右下 \B型
typedef struct
{
	float target_angle_set;	 // 绝对角度的目标值，rad
	float target_speed_set;	 // 绝对速度的目标值，rad/s
	float given_torque;		 // 电机给定的力矩，Nm
} GimbalData;

// ================= 功率模型 + 功率限制（输出只读！） =================
//  s_tw = Σ τi·ωi                        W         ← 转子侧，功率与减速比无关
//  s_tt = Σ τi²                          (N·m)²    ← 转子侧
//  s_ww = Σ ωi²                          (rad/s)²   ← 【轮端】转速，已 ÷减速比
//  p_in = s_tw + K1·s_tt + K2·s_ww + K3  W
// 约定：τ 一律用【转子侧 N·m】；ω 在 s_tw 里用转子，在 s_ww 里换成轮端。
//       所以 K1=2 配转子 τ²、K2=0.01 配轮端 ω²，两者单位不能互换。
// 注意：τ 传【即将下发的指令】，ω 传【反馈转速】。

// ---- 模型系数（与功率限制公式里的 K1/K2/K3 一一对应）----
constexpr float P_K1 = 2.0f;   // Στ² 的系数（配转子 τ²）
constexpr float P_K2 = 0.01f;  // Σω² 的系数（配轮端 ω²）
constexpr float P_K3 = 4.0f;   // 常数偏置 W

// ---- 转速单位换算：电机反馈是【转子 rad/s】，模型的 Σω² 要【轮端 rad/s】----
// 轮端 ω = 转子 ω ÷ 减速比；平方后等价于乘 1/187.69
// ⚠ 必须与 chassis_speed.hpp 的 GEAR_RATIO 保持一致
constexpr float P_GEAR_RATIO = 13.7f;
constexpr float P_OMEGA_TO_WHEEL = 1.0f / P_GEAR_RATIO;
constexpr float P_GEAR_RATIO_SQ = P_GEAR_RATIO * P_GEAR_RATIO;
constexpr float P_OMEGA_SQ_SCALE = P_OMEGA_TO_WHEEL * P_OMEGA_TO_WHEEL;	 // = 1/187.69

struct MotorSums
{
	// ---- 输出：只读！只有 update*() 会写 ----
	float s_tw = 0.0f;	 // Σ τ·ω   W
	float s_tt = 0.0f;	 // Σ τ²    (N·m)²
	float s_ww = 0.0f;	 // Σ ω²    轮端 (rad/s)²
	float p_in = 0.0f;	 // 输入功率估算 W
	bool valid = false;	 // 四台电机反馈都新鲜时才是 true

	// 主入口：τ 用指令值，ω 用反馈值
	void update_raw(float t0, float w0, float t1, float w1, float t2, float w2, float t3, float w3)
	{
		s_tw = t0 * w0 + t1 * w1 + t2 * w2 + t3 * w3;
		s_tt = t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
		s_tt = s_tt * P_GEAR_RATIO_SQ;
		// ω 是转子 rad/s，模型用轮端：÷减速比后平方，等价于乘 1/187.69
		s_ww = (w0 * w0 + w1 * w1 + w2 * w2 + w3 * w3) * P_OMEGA_SQ_SCALE;
		valid = true;

		calc_pin();	 // 三个和一变，p_in 立刻同步
	}

	// 观测入口：τ 用反馈值，只给 Plotter / 打印用，不参与控制
	void update_from_motors(
		const sp::RM_Motor & mx, const sp::RM_Motor & my, const sp::RM_Motor & mx2,
		const sp::RM_Motor & my2, uint32_t now_ms)
	{
		update_raw(
			mx.torque, mx.speed, my.torque, my.speed, mx2.torque, mx2.speed, my2.torque, my2.speed);

		valid = valid && mx.is_alive(now_ms) && my.is_alive(now_ms) && mx2.is_alive(now_ms) &&
				my2.is_alive(now_ms);
	}

	void update()  // 观测入口的便捷版
	{
		update_from_motors(rm_motor_x, rm_motor_y, rm_motor_x2, rm_motor_y2, osKernelSysTick());
	}

	// 解 K1·s_tt·K² + s_tw·K + (K2·s_ww + K3 - p_max) = 0，返回 [0, 1]
	float calc_k(float p_max) const
	{
		// ① 没超限：不缩放（多数时间走这条，省掉开方）
		if (p_in <= p_max) return 1.0f;

		const float a = P_K1 * s_tt;
		const float b = s_tw;
		const float c = P_K2 * s_ww + P_K3 - p_max;

		// ② 退化：几乎没有转矩可缩，二次项消失，方程退化成一次 b·K + c = 0
		if (a <= 1e-6f) {
			if (b > 1e-6f) {
				float k = -c / b;
				return (k < 0.0f) ? 0.0f : ((k > 1.0f) ? 1.0f : k);
			}
			return 0.0f;  // 无转矩、无机械功率，地板功耗仍超 p_max → 全砍
		}

		// ③ 判别式
		const float disc = b * b - 4.0f * a * c;
		if (disc < 0.0f) return 0.0f;  // 模型地板已超 p_max，没有 K 能压下来

		// ④ 取正根（另一根必为负）
		float k = (-b + std::sqrt(disc)) / (2.0f * a);
		if (!(k > 0.0f)) k = 0.0f;	// 顺带挡掉 NaN
		if (k > 1.0f) k = 1.0f;		// 只缩小，不放大
		return k;
	}

  private:
	void calc_pin()
	{
		p_in = s_tw + P_K1 * s_tt + P_K2 * s_ww + P_K3;

		if (!std::isfinite(p_in)) {	 // NaN/Inf 不能流到 PID、限幅、积分
			s_tw = s_tt = s_ww = p_in = 0.0f;
			valid = false;
		}
	}
};

inline MotorSums motor_sums;  // 全局唯一实例

// ================= 功率计（只读，不控制） =================
class PowerMeter
{
  public:
	static constexpr uint16_t RX_ID = 0x301;  // ← 改成功率计实际反馈帧 ID

	float current = 0.0f;  // A
	float voltage = 0.0f;  // V
	float power = 0.0f;	   // W

	// 在 CAN 接收中断里调用：只做移位+换算，别加 osDelay、别打印
	void read(uint8_t * data, uint32_t stamp_ms)
	{
		// 大端，高字节在前；电流/功率按有符号解析（回充为负）
		current = (int16_t)((data[0] << 8) | data[1]) * 0.01f;	 // 电流 ×100
		voltage = (uint16_t)((data[2] << 8) | data[3]) * 0.01f;	 // 电压 ×100
		power = (int16_t)((data[4] << 8) | data[5]) * 0.01f;	 // 功率 ×100

		last_read_ms_ = stamp_ms;
		has_read_ = true;
	}

	bool is_alive(uint32_t now_ms) const { return has_read_ && (now_ms - last_read_ms_ < 100); }

  private:
	uint32_t last_read_ms_ = 0;
	bool has_read_ = false;
};

inline PowerMeter power_meter;	// 全局唯一对象
// ==========================================================

#endif	// CONTROL_TASK_HPP