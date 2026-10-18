#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

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