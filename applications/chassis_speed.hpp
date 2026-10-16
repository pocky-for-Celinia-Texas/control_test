#ifndef CHASSIS_SPEED_HPP
#define CHASSIS_SPEED_HPP

// ============================================================
//  chassis_speed.hpp
//  作用：由四个 M3508 的 CAN 反馈推算底盘速度
//  用法：在任务循环里每 1 ms 调一次 update_chassis_speed()
//        之后直接用全局变量 chassis.vx / chassis.vy / chassis.wz
//  注意：纯头文件，不需要改 CMakeLists.txt
// ============================================================

#include "cmsis_os.h"
#include "control_task.hpp"	 // 四个电机对象 rm_motor_x / y / x2 / y2
#include "tools/mecanum/mecanum.hpp"

// ================ 按你的车填这 4 个数 ================
constexpr float WHEEL_RADIUS = 0.076f;	// 轮子【半径】，m（不是直径！）
constexpr float GEAR_RATIO = 13.7f;		// 减速比 ≈13.7
constexpr float HALF_LENGTH = 0.200f;	// 轴距的一半，m
constexpr float HALF_WIDTH = 0.200f;	// 轮距的一半，m
// ===================================================

struct ChassisSpeed
{
	float vx = 0.0f;	 // 前进速度   m/s   （朝车头为正）
	float vy = 0.0f;	 // 左移速度   m/s   （朝左为正）
	float wz = 0.0f;	 // 自转角速度 rad/s （从上往下看，逆时针为正）
	bool valid = false;	 // 四个电机都在线时，这些值才可信
};

inline ChassisSpeed chassis;  // "当前车速"变量

// 麦轮正运动学解算器
// 轮序约定固定为 LF(左前), LR(左后), RF(右前), RR(右后)
// 使用默认 reverse 标志：左前/左后正装，右前/右后反装
// 若实测方向不对，改为显式传参，顺序: reverse_lf, reverse_lr, reverse_rf, reverse_rr
inline sp::Mecanum mecanum_kinematics(WHEEL_RADIUS, HALF_LENGTH, HALF_WIDTH);

// 每 1 ms 调用一次
inline void update_chassis_speed()
{
	// ① 掉线检查：电机断线时 speed 会停在最后一帧的旧值，不能当实时车速用
	const uint32_t now = osKernelSysTick();
	chassis.valid = rm_motor_x.is_alive(now) && rm_motor_y.is_alive(now) &&
					rm_motor_x2.is_alive(now) && rm_motor_y2.is_alive(now);

	// ② 转子 rad/s → 轮子 rad/s（除以减速比）
	const float w_rf = rm_motor_x.speed / GEAR_RATIO;	// 右前 ID1
	const float w_lf = rm_motor_y.speed / GEAR_RATIO;	// 左前 ID2
	const float w_lr = rm_motor_x2.speed / GEAR_RATIO;	// 左后 ID3
	const float w_rr = rm_motor_y2.speed / GEAR_RATIO;	// 右后 ID4

	// ③ 四个轮子 → 底盘速度（update 参数顺序必须是 LF, LR, RF, RR）
	mecanum_kinematics.update(w_lf, w_lr, w_rf, w_rr);

	chassis.vx = mecanum_kinematics.vx;	 // m/s
	chassis.vy = mecanum_kinematics.vy;	 // m/s
	chassis.wz = mecanum_kinematics.wz;	 // rad/s
}

#endif	// CHASSIS_SPEED_HPP
