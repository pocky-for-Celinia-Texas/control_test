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

#endif	// CONTROL_TASK_HPP