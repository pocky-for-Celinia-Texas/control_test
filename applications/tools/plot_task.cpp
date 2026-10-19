#include "chassis_speed.hpp"  // ← chassis.vx / vy / wz 在这里定义
#include "cmsis_os.h"
#include "control_task.hpp"	 // ← motor_sums / power_meter / P_K2 在这里定义
#include "data_interfaces/uart/uart.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"	// ← 如果还没 include

extern sp::Mahony imu;	// ← 声明外部对象

// control_task.cpp 里的功率限制中间量
extern float power_k;		  // 本帧实际生效的缩放系数 [0, 1]
extern float power_pin_pred;  // 本帧预测功率（不缩放时会用多少瓦）

sp::Plotter plotter(&huart1);

// clang-format off
// 通道顺序（对应上位机 1..6）：
//   1 power_pin_pred       预测功率（限幅器拿去做判断的那个值）  W
//                          > P_MAX_W(80) 就说明正在限幅
//   2 power_meter.power    功率计实测功率                        W
//   3 power_k              缩放系数 K                            [0, 1]
//                          长期 < 1 就是在限幅，贴 0 说明模型地板就超了
//   4 motor_sums.p_in      模型估算功率（反馈口径）              W
//                          ← 和通道 2 对比：贴合说明 K2 标定准
//   5 motor_sums.s_tw      Στω 真实机械功率                      W
//   6 chassis.vx           车速                                  m/s
//
// 921600 baud：6 通道 27 字节/帧 @200Hz ≈ 5.4 kB/s，占带宽不到 6%
extern "C" void plot_task()
{
	while (1) {
		plotter.plot(
			power_pin_pred,
			power_meter.power,
			//power_k,
			//motor_sums.p_in,
			//motor_sums.s_tw,
			chassis.vx
		);

		osDelay(5);
	}
}
// clang-format on
