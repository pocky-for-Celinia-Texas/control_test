#include "cmsis_os.h"
#include "control_task.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"	// ← 如果还没 include

extern sp::Mahony imu;	// ← 声明外部对象

sp::Plotter plotter(&huart1);
/*static float sw_to_float(sp::DBusSwitchMode sw)
{
	switch (sw) {
		case sp::DBusSwitchMode::UP:
			return 1.0f;
		case sp::DBusSwitchMode::MID:
			return 0.0f;
		case sp::DBusSwitchMode::DOWN:
			return -1.0f;
		default:
			return -2.0f;
	}
}*/

// clang-format off
extern "C" void plot_task()
{
  
	while (1) {

    plotter.plot(
      //rm_motor_x.angle,
      rm_motor_x.speed
      //rm_motor_x.torque
      //imu.vyaw,
      //imu.vpitch,
      //imu.vroll
      //sw_to_float(remote.sw_r)
	);

		osDelay(5);
	}
}
// clang-format on