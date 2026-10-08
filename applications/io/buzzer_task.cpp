#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task()
{
	osDelay(200);

	for (int i = 0; i < 3; i++) {
		buzzer.start();
		buzzer.set(1000 + 500 * i, 0.01);
		osDelay(50);
		buzzer.stop();
		osDelay(50);
	}

	while (true) {
		osDelay(100);
	}
}