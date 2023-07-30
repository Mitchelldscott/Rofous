// #include <Arduino.h>
#include "syncor/syncor.h"
// #include "utilities/timing.h"
// #include "utilities/vector.h"
// #include "utilities/assertions.h"
// #include "sensors/lsm6dsox.h"
// #include "syncor/syncor_node.h"

#define MASTER_CYCLE_TIME_US 	1000.0
#define MASTER_CYCLE_TIME_MS 	1.0
#define MASTER_CYCLE_TIME_S 	0.001
#define MASTER_CYCLE_TIME_ERR 	1.001 // ms


FTYK timers;

int main() {
	while(!Serial){}

	Serial.println("=== Starting System Graph tests ===");
	
	float tmp[1] = {0.6};
	int imu_input;
	int cmf_inputs[2] = {9, 10};

	timers.set(0);
	SynCor sc;
	assert_leq<float>(timers.micros(0), 2, "SC init timer");
	
	timers.set(1);
	sc.add("LSM", 10, 0, 0, &imu_input);
	assert_leq<float>(timers.millis(1), 50, "LSM init timer");

	timers.set(1);
	sc.add("CMF", 9, 1, 2, cmf_inputs);
	assert_leq<float>(timers.micros(1), 6, "CMF init timer");

	timers.set(1);
	sc.update_config(9, 0, 1, tmp);
	assert_leq<float>(timers.micros(1), 2, "CMF config timer");

	for (int i = 0; i < 3; i++) {
		timers.set(1);
		sc.spin();
		assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Process run timing test");
	}

	while (!usb_rawhid_available()) {}
	timers.set(0);
	while (timers.millis(0) < 50) { // show HID callback is faster than 1us
		timers.set(1);
		sc.handle_hid();
		assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "HID callback timing test");
	}
	assert_leq<float>(timers.millis(0), 51, "HID full timing test");

	sc.enable_hid_interrupts();

	timers.set(0);
	while (timers.millis(0) < 50) { // show HID callback is faster than 1us
		timers.set(1);
		sc.spin();
		assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Process run with interrupts timing test");
	}
	assert_leq<float>(timers.millis(0), 51, "Process run with interrupts full timing test");


	sc.dump_all();

	Serial.println("=== Finished System Graph tests ===");
	
}