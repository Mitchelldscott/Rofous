#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/loggers.h"
#include "system_graph/system_graph.h"

#define MASTER_CYCLE_TIME_US 	1000.0
#define MASTER_CYCLE_TIME_MS 	1.0
#define MASTER_CYCLE_TIME_S 	0.001
#define MASTER_CYCLE_TIME_ERR 	1.00025 // ms

FTYK timers;
SystemGraph sg;

// Runs once
void setup() {
	if (Serial) {
		Serial.println("-- DyseBot Firmware Initialized --");
	}
	timers.set(0);
}

// Master loop
int main() {
	timers.set(1);
	while (1) {
		// sg.spin();
		sg.handle_hid();
		if (timers.delay_millis(1, MASTER_CYCLE_TIME_MS) > MASTER_CYCLE_TIME_ERR) { // timer error threashold (very tight)
			Serial.printf("Teensy overcycled %f/%f ms\n", timers.millis(1), MASTER_CYCLE_TIME_ERR);
		}
		timers.set(1);
	}
	return 0;
}
