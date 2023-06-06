#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/loggers.h"
#include "system_graph/system_graph.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1E3;
float cycle_time_s = cycle_time_us / 1E6;

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
		sg.handle_hid();
	}
	return 0;
}
