#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/loggers.h"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us / 1E6;

// Runs once
void setup() {
	Serial.begin(1000000);								// the serial monitor is always active 
														// (for debug use Serial.println & tycmd monitor)

	if (Serial) {
		Serial.println("-- TEENSY SERIAL START --");
		Serial.println("-- new build... who dis? --");
	}
}

// Master loop
int main() {											// The Main Jawns
		
	return 0;
}
