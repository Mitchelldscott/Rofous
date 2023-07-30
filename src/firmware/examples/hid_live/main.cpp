#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/assertions.h"
#include "robot_comms/hid_report.h"

#define MASTER_CYCLE_TIME_US 	1000.0
#define MASTER_CYCLE_TIME_MS 	1.0
#define MASTER_CYCLE_TIME_S 	0.001
#define MASTER_CYCLE_TIME_ERR 	1.001 // ms

FTYK timers;
HidReport report;
IntervalTimer myTimer;

volatile int write_count = 0;

void push_hid() {
	report.read();
	report.write();
	write_count++;
}

// Runs once
void setup() {
	while(!Serial){}

	Serial.println("=== Starting Live HID tests ===");
	interrupts();

	timers.set(0);
	myTimer.begin(push_hid, MASTER_CYCLE_TIME_US);
}

// Master loop
int main() {
	setup();
	int loops = write_count;

	timers.set(0);
	timers.set(1);
	while (timers.secs(0) < 1) {		// dump the write timer afap
		if (loops % 100 == 0) {
			Serial.printf("loops per write_count: %i / %i = %f\n", loops, write_count, float(loops) / float(write_count));
		}
		timers.delay_micros(1, MASTER_CYCLE_TIME_US);
		timers.set(1);
		loops++;
	}

	timers.set(0);
	timers.set(1);
	while (timers.secs(0) < 1) {
		assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Teensy overcycled"); // timer error threashold (very tight)
		timers.set(1);
	}

	timers.set(0);
	timers.set(1);
	while (timers.secs(0) < 1) {
		timers.delay_millis(1, MASTER_CYCLE_TIME_MS + 1); // force overcycle
		assert_geq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Teensy did not overcycle"); // timer error threashold (very tight)
		timers.set(1);
	}
	
	return 0;
}
