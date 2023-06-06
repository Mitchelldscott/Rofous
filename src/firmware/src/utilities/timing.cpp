#include "timing.h"


uint32_t duration_info(uint32_t start, uint32_t stop){
	/*
	  Helper to print info about *very small* durations in
	time. Prints the cycles and time in ns.
	@param
	  start: (uint32_t) value of ARM_DWT_CYCCNT at the beginning of a duration
	  stop: (uint32_t) value of ARM_DWT_CYCNT at the end of a duration
	@return
	  delta_ns: (uint32_t) duration in nanoseconds
	*/
	uint32_t delta_cycles = stop - start;
	uint32_t delta_ns = CYCLES_2_NS(delta_cycles); 
	Serial.printf( "\t%1lu cycles, %1lu ns\n", delta_cycles, delta_ns);
	return delta_ns;
}

FTYK::FTYK() {
	for (size_t i = 0; i < MAX_NUM_TIMERS; i++) {
		timers[i] = ARM_DWT_CYCCNT;
	}
}


void FTYK::set(int idx) {
	timers[idx] = ARM_DWT_CYCCNT;
}

void FTYK::mark(int idx) {
	duration_info(timers[idx], ARM_DWT_CYCCNT);
}

uint32_t FTYK::cycles(int idx) {
	return ARM_DWT_CYCCNT - timers[idx];
}

float FTYK::nanos(int idx) {
	return CYCLES_2_NS(cycles(idx)); 
}

float FTYK::micros(int idx) {
	return NS_2_US(CYCLES_2_NS(cycles(idx))); 
}

float FTYK::millis(int idx) {
	return NS_2_MS(CYCLES_2_NS(cycles(idx)));
}

float FTYK::secs(int idx) {
	return NS_2_S(CYCLES_2_NS(cycles(idx)));
}

float FTYK::delay_micros(int idx, float duration){
	/*
	  Helper to pause for a duration. Duration starts
	when set() is called.
	@param
	  duration: (uint32_t) microseconds to wait (from when set() was called)
	@return
		None
	*/
	while(CYCLES_2_US(cycles(idx)) < duration) {}
	return micros(idx);
}

float FTYK::delay_millis(int idx, float duration){
	/*
	  Helper to pause for a duration. Duration starts
	when set() is called.
	@param
	  duration: (uint32_t) milliseconds to wait (from when set() was called)
	@return
		None
	*/
	while(CYCLES_2_MS(cycles(idx)) < duration) {}
	return millis(idx);
}