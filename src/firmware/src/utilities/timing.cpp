/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

#include "timing.h"


FTYK::FTYK() {
	/*
		  Object to get precise timing, nanos is available but not really supported.
		Contains MAX_NUM_TIMERS so different things can be timed simultaneously.

		TODO:
			- add rollover support: ARM_DWT_CYCCNT is a cycle count that resets every 8
			seconds or so, use a member variable to track the number of roll overs.
			This will be dependant on the timer being able to check those rollovers, so
			it will need to be called often (maybe sysgraph needs a timer case check).
	*/
	for (size_t i = 0; i < MAX_NUM_TIMERS; i++) {
		cyccnt_mark[i] = ARM_DWT_CYCCNT;
		accumulator[i] = 0;
	}
}


void FTYK::set(int idx) {
	/*
		  Set the timer at idx to the current cycle count.
		@param:
			idx: (int) index of the timer to set.
	*/
	cyccnt_mark[idx] = ARM_DWT_CYCCNT;
	active_timers[idx] = 1;
	accumulator[idx] = 0;
}

void FTYK::mark(int idx) {
	/*
		  Print info about the timer at idx.
		@param:
			idx: (int) index of the timer to print info about.
	*/
	print(idx);
}

float FTYK::cycles(int idx) {
	/*
		  Get the number of cycles since last timer.set().
		returns the cycle count as a float to avoid overflows.
		@param:
			idx: (int) index of the timer to get cycles from.
		@return:
			cyccnt: (float) cyles since the timer was set
	*/
	float cyccnt = ARM_DWT_CYCCNT;
	// 
	if (cyccnt < cyccnt_mark[idx]) {
		accumulator[idx] += MAX_CYCCNT + cyccnt - cyccnt_mark[idx];
		cyccnt_mark[idx] = cyccnt;
	}
	else {
		accumulator[idx] += cyccnt - cyccnt_mark[idx];
		cyccnt_mark[idx] = cyccnt;
	}

	return accumulator[idx];// + cyccnt - cyccnt_mark[idx];
}

float FTYK::nanos(int idx) {
	/*
		  Get the number of nanoseconds since last timer.set().
		@param:
			idx: (int) index of the timer to get cycles from.
	*/
	return CYCLES_2_NS(cycles(idx)); 
}

float FTYK::micros(int idx) {
	/*
		  Get the number of microseconds since last timer.set().
		@param:
			idx: (int) index of the timer to get cycles from.
	*/
	return CYCLES_2_US(cycles(idx)); 
}

float FTYK::millis(int idx) {
	/*
		  Get the number of milliseconds since last timer.set().
		@param:
			idx: (int) index of the timer to get cycles from.
	*/
	return CYCLES_2_MS(cycles(idx));
}

float FTYK::secs(int idx) {
	/*
		  Get the number of seconds since last timer.set().
		@param:
			idx: (int) index of the timer to get cycles from.
	*/
	return CYCLES_2_S(cycles(idx));
}

float FTYK::delay_micros(int idx, float duration){
	/*
	  Helper to pause for a duration. Duration starts
	when set() is called, which must be called prior.
	@param
		duration: (uint32_t) microseconds to wait (from when set() was called)
	*/
	static int i = 0;
	float tmp = cycles(idx);
	while(CYCLES_2_US(tmp) < duration) {
		tmp = cycles(idx);
		cycles(i);
		i = (i + 1) % MAX_NUM_TIMERS;
	}
	return CYCLES_2_US(tmp);
}

float FTYK::delay_millis(int idx, float duration){
	/*
	  Helper to pause for a duration. Duration starts
	when set() is called, which must be called prior.
	@param
		duration: (uint32_t) milliseconds to wait (from when set() was called)
	*/
	static int i = 0;
	float tmp = cycles(idx);
	while(CYCLES_2_MS(tmp) < duration) {
		tmp = cycles(idx);
		cycles(i);
		i = (i + 1) % MAX_NUM_TIMERS;
	}
	return CYCLES_2_MS(tmp);
}

void FTYK::print(int idx) {
	float cyccnt = cycles(idx);
	float ns = CYCLES_2_NS(cyccnt);
	Serial.printf("Timer %i\n", idx);
	Serial.printf("%f s | %f ms | %f us | %f ns | %f cycles\n", 
		NS_2_S(ns),
		NS_2_MS(ns),
		NS_2_US(ns),
		ns, cyccnt);
}

void FTYK::print(int idx, String title) {
	float cyccnt = cycles(idx);
	float ns = CYCLES_2_NS(cyccnt);
	Serial.print(title); Serial.printf(" Timer %i\n", idx);
	Serial.printf("%f s | %f ms | %f us | %f ns | %f cycles\n", 
		NS_2_S(ns),
		NS_2_MS(ns),
		NS_2_US(ns),
		ns, cyccnt);
}