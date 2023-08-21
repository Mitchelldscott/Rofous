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

#include <Arduino.h>

#ifndef BUFF_TIMING
#define BUFF_TIMING

// Some generic converters
#define MS_2_NS(ms) 	(ms * 1E6)
#define MS_2_US(ms) 	(ms * 1E3)
#define MS_2_S(ms) 		(ms / 1E3)

#define US_2_NS(us) 	(us * 1E3)
#define US_2_MS(us) 	(us / 1E3)
#define US_2_S(us) 		(us / 1E6)

#define NS_2_US(ns) 	(ns / 1E3)
#define NS_2_MS(ns) 	(ns / 1E6)
#define NS_2_S(ns) 		(ns / 1E9)

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_2_S(cycles)  	((cycles) / float(F_CPU))
#define CYCLES_2_MS(cycles)  	((cycles) * (1E3 / float(F_CPU)))
#define CYCLES_2_US(cycles)  	((cycles) * (1E6 / float(F_CPU)))
#define CYCLES_2_NS(cycles)  	((cycles) * (1E9 / float(F_CPU)))

// Get time duration from two cycle counts
#define DURATION_S(cyccnt1, cyccnt2) (CYCLES_TO_S(cyccnt2 - cyccnt1))
#define DURATION_MS(cyccnt1, cyccnt2) (CYCLES_TO_MS(cyccnt2 - cyccnt1))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))

#define MAX_NUM_TIMERS 	10
#define MAX_CYCCNT 		float(0xFFFFFFFF)

/*
	Class to provide multiple timers that have exceptional
	precision (Still hardware limited).
*/

class FTYK {
	private:
		int active_timers[MAX_NUM_TIMERS];
		float cyccnt_mark[MAX_NUM_TIMERS];
		float accumulator[MAX_NUM_TIMERS];

	public:
		FTYK();
		void set(int);
		void mark(int);
		float cycles(int);
		float nanos(int);
		float micros(int);
		float millis(int);
		float secs(int);
		float delay_micros(int, float);
		float delay_millis(int, float);
		void print(int);
		void print(int, String);
};

#endif