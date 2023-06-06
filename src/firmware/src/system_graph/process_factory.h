#include "system_graph/process.h"

#include "sensors/lsm6dsox.h"
#include "algorithms/complimentary_filter.h"

#ifndef PROC_FACTORY
#define PROC_FACTORY

// #define LSM6DSOX_ID "LSM"

class Process_Factory {
	// private:
		
	public:
		Process* new_proc(String index) {
			if (index == "LSM") {
				return new LSM6DSOX();
			}
			else if (index == "CMF") {
				return new ComplimentaryFilter();
			}
			else {
				return nullptr;
			}
		}
};

#endif