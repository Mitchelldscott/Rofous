#include "system_graph/process.h"

#include "sensors/lsm6dsox.h"
#include "algorithms/complimentary_filter.h"

#ifndef PROC_FACTORY
#define PROC_FACTORY

#define LSM6DSOX_ID "LSM"
#define COMPLIMENTARYFILTER_ID "CMF"


/*
	Factories have never been so simple. This factories purpose is to simplify
	initializing processes based on a user defined string id. With this object
	users will initialize processes through an interface rather than through
	organizing and scheduling their own main/setup functions.

	In order to register a new process, add a defining string above and
	add a case below that checks for the proper ID and calls your
	initializer.
*/
class Process_Factory {
	// private:
		
	public:
		Process* new_proc(String index) {
			if (index == LSM6DSOX_ID) {
				return new LSM6DSOX();
			}
			else if (index == COMPLIMENTARYFILTER_ID) {
				return new ComplimentaryFilter();
			}
			else {
				return nullptr;
			}
		}
};

#endif