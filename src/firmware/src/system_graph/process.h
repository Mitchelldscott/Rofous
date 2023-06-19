#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

#include "utilities/vector.h"

/*
	Base class for all user defined processes.
	Must override the functions below, constructor
	optional... I think.
*/

class Process {	
	public:
		virtual void reset();
		virtual void clear();
		virtual void print();
		virtual void setup(Vector<float>*);
		virtual void context(Vector<float>*);
		virtual void run(Vector<float>*, Vector<float>*);
		bool operator == (const Process&);
};

#endif