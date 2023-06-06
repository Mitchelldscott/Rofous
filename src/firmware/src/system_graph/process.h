#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

#include "utilities/vector.h"

class Process {
	public:
		virtual void reset();
		virtual void clear();
		virtual void print();
		virtual void context(Vector<float>*);
		virtual void setup(Vector<float>);
		virtual void run(Vector<float>*, Vector<float>*);
};

#endif