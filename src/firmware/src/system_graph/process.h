#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

#include "system_graph/vector.h"

class Process {
	public:
		Process();
		void reset();
		void clear();
		void print();
		Vector<float> context();
		void setup(Vector<float>);
		Vector<Vector<float>> run(Vector<Vector<float>>);
};

#endif