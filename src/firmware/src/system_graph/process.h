#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

#include "system_graph/vector.h"

enum class DriverTypes {
	LSM6DSOX,
};

template <class T> class Process {
	private:
		T* driver;

	public:
		Process<T>();
		Process<T>(Vector<float>);
		~Process<T>();
		void setup(Vector<float>);
		void reset();
		void clear();
		Vector<float> state();
		Vector<float> run(Vector<Vector<float>>);
		void print();
};

#endif