#ifndef GRAPH_NODE
#define GRAPH_NODE

#include "system_graph/vector.h"
#include "system_graph/process.h"

template <class T> class GraphNode {
	private:
		int id;
		int n_outputs;
		Process<T> proc;
		Vector<int> output;
		Vector<float> proc_out;

	public:
		// object functions
		GraphNode<T>(int, int, int*);
		~GraphNode<T>();
		int* next_ids(); // send output to these IDs
		int n_outputs();

		// process functions
		void run_proc(float*);
		void setup_proc(float*);

		// vector functions
		float* output();
};

#endif