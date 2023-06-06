#ifndef GRAPH_NODE
#define GRAPH_NODE

#include "utilities/vector.h"
#include "system_graph/process.h"

class GraphNode {
	private:
		int output_shape;
		int config_shape;

		Process* proc;
		Vector<int> inputs;
		Vector<float> proc_out;
		Vector<float> setup_data;

	public:
		GraphNode();
		GraphNode(Process*, int, int, Vector<int>);

		void setup_proc();
		void run_proc(Vector<float>*);

		void print_proc();
		void print_output();

		Vector<float>* output();
		Vector<float>* config();
		Vector<int>* input_ids();
		bool is_configured();
};

#endif