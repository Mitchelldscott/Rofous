#ifndef GRAPH_NODE
#define GRAPH_NODE

#include "utilities/vector.h"
#include "system_graph/process.h"

/*
	GraphNode is a large step toward the RTOS this code base creates.
	The GraphNode is like a container for user defined processes.
	Each process will have a GraphNode and each GraphNode will have
	the output and locations of where to get the input.

	By running GraphNodes in a proper order each will aquire it's input
	from the target GraphNodes (inputs) and pass this to its process making the output
	requestable by other GraphNodes. Graph cycles pose a significant threat to 
	the behavior of this system and should be avoided.

	Each GraphNode will also store its own "context" or configuration data.
	This is a Vector in the GraphNode, but each value can be interpreted as
	what ever the user wants. Simply write the driver expecting a specific context
	structure and fill that context out accordingly in the yamls. A context is not
	a specific state but a list of values the driver will interpret.
*/

class GraphNode {
	private:
		int config_shape;

		Process* proc;
		Vector<int> inputs;
		Vector<float> output_buffer;
		Vector<float> config_buffer;
		Vector<float> context_buffer;

	public:
		GraphNode();
		GraphNode(Process*, int, Vector<int>);

		void set_config(int);
		void set_inputs(Vector<int>);
		void set_process(Process*);

		bool setup_proc();
		bool run_proc(Vector<float>*);

		void print_proc();
		void print_output();

		Vector<float>* output();
		Vector<float>* config();
		Vector<float>* context();
		Vector<int>* input_ids();
		bool is_configured();
};

#endif