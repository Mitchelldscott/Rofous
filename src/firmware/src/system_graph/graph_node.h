#ifndef GRAPH_NODE
#define GRAPH_NODE

#include "system_graph/vector.h"
#include "system_graph/process.h"

template <class T> class GraphNode {
	private:
		int id;
		int output_shape;
		// int num_input_edges;
		// int num_output_edges;

		Process<T> proc;
		// Vector<int> input_ids;
		// Vector<int> output_ids;
		// Vector<int> input_shapes;
		Vector<float> proc_out;

	public:
		GraphNode<T>();
		GraphNode<T>(int, int, Vector<float>);
		// ~GraphNode<T>();
		// int* next_ids(); // send output to these IDs
		// int n_outputs();
		void run_proc(Vector<Vector<float>>); // since it can have many inputs (an input is a 1D-Vector) need a 2D Vector
		// void setup_proc(Vector<float>);
		Vector<float> output();
		void print_proc();
		void print_output();
};

#endif