// #include <Arduino.h>
// #include "system_graph/vector.h"
// #include "system_graph/graph_node.h"

#include "sensors/lsm6dsox.h"

template <class T> GraphNode<T>::GraphNode() {
	id = -1;
	// n_inputs = ninputs;
	// n_outputs = outputs;
	output_shape = 0;
	proc_out.reset(0);
}

template <class T> GraphNode<T>::GraphNode(int i, int outputs, Vector<float> config) {
	id = i;
	// n_inputs = ninputs;
	// n_outputs = outputs;
	output_shape = outputs;
	proc_out.reset(outputs);
}


template <class T> void GraphNode<T>::run_proc(Vector<Vector<float>> inputs) {
	proc_out.from_vec(proc.run(inputs));
}

template <class T> Vector<float> GraphNode<T>::output() {
	return proc_out;
}

template <class T> void GraphNode<T>::print_proc() {
	proc.print();
}

template <class T> void GraphNode<T>::print_output() {
	proc_out.print();
}

template class GraphNode<DriverTypes>;
