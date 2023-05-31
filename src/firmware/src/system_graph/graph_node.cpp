// #include <Arduino.h>
// #include "system_graph/vector.h"
// #include "system_graph/graph_node.h"

// #include "sensors/lsm6dsox.h"

// GraphNode::GraphNode() {
// 	id = -1;
// 	// n_inputs = ninputs;
// 	// n_outputs = outputs;
// 	output_shape = 0;
// 	proc_out.reset(0);
// }

// GraphNode::GraphNode(int i, int outputs, Vector<float> config) {
// 	id = i;
// 	// n_inputs = ninputs;
// 	// n_outputs = outputs;
// 	output_shape = outputs;
// 	proc_out.reset(outputs);
// }

// void GraphNode::run_proc(Vector<Vector<float>> inputs) {
// 	proc_out.from_vec(proc.run(inputs));
// }

// Vector<float> GraphNode::output() {
// 	return proc_out;
// }

// void GraphNode::print_proc() {
// 	proc.print();
// }

// void GraphNode::print_output() {
// 	proc_out.print();
// }

