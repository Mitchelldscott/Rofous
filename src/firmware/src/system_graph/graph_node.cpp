#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/graph_node.h"

#include "sensors/lsm6dsox.h"

GraphNode::GraphNode() {
	output_shape = 0;
	inputs.reset(0);
	proc_out.reset(0);
	setup_data.reset(0);
}

GraphNode::GraphNode(Process* p, int outputs, int configs, Vector<int> input_ids) {
	proc = p;
	output_shape = outputs;
	config_shape = configs;

	inputs = input_ids;
	proc_out.reset(0);
	setup_data.reset(0);
}

void GraphNode::setup_proc() {
	proc->setup(setup_data);
}

void GraphNode::run_proc(Vector<float>* input_buffer) {
	proc->run(input_buffer, &proc_out);
}

Vector<float>* GraphNode::output() {
	return &proc_out;
}

Vector<float>* GraphNode::config() {
	return &setup_data;
}

Vector<int>* GraphNode::input_ids() {
	return &inputs;
}

bool GraphNode::is_configured() {
	return config_shape == setup_data.size();
}

void GraphNode::print_proc() {
	proc->print();
}

void GraphNode::print_output() {
	Serial.print("\t");
	proc_out.print();
}

