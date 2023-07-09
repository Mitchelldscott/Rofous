#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/graph_node.h"

#include "sensors/lsm6dsox.h"

GraphNode::GraphNode() {
	/*
		Default constructor for a GraphNode
		Sets everything to 0 (not values, shapes: zero inputs, zero outputs and zero config data)
	*/
	inputs.reset(0);
	output_buffer.reset(0);
	config_buffer.reset(0);
}

GraphNode::GraphNode(Process* p, int configs, int n_inputs, int* input_ids) {
	/*
		GraphNode constructor
		@param
			p: (Process*) The driving process for this node
			configs: (int) number of config values, size of buffer (always floats)
			input_ids: (Vector<int>) the identifiers of input nodes (in order of concatenation),
				does not specify the process index but a unique process id associated with each process
	*/
	proc = p;
	proc->reset();
	config_shape = configs;

	inputs.from_array(input_ids, n_inputs);
	output_buffer.reset(0);
	config_buffer.reset(0);
}

void GraphNode::set_config(int configs) {
	config_shape = configs;
}

void GraphNode::set_inputs(Vector<int> input_ids) {
	inputs = input_ids;
}

void GraphNode::set_process(Process* process) {
	proc = process;
}

bool GraphNode::setup_proc() {
	/*
		Call the processes setup function with the config buffer
		Does nothing when not configured.
		@return
			status: (bool) if setup was called.
	*/
	if (is_configured()) {
		proc->setup(&config_buffer);
		return true;		
	}
	return false;
}

bool GraphNode::run_proc(Vector<float>* input_buffer) {
	/*
		Call the processes run function. Does nothing if not configured
		@param
			input_buffer: (Vector<float>*) concatenated outputs of processes
				listed in input_ids
		@return
			status: (bool) if run was called.
	*/
	if (is_configured()) {
		proc->run(input_buffer, &output_buffer);
		return true;		
	}
	return false;
}

Vector<float>* GraphNode::output() {
	/*
		Get a pointer to the output buffer
		@return
			output: (Vector<float>*) buffer of output data
	*/
	return &output_buffer;
}

Vector<float>* GraphNode::config() {
	/*
		Get a pointer to the setup buffer
		@return
			config: (Vector<float>*) buffer of setup data
	*/
	return &config_buffer;
}

Vector<float>* GraphNode::context() {
	/*
		Get a pointer to the setup buffer
		@return
			config: (Vector<float>*) buffer of setup data
	*/
	proc->context(&context_buffer);
	return &context_buffer;
}

Vector<int>* GraphNode::input_ids() {
	/*
		Get a pointer to the input_ids buffer
		@return
			input_ids: (Vector<float>*) buffer of input ids
	*/
	return &inputs;
}

bool GraphNode::is_configured() {
	/*
		Check if the process is configured. Needed because
		setup data may be sent in chunks.
		@return
			status: (bool) if process is configured
	*/
	return config_shape == config_buffer.size();
}

void GraphNode::print_proc() {
	proc->print();
}

void GraphNode::print_output() {
	Serial.print("\t");
	output_buffer.print();
}

