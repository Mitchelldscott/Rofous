#include <Arduino.h>

#include "utilities/vector.h"
#include "syncor/syncor_node.h"

#include "sensors/lsm6dsox.h"

SynCorNode::SynCorNode() {
	/*
		Default constructor for a SynCorNode
		Sets everything to 0 (values, shapes: inputs, outputs, context and config data)
	*/
	inputs.reset(0);
	output_buffer.reset(0);
	config_buffer.reset(0);
}

SynCorNode::SynCorNode(Process* p, int n_configs, int n_inputs, int* input_ids) {
	/*
		SynCorNode constructor
		@param
			p: (Process*) The driving process for this node
			n_configs: (int) number of config values, size of buffer (always floats)
			n_inputs: (int) number of input nodes
			input_ids: (int*) the identifiers of input nodes (in order of concatenation),
				does not specify the process index in the syncor list but a unique 
				process id associated with each process.
	*/
	set_process(p);
	set_config(n_configs);
	set_inputs(input_ids, n_inputs);
}

int SynCorNode::n_inputs() {
	/*
		Get the number of input nodes.
		@return
			size: (int) number of nodes to recieve input from.
	*/
	return inputs.size();
}

int SynCorNode::input_dim() {
	/*
		Get the size of desired input buffer.
		@return
			size: (int) user defined size of input vector.
	*/
	return proc->input_dim();
}

int SynCorNode::context_dim() {
	/*
		Get the size of desired context buffer.
		@return
			size: (int) user defined size of context vector.
	*/
	return proc->context_dim();
}

int SynCorNode::output_dim() {
	/*
		Get the size of desired output buffer.
		@return
			size: (int) user defined size of output vector.
	*/
	return proc->output_dim();
}

int SynCorNode::input_id(int index) {
	/*
		Get an id at index from the input id list
		@return
			id: (int) id of input at index
	*/
	return inputs[index];
}

bool SynCorNode::is_configured() {
	/*
		Check if the process is configured. Needed because
		setup data may be sent in chunks.
		@return
			status: (bool) if process is configured
	*/
	return config_shape == config_buffer.size();
}

Vector<float>* SynCorNode::output() {
	/*
		Get a pointer to the output buffer
		@return
			output: (Vector<float>*) buffer of output data
	*/
	return &output_buffer;
}

Vector<float>* SynCorNode::config() {
	/*
		Get a pointer to the setup buffer
		@return
			config: (Vector<float>*) buffer of setup data
	*/
	return &config_buffer;
}

Vector<float>* SynCorNode::context() {
	/*
		Get a pointer to the context buffer
		@return
			context: (Vector<float>*) buffer of context data
	*/
	proc->context(&context_buffer);
	return &context_buffer;
}

void SynCorNode::set_config(int configs) {
	/*
		Sets the shape of the config buffer, this is
		called when setting up a process or new node.
		Will also zero the current config to ensure reinitialization.
		@param
			configs: (int) number of config values (floats)
	*/
	config_shape = configs;
	config_buffer.reset(0);
}

void SynCorNode::set_inputs(int* input_ids, int n_inputs) {
	/*
		Set the unique IDs of all input nodes.
		@param
			input_ids: (int*) unique IDs of input nodes.
			n_inputs: (int) number of input nodes.
	*/
	inputs.from_array(input_ids, n_inputs);
}

void SynCorNode::set_process(Process* process) {
	/*
		Set the process that drives (input, config) -> (context, output).
		Also resets the process and output vector to ensure initial
		state is the zero state.
		@param
			process: (Process) User defined process.
	*/
	proc = process;
	proc->reset();
	output_buffer.reset(proc->output_dim());
}

bool SynCorNode::setup_proc() {
	/*
		Call the processes user defined setup function with the config buffer.
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

bool SynCorNode::run_proc(Vector<float>* input_buffer) {
	/*
		Call the processes user defined run function. Does nothing if not configured
		or if input size (generated) is not equal to input dimension (user defined).
		@param
			input_buffer: (Vector<float>*) concatenated outputs of processes
				listed in input_ids
		@return
			status: (bool) if run was called.
	*/
	if (is_configured() && input_buffer->size() == proc->input_dim()) {
		proc->run(input_buffer, &output_buffer);
		return true;		
	}
	return false;
}

void SynCorNode::print_proc() {
	/*
		Dump all process info. Requires that the user defined a print
		for their process.
	*/
	proc->print();
}

void SynCorNode::print_output() {
	/*
		Dump output buffer.
	*/
	Serial.print("\t");
	output_buffer.print();
}

