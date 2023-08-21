/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

// #include <Arduino.h>

// #include "utilities/vector.h"
#include "task_manager/task_node.h"

// #include "sensors/lsm6dsox.h"

TaskNode::TaskNode() {
	/*
		Default constructor for a TaskNode
		Sets everything to 0 (values, shapes: inputs, outputs, context and config data)
	*/
	inputs.reset(0);
	output_buffer.reset(0);
	config_buffer.reset(0);
}

TaskNode::TaskNode(Task* p, int n_inputs, int* input_ids) {
	/*
		TaskNode constructor
		@param
			p: (Tasks*) The driving tasks for this node
			n_inputs: (int) number of input nodes
			input_ids: (int*) the identifiers of input nodes (in order of concatenation),
				does not specify the tasks index in the syncor list but a unique 
				tasks id associated with each tasks.
	*/
	
	set_task(p);
	set_config();
	set_inputs(input_ids, n_inputs);
}

int TaskNode::n_inputs() {
	/*
		Get the number of input nodes.
		@return
			size: (int) number of nodes to recieve input from.
	*/
	return inputs.size();
}

int TaskNode::input_dim() {
	/*
		Get the size of desired input buffer.
		@return
			size: (int) user defined size of input vector.
	*/
	return task->input_dim();
}

int TaskNode::context_dim() {
	/*
		Get the size of desired context buffer.
		@return
			size: (int) user defined size of context vector.
	*/
	return task->context_dim();
}

int TaskNode::output_dim() {
	/*
		Get the size of desired output buffer.
		@return
			size: (int) user defined size of output vector.
	*/
	return task->output_dim();
}

int TaskNode::input_id(int index) {
	/*
		Get an id at index from the input id list
		@return
			id: (int) id of input at index
	*/
	return input_ids[index];
}

bool TaskNode::is_configured() {
	/*
		Check if the tasks is configured. Needed because
		setup data may be sent in chunks.
		@return
			status: (bool) if tasks is configured
	*/
	return task->params_dim() == config_buffer.size();
}

Vector<float>* TaskNode::output() {
	/*
		Get a pointer to the output buffer
		@return
			output: (Vector<float>*) buffer of output data
	*/
	return &output_buffer;
}

Vector<float>* TaskNode::config() {
	/*
		Get a pointer to the setup buffer
		@return
			config: (Vector<float>*) buffer of setup data
	*/
	return &config_buffer;
}

Vector<float>* TaskNode::context() {
	/*
		Get a pointer to the context buffer
		@return
			context: (Vector<float>*) buffer of context data
	*/
	task->context(&context_buffer);
	return &context_buffer;
}

void TaskNode::set_config() {
	/*
		Zero the current config to ensure reinitialization.

	*/
	task->reset();
}

void TaskNode::set_inputs(int* inputids, int n_inputs) {
	/*
		Set the unique IDs of all input nodes.
		@param
			input_ids: (int*) unique IDs of input nodes.
			n_inputs: (int) number of input nodes.
	*/
	input_ids.from_array(inputids, n_inputs);
}

void TaskNode::set_task(Task* new_task) {
	/*
		Set the task that drives (input, config) -> (context, output).
		Also resets the task and output vector to ensure initial
		state is the zero state.
		@param
			task: (Task) User defined task.
	*/
	task = new_task;
	task->reset();
	output_buffer.reset(task->output_dim());
}

bool TaskNode::setup_task() {
	/*
		Call the tasks user defined setup function with the config buffer.
		Does nothing when not configured.
		@return
			status: (bool) if setup was called.
	*/
	if (is_configured()) {
		task->setup(&config_buffer);
		return true;		
	}
	return false;
}

bool TaskNode::run_task(Vector<float>* input_buffer) {
	/*
		Call the tasks user defined run function. Does nothing if not configured
		or if input size (generated) is not equal to input dimension (user defined).
		@param
			input_buffer: (Vector<float>*) concatenated outputs of tasks
				listed in input_ids
		@return
			status: (bool) if run was called.
	*/

	if (is_configured() && input_buffer->size() == task->input_dim()) {
		task->run(input_buffer, &output_buffer);
		return true;		
	}

	return false;
}

void TaskNode::print() {
	/*
		Dump all task info. Requires that the user defined a print
		for their task.
	*/
	task->print();
}

void TaskNode::print_output() {
	/*
		Dump output buffer.
	*/
	printf("\t");
	output_buffer.print();
}

// define the template specialization
// for fancy printing
template <> void Vector<TaskNode*>::print() {
	if (length == 0) {
		printf("Task Vector [empty]\n");
		return;
	}

	printf("Task Vector [%i]: [\n", length);
	for (int i = 0; i < length-1; i++) {
		buffer[i]->print();
	}
	buffer[length-1]->print();
	printf("]\n");
}