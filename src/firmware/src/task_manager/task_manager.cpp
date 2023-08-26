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

#include "task_manager/task_manager.h"

FTYK sys_timers;
float sys_lifetime = 0;

Vector<TaskNode*> nodes(0);
CommsPipeline* pipeline_internal;

CommsPipeline* init_task_manager() {
	sys_timers.set(0);		// setup master cycle timer
	sys_timers.set(1);		// setup individual run timer

	nodes.reset(0);		// start with zero nodes

	pipeline_internal = enable_hid_interrupts();
	return pipeline_internal;
}

int node_index(int task_id) {
	return pipeline_internal->ids.find(task_id);
}

bool link_nodes(int index) {
	for (int i = nodes[index]->n_links(); i < nodes[index]->n_inputs(); i++) {
		int node_idx = node_index(nodes[index]->input_id(i));
		if (node_idx >= 0) {
			nodes[index]->link_input(nodes[node_idx]);
		}
		else {
			return false;
		}
	}
	return true;
}

void add_task(TaskSetupPacket* task_init) {
	int index = node_index(task_init->task_id);
	if (index == -1) {	// Node does not exist yet
		// Add new node, node id
		TaskFeedback* task_fb = new TaskFeedback;
		task_fb->task_id = task_init->task_id;
		task_fb->timestamp = 0;
		task_fb->output.reset(0);
		task_fb->context.reset(0);
		pipeline_internal->ids.push(task_init->task_id);
		pipeline_internal->feedback.push(task_fb);
		nodes.push(new TaskNode(new_task(task_init->key), task_init->n_inputs, task_init->inputs.as_array()));
		// printf("Pushed %i\n", task_init->task_id);
	}
	else { // Node exists so update it's params (and deconfig)
		nodes[index]->reset_config();
		nodes[index]->set_inputs(task_init->inputs.as_array(), task_init->n_inputs);
	}

}

void update_task(TaskSetupPacket* task_params) {
	int node_idx = node_index(task_params->task_id);

	if (node_idx == -1) {
		printf("Node not found %i\n\tThis should never happen\n", task_params->task_id);
		return;
	}

	// This will set the config vector to however long it needs to be
	// If a config chunk is missed the task may still think it's configured
	(*nodes[node_idx])[PARAMS_DIMENSION]->insert(task_params->parameters.as_array(),
										task_params->chunk_id * task_params->chunk_size, 
										task_params->chunk_size);

	nodes[node_idx]->setup_task();
}

void overwrite_task(TaskSetupPacket* task_update) {
	int node_idx = node_index(task_update->task_id);
	if (node_idx == -1) {
		printf("Node not found %i\n\tThis should never happen\n", task_update->task_id);
		return;
	}

	// context_alt + 1 = CONTEXT_DIMENSION (if context_alt = 0)
	// context_alt + 1 = OUTPUT_DIMENSION (if context_alt = 1)
	// need to send another overwrite to unlatch
	*(*nodes[node_idx])[task_update->context_alt] = task_update->data;
	nodes[node_idx]->latch(task_update->latch);
}

void task_setup_handler() {
	noInterrupts();
	int n_items = pipeline_internal->setup_queue.size();
	interrupts();

	for (int i = 0; i < n_items; i++) {
		noInterrupts();
		TaskSetupPacket* p = pipeline_internal->setup_queue.pop();
		interrupts();

		switch (p->packet_type) {
			case 0:
				add_task(p);
				break;

			case 1:
				update_task(p);
				break;

			case 2:
				overwrite_task(p);
				break;

			default:
				break;
		}
		
		delete p;
	}
}

void task_publish_handler(int i) {
	noInterrupts();
	pipeline_internal->feedback[i]->output = *(*nodes[i])[OUTPUT_DIMENSION];
	pipeline_internal->feedback[i]->context = *(*nodes[i])[CONTEXT_DIMENSION];
	pipeline_internal->feedback[i]->timestamp = sys_lifetime + sys_timers.secs(0);
	interrupts();
}

void spin() {
	// handle queued setup
	task_setup_handler();

	// handle task execution
	for (int i = 0; i < nodes.size(); i++) {
		// If task isn't fully linked to inputs this will link them (if the input tasks exist)
		if (link_nodes(i)) {
			// pulls outputs from input tasks and runs the current task
			if (nodes[i]->run_task()){
				// put task output, context in the pipeline for publishing
				task_publish_handler(i);
			}
		}
	}

	sys_lifetime += sys_timers.secs(0);
	sys_timers.set(0);
}

void dump_all_tasks() {
	printf("Task Manager lifetime: %f\n", sys_lifetime);
	printf("Node ids\n\t");
	pipeline_internal->ids.print();
	printf("Nodes\n\t");
	nodes.print();
}
