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

Vector<int> node_ids(0);
Vector<TaskNode*> nodes(0);
CommsPipeline* pipeline_internal;

CommsPipeline* init_task_manager() {
	sys_timers.set(0);		// setup master cycle timer
	sys_timers.set(1);		// setup individual run timer

	nodes.reset(0);		// start with zero nodes

	pipeline_internal = enable_hid_interrupts();
	return pipeline_internal;
}

int node_index(int id) {
	return node_ids.find(id);
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
		nodes.push(new TaskNode(new_task(task_init->key), task_init->n_inputs, task_init->inputs.as_array()));
		node_ids.push(task_init->task_id);
		index = nodes.size() - 1;

		TaskFeedback* task_fb = new TaskFeedback;
		task_fb->task_id = task_init->task_id;
		task_fb->latch = 0;
		task_fb->timestamp = 0;
		task_fb->output.reset(0);
		pipeline_internal->feedback.push(task_fb);
		
		// printf("Pushed %i\n", task_init->task_id);
	}
	else { // Node exists so update it's params (and deconfig)
		nodes[index]->set_task(new_task(task_init->key));
		nodes[index]->set_inputs(task_init->inputs.as_array(), task_init->n_inputs);
	}

	nodes[index]->latch(0); // always unlatch here

}

void update_task(TaskSetupPacket* task_params) {
	int node_idx = node_index(task_params->task_id);

	if (node_idx == -1) {
		printf("Node not found %i\n\tThis should never happen\n", task_params->task_id);
		return;
	}

	// This will set the config vector to however long it needs to be
	// If a config chunk is missed the task may still think it's configured
	(*nodes[node_idx])[PARAM_DIMENSION]->insert(task_params->parameters.as_array(),
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

	// need to send another overwrite to unlatch
	*(*nodes[node_idx])[OUTPUT_DIMENSION] = task_update->data;
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
	pipeline_internal->feedback[i]->latch = nodes[i]->is_latched();
	pipeline_internal->feedback[i]->output = *(*nodes[i])[OUTPUT_DIMENSION];
	pipeline_internal->feedback[i]->timestamp = pipeline_internal->lifetime + sys_timers.secs(0);
	interrupts();
}

void spin() {
	// handle queued setup
	task_setup_handler();

	// handle task execution
	for (int i = 0; i < nodes.size(); i++) {
		// If task isn't fully linked to inputs this will link them (if the input tasks exist)
		if (link_nodes(i)) {
			// printf("Linked: %i\tc %i\ti %i\tl %i\n", i, nodes[i]->is_configured(), nodes[i]->n_inputs(), nodes[i]->n_links());
			// pulls outputs from input tasks and runs the current task
			sys_timers.set(1);
			if (nodes[i]->run_task()){
				// printf("Ran: %i %i\n", i, nodes[i]->is_latched());
				// put task output, context in the pipeline for publishing
				// if (i == 2) {
				// 	(*nodes[i])[OUTPUT_DIMENSION]->print();
				// }
				task_publish_handler(i);
			}
		}

		noInterrupts();
		pipeline_internal->lifetime += sys_timers.secs(0); // update lifetime everytime a task is run
		sys_timers.set(0);
		interrupts();
	}
}

void dump_all_tasks() {
	printf("Task Manager lifetime: %f\n", sys_lifetime);
	printf("Node ids\n\t");
	node_ids.print();
	printf("Nodes\n\t");
	nodes.print();
}
