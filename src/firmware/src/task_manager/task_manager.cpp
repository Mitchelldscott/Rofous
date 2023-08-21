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
Vector<FWTaskPacket*>* tc_queue;

Vector<int> node_ids(0);
Vector<TaskNode*> nodes(0);

Vector<FWTaskPacket*>* init_task_manager() {
	sys_timers.set(0);		// setup master cycle timer
	sys_timers.set(1);		// setup individual run timer
	nodes.reset(0);		// start with zero nodes
	node_ids.reset(0);	// len(nodes) always = len(node_ids) (soon to be execution order)

	tc_queue = enable_hid_interrupts(&node_ids, &nodes);
	return tc_queue;
}

int node_index(int task_id) {
	return node_ids.find(task_id);
}

Vector<float> collect_outputs(int index) {
	int curr_size = 0;
	Vector<float> data(nodes[index]->input_dim());
	for (int i = 0; i < nodes[index]->n_inputs(); i++) {
		int node_id = node_ids.find(nodes[index]->input_id(i));
		if (node_id >= 0) {
			int output_dim = nodes[node_id]->output_dim();
			data.insert(nodes[node_id]->output()->as_array(), curr_size, output_dim);
			curr_size += output_dim;
		}
	}
	return data;
}

void add_task(FWTaskPacket* task_init) {
	int index = node_index(task_init->task_id);
	if (index == -1) {	// Node does not exist yet
		// Add new node, node id
		node_ids.push(task_init->task_id);
		nodes.push(new TaskNode(new_task(task_init->key), task_init->n_inputs, task_init->inputs.as_array()));
		// printf("Pushed %i\n", task_init->task_id);
	}
	else { // Node exists so update it's params
		nodes[index]->set_config();
		nodes[index]->set_inputs(task_init->inputs.as_array(), task_init->n_inputs);
	}
}

void update_task(FWTaskPacket* task_params) {
	int node_idx = node_index(task_params->task_id);

	if (node_idx == -1) {
		printf("Node not found %i\n\tThis should never happen\n", task_params->task_id);
		return;
	}

	// This will set the config vector to however long it needs to be
	// If a config chunk is missed the task may still think it's configured
	nodes[node_idx]->config()->insert(task_params->parameters.as_array(),
										task_params->chunk_id * task_params->chunk_size, 
										task_params->chunk_size);


	if (nodes[node_idx]->is_configured()) {
		nodes[node_idx]->setup_task();
	}
}

void spin() {
	if (tc_queue->size() > 0) {
		FWTaskPacket* p = tc_queue->pop();
		// printf("New Packet %i\n", p->task_id);

		if (p->packet_type == 0) {
			add_task(p);
		}
		else {
			update_task(p);
		}
	}
	for (int i = 0; i < nodes.size(); i++) {
		Vector<float> input = collect_outputs(i);
		nodes[i]->run_task(&input);
	}
}

void dump_all_tasks() {
	printf("Task Manager lifetime: %f\n", sys_lifetime);
	printf("Node ids\n\t");
	node_ids.print();
	printf("Nodes\n\t");
	nodes.print();
}
