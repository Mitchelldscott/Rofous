#include "syncor/syncor.h"

SynCor::SynCor() {
	timers.set(0);		// setup master cycle timer
	timers.set(1);		// setup individual run timer
	nodes.reset(0);		// start with zero nodes
	node_ids.reset(0);	// len(nodes) always = len(node_ids) (soon to be execution order)

	setup_blink();		// state indicator
}

int SynCor::node_index(int proc_id) {
	return node_ids.find(proc_id);
}

Vector<float> SynCor::collect_outputs(int index) {
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

void SynCor::add(HidProcessParams* proc_params) {
	// status.push(0);
	int index = node_index(proc_params->proc_id);
	if (index == -1) {	// Node does not exist yet
		// Add new node, node id
		node_ids.push(proc_params->proc_id);
		nodes.push(new SynCorNode(p_factory.new_proc(proc_params->proc_key), proc_params->n_configs, proc_params->n_inputs, proc_params->inputs));
	}
	else { // Node exists so update it's params
		nodes[index]->set_config(proc_params->n_configs);
		nodes[index]->set_inputs(proc_params->inputs, proc_params->n_inputs);
	}
}

void SynCor::update_config(HidProcessConfig proc_config) {
	int node_index = node_index(proc_config->proc_id);

	if (node_index == -1) {
		Serial.printf("Node not found %i\n\tThis should never happen\n", proc_config->proc_id);
		return;
	}

	// This will set the config vector to however long it needs to be
	// If a config chunk is missed the process may still think it's configured
	nodes[node_index]->config()->insert(proc_config->config, 
										proc_config->chunk_id * proc_config->n_configs, 
										proc_config->n_configs);	
	
	if (nodes[node_index]->is_configured()) {
		nodes[node_index]->setup_proc();
	}
}

void SynCor::spin() {
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i]->is_configured()) {
			Vector<float> input = collect_outputs(i);
			nodes[i]->run_proc(&input);
			
		}
		else {
			nodes[i]->setup_proc();
		}
	}
}

void SynCor::dump_all() {
	Serial.printf("SynCor lifetime: %f\n", lifetime);
	Serial.print("Node ids\n\t");
	node_ids.print();
	for (int i = 0; i < nodes.size(); i++) {
		nodes[i]->print_proc();
		nodes[i]->print_output();
	}
}
