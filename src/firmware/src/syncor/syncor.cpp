#include "syncor/syncor.h"

SynCor::SynCor() {
	timers.set(1);
	lifetime = 0;
	timers.set(0);
	status.reset(0);
	nodes.reset(0);
	node_ids.reset(0);

	setup_blink();
}

// void SynCor::enable_hid_interrupts() {
// 	hidtimer.begin(handle_hid, HID_READ_WRITE_RATE_US);
// }

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

void SynCor::add(HidProcessParams* proc_info) {
	// status.push(0);
	int index = node_ids.find(proc_info->proc_id);
	if (index == -1) {
		node_ids.push(proc_info->proc_id);
		nodes.push(new SynCorNode(factory.new_proc(proc_info->proc_key), proc_info->n_configs, proc_info->n_inputs, proc_info->inputs));

		if (n_configs == 0) {
			status.push(1);
		}
		else {
			status.push(0);
		}
	}
	else {
		nodes[index]->set_config(proc_info->n_configs);
		nodes[index]->set_inputs(proc_info->inputs, proc_info->n_inputs);
	}
}

void SynCor::update_config(HidProcessConfig) {
	int node_index = node_ids.find(proc_info->proc_id);
	status[node_index] = 0;

	if (node_index == -1) {
		Serial.printf("Node not found %i\n", proc_info->proc_id);
		return;
	}

	nodes[node_index]->config()->insert(proc_info->config, proc_info->chunk_id * proc_info->n_configs, proc_info->n_configs);	
	if (nodes[node_index]->is_configured()) {
		nodes[node_index]->setup_proc();
		status[node_index] = 1;
	}
}

void SynCor::spin() {
	for (int i = 0; i < nodes.size(); i++) {
		if (status[i]) {
			Vector<float> input = collect_outputs(i);
			nodes[i]->run_proc(&input);
		}
		else if (nodes[i]->is_configured()) {
			nodes[i]->setup_proc();
			status[i] = 1;
		}
		else {
			Serial.printf("Process %i is not configured\n", i);
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
