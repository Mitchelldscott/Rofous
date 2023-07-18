#include <Arduino.h>
#include "utilities/blink.h"
#include "utilities/vector.h"
#include "syncor/process.h"
#include "syncor/syncor_node.h"
#include "syncor/syncor.h"

SynCor::SynCor() {
	lifetime = 0;
	watch.set(0);
	status.reset(0);
	nodes.reset(0);
	node_ids.reset(0);

	setup_blink();
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

void SynCor::add(String proc_id, int id, int n_configs, int n_inputs, int* inputs) {
	// status.push(0);
	int index = node_ids.find(id);
	if (index == -1) {
		node_ids.push(id);
		nodes.push(new SynCorNode(factory.new_proc(proc_id), n_configs, n_inputs, inputs));

		if (n_configs == 0) {
			status.push(1);
		}
		else {
			status.push(0);
		}
	}
	else {
		nodes[index]->set_config(n_configs);
		nodes[index]->set_inputs(inputs, n_inputs);
		// nodes[index]->set_process(factory.new_proc(proc_id)); // Doesn't seem to reinit imu but it should... imu init takes forever though so whatever
	}
}

void SynCor::update_config(int id, int chunk_id, int n_configs, float* config) {
	int node_index = node_ids.find(id);
	status[node_index] = 0;

	if (node_index == -1) {
		Serial.printf("Node not found %i\n", id);
		return;
	}

	nodes[node_index]->config()->insert(config, chunk_id * n_configs, n_configs);	
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

void SynCor::handle_hid() {
	switch (report.read()) {
		case 64:
			blink();										// only blink when connected to hid
			// Serial.printf("[%f]Report %i, %i, %i\n", lifetime, report.get(0), report.get(1), report.get(2));
			switch (report.get(0)) {
				case 255:
					// report.print();
					switch (report.get(1)) {
						case 1:
							init_process_hid();
							break;

						case 2:
							config_process_hid();
							break;

						default:
							break;
					}
					lifetime = 0;
					break;

				case 1:
					switch (report.get(1)) {
						case 1:
							dump_vector(nodes[report.get(2)]->context());
							break;

						case 2:
							dump_vector(nodes[report.get(2)]->output());
							break;

						default:
							break;
					}
					break;

				default:
					break;
			}
			break;
		
		default:
			// Serial.println("No report");
			break;
	}
	lifetime += US_2_S(watch.delay_micros(0, 1E3));
	report.put_float(60, lifetime);
	report.write();
	watch.set(0);
}

void SynCor::init_process_hid() {
	String key = "";
	int id = report.get(2);
	key += char(report.get(3));
	key += char(report.get(4));
	key += char(report.get(5));
	int n_config = report.get(6);
	int n_inputs = report.get(7);

	Serial.printf("new process %i: ", id);
	Serial.print(key);
	Serial.printf(" %i %i\n", n_config, n_inputs);

	int input_ids[n_inputs];
	for (int i = 0; i < n_inputs; i++) {
		input_ids[i] = report.get(i + 8);
	}
	add(key, id, n_config, n_inputs, input_ids);
}

void SynCor::config_process_hid() {
	int id = report.get(2);
	int chunk_num = report.get(3);
	int chunk_size = report.get(4);
	Serial.printf("configure %i, %i, %i\n",id, chunk_num, chunk_size);

	float data[chunk_size];
	for (int i = 0; i < chunk_size; i++) {
		data[i] = report.get_float((4 * i) + 5);
	}
	update_config(id, chunk_num, chunk_size, data);
}

void SynCor::dump_vector(Vector<float>* data) {
	report.put(3, data->size());
	for (int i = 0; i < data->size(); i++){
		report.put_float((4 * i) + 4, (*data)[i]);
	}
}
