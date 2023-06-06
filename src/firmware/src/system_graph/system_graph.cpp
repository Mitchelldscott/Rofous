#include <Arduino.h>
#include "utilities/blink.h"
#include "utilities/vector.h"
#include "system_graph/process.h"
#include "system_graph/graph_node.h"
#include "system_graph/system_graph.h"

SystemGraph::SystemGraph() {
	lifetime = 0;
	watch.set(0);
	status.reset(0);
	nodes.reset(0);
	node_ids.reset(0);

	setup_blink();
}

void SystemGraph::collect_outputs(int index, Vector<float>* data) {
	data->reset(0);
	Vector<int> ids = *nodes[index]->input_ids();
	for (int i = 0; i < ids.size(); i++) {
		int node_id = node_ids.find(ids[i]);
		if (node_id > 0) {
			data->append(nodes[node_id]->output());			
		}
	}
}

void SystemGraph::add(String proc_id, int id, int n_outputs, int n_configs, Vector<int> inputs) {
	// status.push(0);
	node_ids.push(id);
	nodes.push(new GraphNode(factory.new_proc(proc_id), n_outputs, n_configs, inputs));
	if (n_configs == 0) {
		status.push(1);
	}
	else {
		status.push(0);
	}
	// Serial.printf("New node: %i\n", nodes.size());
}

void SystemGraph::update_config(int id, int chunk_id, Vector<float> config) {
	int node_index = node_ids.find(id);
	nodes[node_index]->config()->insert(config, chunk_id * config.size());
	if (nodes[node_index]->is_configured()) {
		status[node_index] = 1;
		nodes[node_index]->setup_proc();
	}
}

void SystemGraph::spin() {
	Vector<float> input(0);
	for (int i = 0; i < nodes.size(); i++) {
		collect_outputs(i, &input);
		if (status[i] == 1) {
			nodes[i]->run_proc(&input);
		}
		else {
			Serial.printf("Node %i not configured\n", node_ids[i]);
		}
	}
}

void SystemGraph::dump_all() {
	Serial.printf("lifetime: %f\n", lifetime);
	Serial.print("Node ids\n\t");
	node_ids.print();
	for (int i = 0; i < nodes.size(); i++) {
		nodes[i]->print_proc();
		nodes[i]->print_output();
	}
}

void SystemGraph::handle_hid() {
	report.write();
	lifetime += US_2_S(watch.delay_micros(0, 1E3));
	watch.set(0);
	Serial.printf("lifetime: %f\n", lifetime);
	switch (report.read()) {
		case 64:
			// Serial.printf("report %f\n", lifetime);
			blink();										// only blink when connected to hid
			switch (report.get(0)) {
				case 255:
					Serial.println("Packet 255");
					break;

				case 1:
					Serial.println("Packet 1");
					break;

				default:
					Serial.printf("Packet %i\n", report.get(0));
					break;
			}
			report.put_float(60, lifetime);
			// timer_set(3);
			break;
		
		default:
			// Serial.println("No report");
			break;
	}
}

void SystemGraph::init_process_hid() {
	String key = "";
	int id = report.get(1);
	key += char(report.get(2));
	key += char(report.get(3));
	key += char(report.get(4));
	int n_output = report.get(5);
	int n_config = report.get(6);
	int n_inputs = report.get(7);
	Vector<int> input_ids(n_inputs);
	for (int i = 0; i < n_inputs; i++) {
		input_ids[i] = report.get(i + 7);
	}
	add(key, id, n_output, n_config, input_ids);
}

void SystemGraph::config_process_hid() {
	int id = report.get(1);
	int chunk_num = report.get(2);
	int chunk_size = report.get(3);
	Vector<float> data(chunk_size);
	for (int i = 0; i < chunk_size; i++) {
		data[i] = report.get_float((4 * i) + 3);
	}
	update_config(id, chunk_num, data);
}
