#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/graph_node.h"
#include "system_graph/process_factory.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting Graph Node tests ===");

	float tmp[1] = {0.6};
	int cmf_inputs[2] = {1, 0};

	Process_Factory p_fact;
	GraphNode* nodelist[2];

	nodelist[0] = new GraphNode(p_fact.new_proc("LSM"), 0, Vector<int>(0));
	nodelist[1] = new GraphNode(p_fact.new_proc("CMF"), 1, Vector<int>(cmf_inputs, 2));

	Vector<float> input(1);

	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->print_proc();
	nodelist[0]->print_output();

	input.reset(3);
	input.append(nodelist[0]->output());
	Serial.print("CMF inputs\t"); input.print();
	bool status = nodelist[1]->run_proc(&input);
	Serial.printf("No setup Run status %i\n", status);
	Vector<float>* config = nodelist[1]->config();
	config->from_array(tmp, 1);
	nodelist[1]->setup_proc();
	status = nodelist[1]->run_proc(&input);
	Serial.printf("Setup Run status %i\n", status);
	nodelist[1]->print_proc();
	nodelist[1]->print_output();

	Serial.println("=== Finished Graph Node tests ===");
}