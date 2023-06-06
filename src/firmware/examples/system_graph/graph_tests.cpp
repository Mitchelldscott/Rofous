#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/graph_node.h"
#include "system_graph/process_factory.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting Graph Node tests ===");

	float tmp[1] = {0.4};
	int cmf_inputs[2] = {1, 0};

	Process_Factory p_fact;
	GraphNode* nodelist[2];

	nodelist[0] = new GraphNode(p_fact.new_proc("LSM"), 9, Vector<float>(tmp, 1));
	nodelist[1] = new GraphNode(p_fact.new_proc("CMF"), 3, Vector<int>(cmf_inputs, 2), Vector<float>(tmp, 1));

	Vector<float> input(1);

	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->print_proc();
	nodelist[0]->print_output();

	input.reset(3);
	input.append(nodelist[0]->output());
	nodelist[1]->run_proc(&input);
	nodelist[1]->print_proc();
	nodelist[1]->print_output();

	Serial.println("=== Finished Graph Node tests ===");
}