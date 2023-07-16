#include <Arduino.h>

#include "utilities/vector.h"
#include "syncor/syncor_node.h"
#include "syncor/process_factory.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting SynCorNode tests ===");

	int empty = 0;
	float tmp[1] = {0.6};
	int cmf_inputs[2] = {1, 0};


	Serial.println("=== Init Process Factory ===");
	Process_Factory p_fact;

	Serial.println("=== Init Graph Node ===");
	SynCorNode* nodelist[2];

	Serial.println("=== Add LSM6DSOX ===");
	nodelist[0] = new SynCorNode(p_fact.new_proc("LSM"), 0, 0, &empty);

	Serial.println("=== Add Complimentary Filter ===");
	nodelist[1] = new SynCorNode(p_fact.new_proc("CMF"), 1, 2, cmf_inputs);

	Serial.println("=== Init input Vector ===");
	Vector<float> input(0);

	Serial.println("=== Run Process 0 ===");
	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->run_proc(&input);
	nodelist[0]->print_proc();
	nodelist[0]->print_output();

	Serial.println("=== Setup Process 1 ===");
	input.reset(0);
	input.append(nodelist[1]->output());				// get inputs from the nodes, this case input(0:3) is empty
	input.append(nodelist[0]->output());				// Add output of nodelist[0] to inputs, append is a heavy function don't use in runtime
	Serial.print("CMF inputs\t"); input.print();		// display the inputs
	
	bool status = nodelist[1]->run_proc(&input);		// try to run the process without calling setup
	Serial.printf("No setup Run status %i\n", status);
	
	Vector<float>* config = nodelist[1]->config();		// Get the config buffer
	config->from_array(tmp, 1);							// set the config, make sure if the config gets filled setup is called (config.size() is how configuration is checked)
	nodelist[1]->setup_proc();							// call setup to initialize the process	
	status = nodelist[1]->run_proc(&input);				// Run the process with setup
	
	Serial.printf("Setup Run status %i\n", status);
	nodelist[1]->print_proc();
	nodelist[1]->print_output();


	Serial.println("=== Reconfigure and Setup Process 1 ===");
	nodelist[1]->set_config(1);

	status = nodelist[1]->run_proc(&input);				// try to run the process without calling setup
	Serial.printf("No setup Run status %i\n", status);

	config = nodelist[1]->config();						// Get the config buffer
	config->from_array(tmp, 1);							// set the config, make sure if the config gets filled setup is called (config.size() is how configuration is checked)
	nodelist[1]->setup_proc();							// call setup to initialize the process

	input.reset(0);
	input.append(nodelist[1]->output());				// get inputs from the nodes, this case input(0:3) is empty
	input.append(nodelist[0]->output());				// Add output of nodelist[0] to inputs, append is a heavy function don't use in runtime
	status = nodelist[1]->run_proc(&input);				// Run the process with setup
	
	Serial.printf("Setup Run status %i\n", status);
	nodelist[1]->print_proc();
	nodelist[1]->print_output();

	Serial.println("=== Finished Graph Node tests ===");
}