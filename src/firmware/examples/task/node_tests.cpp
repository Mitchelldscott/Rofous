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
#include <Arduino.h>
#include "utilities/timing.h"
#include "utilities/splash.h"
#include "utilities/assertions.h"
#include "task_manager/task_node.h"
#include "task_manager/task_factory.h"

FTYK timers;

int main() {

	unit_test_splash("Task Node", 0);

	int empty = 0;
	float tmp[1] = {0.6};
	int cmf_inputs[2] = {1, 0};

	printf("=== Init task Factory ===\n");

	printf("=== Init Graph Node ===\n");
	TaskNode* nodelist[2];

	printf("=== Add LSM6DSOX ===\n");
	timers.set(0);
	nodelist[0] = new TaskNode(new_task("LSM"), 0, &empty);
	assert_leq<float>(timers.micros(0), 2, "LSM new node timer");

	printf("=== Add Complimentary Filter ===\n");
	timers.set(0);
	nodelist[1] = new TaskNode(new_task("CMF"), 2, cmf_inputs);
	assert_leq<float>(timers.micros(0), 2, "CMF new node timer");

	printf("=== Init input Vector ===\n");
	Vector<float> input(0);

	printf("=== Run task 0 ===\n");
	nodelist[0]->run_task(&input);
	nodelist[0]->run_task(&input);
	nodelist[0]->run_task(&input);
	nodelist[0]->print();
	nodelist[0]->print_output();

	printf("=== Setup task 1 ===\n");
	input.reset(0);
	input.append(nodelist[1]->output());				// get inputs from the nodes, this case input(0:3) is empty
	input.append(nodelist[0]->output());				// Add output of nodelist[0] to inputs, append is a heavy function don't use in runtime
	printf("CMF inputs\t"); input.print();				// display the inputs
	
	bool status = nodelist[1]->run_task(&input);		// try to run the task without calling setup
	printf("No setup Run status %i\n", status);
	
	Vector<float>* config = nodelist[1]->config();		// Get the config buffer
	config->from_array(tmp, 1);							// set the config, make sure if the config gets filled setup is called (config.size() is how configuration is checked)
	nodelist[1]->setup_task();							// call setup to initialize the task	
	status = nodelist[1]->run_task(&input);				// Run the task with setup
	
	printf("Setup Run status %i\n", status);
	nodelist[1]->print();
	nodelist[1]->print_output();

	printf("=== Reconfigure and Setup task 1 ===\n");
	nodelist[1]->set_config();

	status = nodelist[1]->run_task(&input);				// try to run the task without calling setup
	printf("No setup Run status %i\n", status);

	config = nodelist[1]->config();						// Get the config buffer
	config->from_array(tmp, 1);							// set the config, make sure if the config gets filled setup is called (config.size() is how configuration is checked)
	nodelist[1]->setup_task();							// call setup to initialize the task

	input.reset(0);
	input.append(nodelist[1]->output());				// get inputs from the nodes, this case input(0:3) is empty
	input.append(nodelist[0]->output());				// Add output of nodelist[0] to inputs, append is a heavy function don't use in runtime
	status = nodelist[1]->run_task(&input);				// Run the task with setup
	
	printf("Setup Run status %i\n", status);
	nodelist[1]->print();
	nodelist[1]->print_output();

	printf("=== Finished Graph Node tests ===\n");
}