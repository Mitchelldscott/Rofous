#include <Arduino.h>
#include "utilities/vector.h"
#include "system_graph/process.h"

// Drivers
#include "sensors/lsm6dsox.h"

void Process::reset() {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
	*/
	Serial.println("PBO Reset");
}

void Process::clear() {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
	*/
	Serial.println("PBO Clear");
}

void Process::print() {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
	*/
	Serial.println("PBO Print");
}

void Process::setup(Vector<float>* config) {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
		@param
			config: (Vector<float>) Vector of configuration data, 
				organization is handled by user
	*/
	Serial.println("PBO Setup");
	config->print();
}

void Process::context(Vector<float>* context) {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
		@param
			context: (Vector<float>*) empty vector to fill with the context
	*/
	Serial.println("Requested PBO context");
	context->reset(0);
}

void Process::run(Vector<float>* input, Vector<float>* output) {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
		@param
			input: (Vector<float>*) flattened Vector of input data for process
			output: (Vector<float>*) flattened Vector of output data from process
	*/
	Serial.println("PBO Run");
	output->reset(0);
}

bool Process::operator==(const Process& p) {
	/*
		Base implementation of process functions.
		Only defined to show user when inheritance has issues.
		@param
			
	*/
	Serial.println("PBO Comparison");
	return false;
}





