#include <Arduino.h>
#include "utilities/vector.h"
#include "system_graph/process.h"

// Drivers
#include "sensors/lsm6dsox.h"

void Process::reset() {
	Serial.println("Resetting PBO");
}

void Process::clear() {
	Serial.println("Clearing PBO");
}

void Process::print() {
	Serial.println("Printing PBO");
}

void Process::context(Vector<float>* state) {
	Serial.println("Acessing PBO context");
}

void Process::setup(Vector<float> config) {
	Serial.println("Initalizing PBO");
	config.print();
}

void Process::run(Vector<float>* input, Vector<float>* output) {
	Serial.println("Running PBO");
}





