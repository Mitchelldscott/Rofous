#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/process.h"

// Drivers
#include "sensors/lsm6dsox.h"

Process::Process() {
	Serial.println("Constructing PBO");
}

void Process::reset() {
	Serial.println("Resetting PBO");
}

void Process::clear() {
	Serial.println("Clearing PBO");
}

void Process::print() {
	Serial.println("Printing PBO");
}

Vector<float> Process::context() {
	Serial.println("Acessing PBO context");
	Vector<float> v(1, 1);
	return v;
}

void Process::setup(Vector<float> config) {
	Serial.println("Initalizing PBO");
	config.print();
}

Vector<Vector<float>> Process::run(Vector<Vector<float>> input) {
	Serial.println("Running PBO");
	return input;
}





