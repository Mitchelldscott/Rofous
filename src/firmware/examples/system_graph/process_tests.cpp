#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/process.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting Process tests ===");

	Process<LSM6DSOX> p;

	p.clear();
	p.reset();
	p.print();

	float inputs[1] = {0};
	p.run(inputs);
	p.run(inputs);
	p.run(inputs);
	p.state();

	p.print();

	Serial.println("=== Finished Process tests ===");	
}