#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/process.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting PBO tests ===");

	Process p;

	p.clear();
	p.reset();
	p.print();

	Vector<Vector<float>> inputs(1, 1);

	p.run(inputs);
	p.run(inputs);
	Vector<Vector<float>> output = p.run(inputs);

	Serial.println("Process outputs:");
	output.print();

	Serial.println("Process State:");
	Vector<float> state = p.context();
	state.print();

	p.print();

	Serial.println("=== Starting LSM6DSOX tests ===");

	LSM6DSOX imu;

	imu.clear();
	imu.reset();
	imu.print();

	imu.run(inputs);
	imu.run(inputs);
	Vector<Vector<float>> output1 = imu.run(inputs);

	Serial.println("LSM6DSOX outputs:");
	output1.print();

	Serial.println("Process State:");
	Vector<float> state1 = imu.context();
	state1.print();

	imu.print();

	Serial.println("=== Finished Process tests ===");
}