#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/process.h"
#include "system_graph/process_factory.h"
#include "algorithms/complimentary_filter.h"

#include "sensors/lsm6dsox.h"


int main() {
	while(!Serial){}

	Serial.println("=== Starting PBO tests ===");

	Process p;

	p.clear();
	p.reset();
	p.print();

	Vector<float> inputs(1);
	Vector<float> outputs(1);
	Vector<float> state(1);
	p.run(&inputs, &outputs);

	Serial.println("Process outputs:");
	outputs.print();

	Serial.println("Process State:");
	p.context(&state);
	state.print();

	p.print();

	Serial.println("=== Starting LSM6DSOX tests ===");

	LSM6DSOX imu;

	imu.clear();
	imu.reset();
	imu.print();

	outputs.reset(9);
	imu.run(&inputs, &outputs);
	imu.run(&inputs, &outputs);
	imu.run(&inputs, &outputs);

	Serial.println("LSM6DSOX outputs:");
	outputs.print();

	Serial.println("Process State:");
	state.reset(0);
	imu.context(&state);
	state.print();

	imu.print();

	Serial.println("=== Starting ComplimentaryFilter tests ===");

	ComplimentaryFilter cmf;

	cmf.clear();
	cmf.reset();
	cmf.print();

	inputs.reset(12);
	outputs.reset(9);
	cmf.run(&inputs, &outputs);
	cmf.run(&inputs, &outputs);
	cmf.run(&inputs, &outputs);

	Serial.println("ComplimentaryFilter outputs:");
	outputs.print();

	Serial.println("Process State:");
	cmf.context(&state);
	state.print();

	cmf.print();

	Serial.println("=== Starting Factory tests ===");

	Process_Factory p_fact;

	Vector<Process*> p_list(2);
	p_fact.new_proc("LSM");
	p_list.push(p_fact.new_proc("LSM"));
	p_list.push(p_fact.new_proc("CMF"));
	// p_list.push(&imu);

	p_list[0]->run(&inputs, &outputs);
	p_list[0]->run(&inputs, &outputs);
	p_list[0]->run(&inputs, &outputs);

	p_list[0]->print();

	p_list[1]->run(&inputs, &outputs);
	p_list[1]->run(&inputs, &outputs);
	p_list[1]->run(&inputs, &outputs);

	p_list[1]->print();


	Serial.println("=== Finished Process tests ===");

	// while(1) {}
}