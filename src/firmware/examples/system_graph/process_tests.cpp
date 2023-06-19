#include <Arduino.h>

#include "utilities/vector.h"
#include "system_graph/process.h"
#include "system_graph/process_factory.h"
#include "algorithms/complimentary_filter.h"

#include "sensors/lsm6dsox.h"


void simple_proc_test(Process* p) {
	p->reset();
	p->print();

	Vector<float> inputs(0);
	Vector<float> outputs(0);
	Vector<float> state(0);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	Serial.print("output\t"); outputs.print();

	p->context(&state);
	Serial.print("context\t"); state.print();

	p->print();

	p->clear();
	p->print();
}

void setup_proc_test(Process* p, Vector<float>* config, int num_inputs) {
	p->reset();
	p->print();

	p->setup(config);

	Vector<float> inputs(num_inputs);
	Vector<float> outputs(0);
	Vector<float> context(0);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	Serial.print("output\t"); outputs.print();

	p->context(&context);
	Serial.print("context\t"); context.print();

	p->print();

	p->reset();
	p->print();
}


int main() {
	while(!Serial){}

	Serial.println("=== Starting PBO tests ===");
	Process p;
	simple_proc_test(&p);

	Serial.println("=== Starting LSM6DSOX tests ===");

	LSM6DSOX imu;
	simple_proc_test(&imu);

	Serial.println("=== Starting ComplimentaryFilter tests ===");

	ComplimentaryFilter cmf;
	Vector<float> cmf_config(0);
	cmf_config.push(0.6);
	setup_proc_test(&cmf, &cmf_config, 12);

	Serial.println("=== Starting Factory tests ==="); Serial.flush();

	Process_Factory p_fact;

	Vector<Process*> p_list(0);
	p_list.print();
	p_list.push(p_fact.new_proc("LSM"));
	p_list.print();
	p_list.push(p_fact.new_proc("CMF"));
	p_list.print();

	Serial.println("=== LSM6DSOX ===");
	simple_proc_test(p_list[0]);
	Serial.println("=== ComplimentaryFilter ===");
	setup_proc_test(p_list[1], &cmf_config, 12);


	Serial.println("=== Finished Process tests ===");

	return 0;
}