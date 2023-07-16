#include <Arduino.h>

#include "utilities/vector.h"
#include "utilities/assertions.h"
#include "system_graph/process.h"
#include "system_graph/process_factory.h"
#include "algorithms/complimentary_filter.h"

#include "sensors/lsm6dsox.h"


int simple_proc_test(Process* p) {
	int errors = 0;

	p->reset();
	p->print();

	Vector<float> inputs(0);
	Vector<float> outputs(0);
	Vector<float> context(0);
	Vector<float> config(0);

	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "Process uninitialized empty context test", context.size());

	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	p->context(&context);
	errors += assert_neq<float>(context.as_array(), 0.0f, "Process post run non-empty context test", context.size());
	errors += assert_neq<float>(outputs.as_array(), 0.0f, "Process post run non-empty outputs test", outputs.size());

	p->clear();

	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "Process post run cleared empty context test", context.size());

	return errors;
}

int setup_proc_test(Process* p, Vector<float>* config, int num_inputs) {
	int errors = 0;
	Vector<float> outputs(0);
	Vector<float> context(0);
	Vector<float> inputs(num_inputs);

	p->reset();
	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "Process with setup uninitialized empty context test", context.size());

	p->setup(config);
	
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	// errors += assert_neq<float>(outputs.as_array(), 0.0f, "Process with setup post run non-empty outputs test", outputs.size());

	// p->context(&context);
	// errors += assert_neq<float>(context.as_array(), 0.0f, "Process with setup post run non-empty context test", context.size());

	p->reset();
	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "Process with setup post run cleared empty context test", context.size());

	return errors;
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