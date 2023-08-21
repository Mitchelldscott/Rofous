#include <Arduino.h>

#include "utilities/splash.h"
#include "utilities/vector.h"
#include "utilities/assertions.h"
#include "task_manager/task.h"
#include "task_manager/task_factory.h"
#include "algorithms/complimentary_filter.h"

#include "sensors/lsm6dsox.h"


int simple_proc_test(Task* p) {
	int errors = 0;

	p->reset();
	p->print();

	Vector<float> inputs(0);
	Vector<float> outputs(0);
	Vector<float> context(0);
	Vector<float> config(0);

	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "task uninitialized empty context test", context.size());

	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	p->context(&context);
	errors += assert_neq<float>(context.as_array(), 0.0f, "task post run non-empty context test", context.size());
	errors += assert_neq<float>(outputs.as_array(), 0.0f, "task post run non-empty outputs test", outputs.size());

	p->clear();

	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "task post run cleared empty context test", context.size());

	return errors;
}

int setup_proc_test(Task* p, Vector<float>* config, int num_inputs) {
	int errors = 0;
	Vector<float> outputs(0);
	Vector<float> context(0);
	Vector<float> inputs(num_inputs);

	p->reset();
	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "task with setup uninitialized empty context test", context.size());

	p->setup(config);
	
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);
	p->run(&inputs, &outputs);

	// errors += assert_neq<float>(outputs.as_array(), 0.0f, "task with setup post run non-empty outputs test", outputs.size());

	// p->context(&context);
	// errors += assert_neq<float>(context.as_array(), 0.0f, "task with setup post run non-empty context test", context.size());

	p->reset();
	p->context(&context);
	errors += assert_eq<float>(context.as_array(), 0.0f, "task with setup post run cleared empty context test", context.size());

	return errors;
}


int main() {
	int errors = 0;

	unit_test_splash("Task", 0);

	Task p;
	errors = simple_proc_test(&p);

	printf("=== Starting LSM6DSOX tests ===\n");

	LSM6DSOX imu;
	errors += simple_proc_test(&imu);

	printf("=== Starting ComplimentaryFilter tests ===\n");

	ComplimentaryFilter cmf;
	Vector<float> cmf_config(0);
	cmf_config.push(0.6);
	errors += setup_proc_test(&cmf, &cmf_config, 12);

	printf("=== Starting Factory tests ===\n");

	Vector<Task*> p_list(0);
	p_list.print();
	p_list.push(new_task("LSM"));
	p_list.print();
	p_list.push(new_task("CMF"));
	p_list.print();

	printf("=== LSM6DSOX ===\n");
	errors += simple_proc_test(p_list[0]);
	printf("=== ComplimentaryFilter ===\n");
	errors += setup_proc_test(p_list[1], &cmf_config, 12);


	printf("=== Finished task tests === %i errors\n", errors);

	return 0;
}