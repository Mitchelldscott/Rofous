#include "utilities/splash.h"
#include "utilities/assertions.h"
#include "task_manager/task_manager.h"

#define MASTER_CYCLE_TIME_US 	1000.0
#define MASTER_CYCLE_TIME_MS 	1.0
#define MASTER_CYCLE_TIME_S 	0.001
#define MASTER_CYCLE_TIME_ERR 	1.001 // ms


FTYK timers;
Vector<FWTaskPacket*>* queue;

int main() {
	while(!Serial){}

	unit_test_splash("Task Manager", 0);
	
	int cmf_inputs[2] = {9, 10};

	FWTaskPacket* lsm = new FWTaskPacket;
	lsm->key = "LSM";
	lsm->task_id = 10;
	lsm->n_inputs = 0;

	FWTaskPacket* cmf = new FWTaskPacket;
	cmf->key = "CMF";
	cmf->task_id = 9;
	cmf->n_inputs = 2;
	cmf->inputs.from_array(cmf_inputs, 2);

	FWTaskPacket* cmf_params = new FWTaskPacket;
	cmf_params->task_id = 9;
	cmf_params->chunk_id = 0;
	cmf_params->chunk_size = 1;
	cmf_params->parameters.reset(1);
	cmf_params->parameters[0] = 0.6;

	printf("\n=== Add nodes and parameters manually ===\n");

	timers.set(0);
	queue = init_task_manager();
	assert_leq<float>(timers.micros(0), 2, "TM init timer");
	
	timers.set(1);
	add_task(lsm);
	assert_leq<float>(timers.millis(1), 50, "LSM add timer");

	timers.set(1);
	add_task(cmf);
	assert_leq<float>(timers.micros(1), 6, "CMF add timer");

	timers.set(1);
	update_task(cmf_params);
	assert_leq<float>(timers.micros(1), 2, "CMF update timer");

	for (int i = 0; i < 3; i++) {
		timers.set(1);
		spin();
		assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Task run timing test");
	}

	dump_all_tasks();

	printf("\n=== Reset nodes and update parameters from the queue ===\n");

	// reset the tasks so they require a reconfig
	timers.set(1);
	add_task(lsm);
	assert_leq<float>(timers.millis(1), 50, "LSM reset timer");

	timers.set(1);
	add_task(cmf);
	assert_leq<float>(timers.micros(1), 6, "CMF reset timer");

	queue->push(cmf_params);

	dump_all_tasks();

	timers.set(1);
	spin();
	assert_leq<float>(timers.delay_millis(1, MASTER_CYCLE_TIME_MS), MASTER_CYCLE_TIME_ERR, "Task run timing test");

	dump_all_tasks();

	printf("=== Finished Task Manager tests ===\n");
	
}