/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

#include "utilities/timing.h"
#include "utilities/splash.h"
#include "utilities/assertions.h"
#include "hid_comms/hid_comms.h"

#define MASTER_CYCLE_TIME_MS 	1
#define MASTER_CYCLE_TIME_S 	(MASTER_CYCLE_TIME_MS * 1E-3)
#define MASTER_CYCLE_TIME_US 	(MASTER_CYCLE_TIME_MS * 1E3 )
#define MASTER_CYCLE_TIME_ERR 	(MASTER_CYCLE_TIME_MS + 0.01)

#define TEST_DURATION_S			30

FTYK timers;
int tasks = 0;

void task_publish_handler(CommsPipeline* comms_pipe, int id, Vector<float>& output) {
	noInterrupts();
	comms_pipe->feedback[id]->output = output;
	interrupts();
}

void add_feedback_node(CommsPipeline* comms_pipe, int id) {
	TaskFeedback* task_fb = new TaskFeedback;
	task_fb->latch = 0;
	task_fb->task_id = id;
	task_fb->output.reset(0);
	task_fb->timestamp = -1;

	noInterrupts();
	comms_pipe->feedback.push(task_fb);
	interrupts();
}

void task_setup_handler(CommsPipeline* comms_pipe) {
	// consume all input packets
	noInterrupts();
	int n_items = comms_pipe->setup_queue.size();
	interrupts();

	if (n_items <= 0) {
		return;
	}

	for (int i = 0; i < n_items; i++) {
		noInterrupts();
		TaskSetupPacket* p = comms_pipe->setup_queue.pop();
		interrupts();

		printf("Consuming %i/%i items\n", i, n_items);


		if (p->packet_type == 0) {
			add_feedback_node(comms_pipe, p->task_id);
			tasks += 1;
		}

		if (p->packet_type == 2) {
			noInterrupts();
			comms_pipe->feedback[i]->output = p->data;
			interrupts();
		}
		
		delete p;
	}
}

// Runs once
int main() {

	int errors = 0;

	unit_test_splash("Live comms", TEST_DURATION_S);

	CommsPipeline* comms_pipe = enable_hid_interrupts();

	while (!usb_rawhid_available()) {};

	int lifetime = 0;
	Vector<float> output(5);
	timers.set(0);
	timers.set(1);
	while (lifetime < TEST_DURATION_S) {

		task_setup_handler(comms_pipe);
		for(int i = 0; i < tasks; i++) {
			task_publish_handler(comms_pipe, i, output);
		}
		
		noInterrupts();
		comms_pipe->lifetime += MS_2_S(timers.delay_millis(1, MASTER_CYCLE_TIME_MS));
		interrupts()
		errors += assert_leq<float>(timers.millis(1), MASTER_CYCLE_TIME_ERR, "Teensy overcycled"); 
		timers.set(1);
		lifetime += timers.secs(0);
		timers.set(0);
	}

	printf(" * Finished tests in %fs\n", comms_pipe->lifetime);
	printf(" * %i failed\n", int(errors + hid_errors));

	return 0;
}


