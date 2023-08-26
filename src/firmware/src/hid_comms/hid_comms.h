#include "bytebuffer.h"
#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/vector.h"

#ifndef HIDCOMMS_H
#define HIDCOMMS_H

#define HID_REFRESH_RATE 500.0

extern float hid_errors;

struct TaskSetupPacket {
	int task_id;
	int packet_type;
	
	String key;
	int n_inputs;
	Vector<int> inputs;

	int chunk_id;
	int chunk_size;
	Vector<float> parameters;

	int latch;
	int data_len;
	int context_alt;
	Vector<float> data;

	void print() {
		printf("ID: %i\nPacket type: %i\n", task_id, packet_type);
	}
};

struct TaskFeedback {
	// int error;	// someone should figure this out
	int task_id;
	float timestamp;
	Vector<float> output;
	Vector<float> context;
};

struct CommsPipeline {
	Vector<int> ids;
	int recent_update;
	Vector<TaskFeedback*> feedback;
	Vector<TaskSetupPacket*> setup_queue;
};

void push_hid();
void send_hid_status();
void send_hid_feedback();
void send_hid_with_timestamp();
void init_task_hid();
void config_task_hid();
void overwrite_task_hid();
void reset_hid_stats();
void dump_vector(Vector<float>*);
CommsPipeline* enable_hid_interrupts();

#endif