#include "bytebuffer.h"
#include "utilities/blink.h"
#include "utilities/timing.h"
#include "utilities/vector.h"
#include "task_manager/task_node.h"

#ifndef HIDCOMMS_H
#define HIDCOMMS_H

#define HID_REFRESH_RATE 500.0

extern float hid_errors;

struct FWTaskPacket {
	int task_id;
	int packet_type;
	
	String key;
	int n_inputs;
	Vector<int> inputs;

	int chunk_id;
	int chunk_size;
	Vector<float> parameters;
};

void push_hid();
void init_task_hid();
void config_task_hid();
void reset_hid_stats();
void dump_vector(Vector<float>*);
Vector<FWTaskPacket*>* enable_hid_interrupts(Vector<int>*, Vector<TaskNode*>*);

#endif