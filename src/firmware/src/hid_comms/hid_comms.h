
#ifndef HID_COMMS
#define HID_COMMS

struct HidProcessParams {
	String proc_key;
	int proc_id;
	int n_inputs;
	int n_configs;
	Vector<int> inputs;
}

struct HidProcessConfig {
	int proc_id;
	int chunk_id;
	int n_configs;
	Vector<float> config;
}

struct HidProcessInfo {
	int proc_id;
	float timestamp;
	Vector<float> info;
}

Vector<HidProcessParams*> hid_process_queue;
Vector<HidProcessConfig*> hid_config_queue;
Vector<HidProcessInfo*> hid_context_buffer;
Vector<HidProcessInfo*> hid_output_buffer;

FTYK hid_timers;
HidReport report;

void push_hid();
void init_process_hid();
void config_process_hid();
void enable_hid_interrupts();

#endif