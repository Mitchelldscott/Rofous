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

#include "hid_comms.h"
#include "utilities/assertions.h"

float hid_errors = 0;

float pc_lifetime = 0;
float hid_lifetime = 0;
float pc_read_count = 0;
float pc_write_count = 0;
float mcu_read_count = 0;
float mcu_write_count = 0;

FTYK hid_timers;
ByteBuffer<64> buffer;
IntervalTimer hid_interval_timer;

Vector<int>* node_ids_p;
Vector<TaskNode*>* nodes_p;
Vector<FWTaskPacket*> task_control_queue;

void push_hid() {
	int index = 0;

	if(!usb_rawhid_available()) {
		// printf("RawHID not available %f\n", hid_timers.secs(1));
		if (hid_timers.secs(1) > 5) { // reset stats when no connection
			reset_hid_stats();
			hid_timers.set(1);
		}
		return;
	}

	hid_timers.set(1);
	

	switch (usb_rawhid_recv(buffer.buffer(), 0)) {
		case 64:
			blink();										// only blink when connected to hid
			mcu_read_count += 1;
			switch (buffer.get<byte>(0)) {
				case 255:
					// printf("Recieved Config Report\n");
					switch (buffer.get<byte>(1)) {
						case 1:
							init_task_hid();
							break;

						case 2:
							config_task_hid();
							break;

						case 255:
							// write count is a better interpretation of connection, don't want to force lifetimes equal
							// hid_errors += assert_eq<float>(buffer.get<float>(2), mcu_write_count, 1, "pc vs mcu write count");
							// hid_errors += assert_eq<float>(buffer.get<float>(2), pc_write_count, 1, "pc write count jump");
							// hid_errors += assert_eq<float>(buffer.get<float>(60), pc_lifetime, 0.001, "pc lifetime jump");
							pc_write_count = buffer.get<float>(2);
							pc_read_count = buffer.get<float>(6);
							// assert_leq<float>(abs(pc_lifetime - hid_lifetime), 0.001, "pc time - mcu time\t\t");
							buffer.put<float>(2, mcu_write_count);
							buffer.put<float>(6, mcu_read_count);
							break;

						default:
							break;
					}
					break;

				case 1:
					index = node_ids_p->find(buffer.get<byte>(2));
					// printf("node index: %i %i\n", buffer.get<byte>(2), index);
					// printf("nodes: %i\n", nodes_p->size());
					if (index >= 0 && nodes_p->size() > index) {
						switch (buffer.get<byte>(1)) {
							case 1:
									dump_vector((*nodes_p)[index]->context());
									// dump_vector(hid_context_buffer[index]->info());
								break;

							case 2:
									dump_vector((*nodes_p)[index]->output());
									// dump_vector(hid_output_buffer[index]->info());
								break;

							default:
								break;
						}
					}
					break;

				case 13:
					reset_hid_stats();
					break;

				default:
					break;
			}
			break;
		
		default:
			// printf("No packet available\n");
			break;
	}

	pc_lifetime = buffer.get<float>(60);
	hid_lifetime += hid_timers.secs(0);
	hid_timers.set(0);
	buffer.put<float>(60, hid_lifetime);
	if (usb_rawhid_send(buffer.buffer(), 0) > 0) {
		mcu_write_count += 1;
	}
	else {
		printf("failed to write\n");
	}
}

void init_task_hid() {

	// printf("Init Task %i\n", buffer.get<byte>(2));
	FWTaskPacket* task = new FWTaskPacket;
	
	task->packet_type = 0;
	task->key = buffer.get<char>(3);
	task->key += buffer.get<char>(4);
	task->key += buffer.get<char>(5);
	task->task_id = buffer.get<byte>(2);
	task->n_inputs = buffer.get<byte>(6);

	for (int i = 0; i < task->n_inputs; i++) {
		task->inputs.push(buffer.get<byte>(i + 7));
	}

	// push to new node queue
	task_control_queue.push(task);
}

void config_task_hid() {

	FWTaskPacket* task = new FWTaskPacket;

	task->packet_type = 1;
	task->task_id = buffer.get<byte>(2);
	task->chunk_id = buffer.get<byte>(3);
	task->chunk_size = buffer.get<byte>(4);

	for (int i = 0; i < task->chunk_size; i++) {
		task->parameters.push(buffer.get<float>((4 * i) + 5));
	}

	// push config packet to config queue
	task_control_queue.push(task);
}

void reset_hid_stats() {
	mcu_write_count = 0;
	mcu_read_count = 0;
	pc_write_count = 0;
	pc_read_count = 0;
	hid_lifetime = 0;
	pc_lifetime = 0;
	hid_errors = 0;
}

void dump_vector(Vector<float>* data) {
	buffer.put<byte>(3, data->size());
	buffer.put<float>(4, data->size(), data->as_array());
}

Vector<FWTaskPacket*>* enable_hid_interrupts(Vector<int>* ids_ptr, Vector<TaskNode*>* node_ptr) {
	nodes_p = node_ptr;
	node_ids_p = ids_ptr;
	task_control_queue.reset(0);
	hid_interval_timer.begin(push_hid, HID_REFRESH_RATE);
	return &task_control_queue;
}