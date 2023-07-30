#ifndef SYS_GRAPH_OBJ
#define SYS_GRAPH_OBJ

#include "utilities/blink.h"

#include "robot_comms/hid_report.h"

#include "syncor/syncor_node.h"
#include "syncor/process_factory.h"

#define MAXIMUM_GRAPH_NODES 10
#define HID_READ_WRITE_RATE_US 1000

class SynCor {
	private:
		HidReport report;
		IntervalTimer hidtimer;
		Process_Factory factory;

		FTYK timers;		// timer0 = hid write timer
		float lifetime;

		Vector<int> status;
		Vector<int> node_ids;
		Vector<SynCorNode*> nodes;

	public:
		SynCor();
		// ~SystemGraph();
		void enable_hid_interrupts();
		Vector<float> collect_outputs(int);
		void add(String, int, int, int, int*);
		void update_config(int, int, int, float*);
		void spin();
		void dump_all();
		void handle_hid();
		void init_process_hid();
		void config_process_hid();
		void dump_vector(Vector<float>*);
};

#endif