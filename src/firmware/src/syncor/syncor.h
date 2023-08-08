#ifndef SYS_GRAPH_OBJ
#define SYS_GRAPH_OBJ

#include "utilities/blink.h"

#include "hid_comms/hid_report.h"
#include "hid_comms/hid_comms.h"

#include "syncor/syncor_node.h"
#include "syncor/process_factory.h"

#define MAXIMUM_GRAPH_NODES 10
#define HID_READ_WRITE_RATE_US 1000

Process_Factory p_factory;


class SynCor {
	private:

		FTYK timers;
		float lifetime;

		Vector<int> status;
		Vector<int> node_ids;
		Vector<SynCorNode*> nodes;

	public:
		SynCor();
		// ~SystemGraph();
		int node_index(int proc_id);
		void add(HidProcessParams*);
		void update_config(HidProcessConfig*);
		Vector<float> collect_outputs(int);

		void spin();
		void dump_all();
		// void handle_hid();
		// void dump_vector(Vector<float>*);
};

#endif