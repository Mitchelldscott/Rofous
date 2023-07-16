#ifndef SYS_GRAPH_OBJ
#define SYS_GRAPH_OBJ

#include "utilities/timing.h"
#include "utilities/vector.h"
#include "syncor/process.h"
#include "robot_comms/hid_report.h"
#include "syncor/syncor_node.h"
#include "syncor/process_factory.h"

#define MAXIMUM_GRAPH_NODES 10

class SynCor {
	private:
		HidReport report;
		Process_Factory factory;

		FTYK watch;		// timer0 = hid write timer
		float lifetime;

		Vector<int> status;
		Vector<int> node_ids;
		Vector<SynCorNode*> nodes;

	public:
		SynCor();
		// ~SystemGraph();
		void collect_outputs(int, Vector<float>*);
		void add(String, int, int, int, int*);
		void update_config(int, int, int, float*);
		void spin();
		void dump_all();
		void handle_hid();
		void init_process_hid();
		void config_process_hid();
		void process_state_hid();
		void process_output_hid();
};

#endif