#ifndef SYNCOR_NODE
#define SYNCOR_NODE

#include <Arduino.h>
#include "utilities/vector.h"
#include "syncor/process.h"

/*
	SynCorNode is a large step toward the RTOS this code base creates.
	The SynCorNode is like a container for user defined processes.
	Each process will have a SynCorNode and each SynCorNode will have
	the output and locations of where to get the input.

	By running SynCorNodes in a proper order each will aquire it's input
	from the target SynCorNodes (inputs) and pass this to its process making the output
	requestable by other SynCorNodes. Graph cycles pose a significant threat to 
	the behavior of this system and should be avoided.

	Each SynCorNode will also store its own configuration and context data.
	These are Vectors in the SynCorNode, but each value can be interpreted as
	what ever the user wants. Simply write the driver expecting a specific config
	structure and fill the context out accordingly. A config is not
	a state but a list of values the driver will need to initialize. The context
	is closer to a state but does not need physical significance. Configurations
	come from the PC through HID and contexts are sent back to the PC though HID.
*/

class SynCorNode {
	private:
		int config_shape;

		Process* proc;					// The thing that does the jawns
		Vector<int> inputs_ids;			// proc_ids of input nodes (maybe useless)
		Vector<SynCorNode*> inputs;		// pointers to the input nodes (not implemented)
		Vector<float> output_buffer;	// a buffer of output data, maybe ditching soon
		Vector<float> config_buffer;	// configuration buffer (here to stay)
		Vector<float> context_buffer;	// buffer of context data, also ditching soon

	public:
		SynCorNode();
		SynCorNode(Process*, int, int, int*);

		int n_inputs();
		int input_dim();
		int context_dim();
		int output_dim();
		int input_id(int);
		bool is_configured();
		Vector<float>* output();
		Vector<float>* config();
		Vector<float>* context();

		void set_config(int);
		void set_inputs(int*, int);
		void set_process(Process*);

		bool setup_proc();
		bool run_proc(Vector<float>*);

		void print_proc();
		void print_output();
};

#endif