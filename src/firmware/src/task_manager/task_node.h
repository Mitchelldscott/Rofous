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

#ifndef SYNCOR_NODE
#define SYNCOR_NODE

#include "task_manager/task_factory.h"

class TaskNode {
	private:
		int is_latched;

		Task* task;					// The thing that does the jawns
		Vector<int> input_ids;			// proc_ids of input nodes (maybe useless)
		Vector<TaskNode*> inputs;		// pointers to the input nodes (not implemented)
		Vector<float> input_buffer;
		Vector<float> output_buffer;	// a buffer of output data, maybe ditching soon
		Vector<float> context_buffer;	// buffer of context data, also ditching soon
		Vector<float> parameter_buffer;	// configuration buffer (here to stay)

	public:
		TaskNode();
		TaskNode(Task*, int, int*);

		void latch(int);

		int n_inputs();
		int input_id(int);

		void reset_config();
		bool is_configured();

		int n_links();
		void set_inputs(int*, int);
		void link_input(TaskNode*);

		void set_task(Task*);
		bool setup_task();
		void collect_inputs();
		bool run_task();

		void print();
		void print_output();

		Vector<float>* operator [](int index) {
			switch (index) {
				case INPUT_DIMENSION:
					return &input_buffer;
					break;

				case CONTEXT_DIMENSION:
					return &context_buffer;
					break;

				case PARAMS_DIMENSION:
					return &parameter_buffer;

				default: // output returns by default
					return &output_buffer;
					break; 
			}
		}
};

#endif