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

#include "task_manager/task.h"


class TaskNode {
	private:
		Task* task;					// The thing that does the jawns
		Vector<int> input_ids;			// proc_ids of input nodes (maybe useless)
		Vector<TaskNode*> inputs;		// pointers to the input nodes (not implemented)
		Vector<float> output_buffer;	// a buffer of output data, maybe ditching soon
		Vector<float> config_buffer;	// configuration buffer (here to stay)
		Vector<float> context_buffer;	// buffer of context data, also ditching soon

	public:
		TaskNode();
		TaskNode(Task*, int, int*);

		int n_inputs();
		int input_dim();
		int context_dim();
		int output_dim();
		int input_id(int);
		bool is_configured();
		Vector<float>* output();
		Vector<float>* config();
		Vector<float>* context();

		void set_config();
		void set_inputs(int*, int);
		void set_task(Task*);

		bool setup_task();
		bool run_task(Vector<float>*);

		void print();
		void print_output();
};

#endif