#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

#include "utilities/vector.h"

#define PROCESS_DIMENSIONS 	3
#define INPUT_DIMENSION 	0
#define CONTEXT_DIMENSION 	1
#define OUTPUT_DIMENSION 	2
/*
	Base class for all user defined processes.
	Must override the functions below, constructor
	optional... I think.
*/

class Process {	
	public:
		Vector<int> dimensions;
		virtual int input_dim();
		virtual int context_dim();
		virtual int output_dim();
		virtual void reset();
		virtual void clear();
		virtual void print();
		virtual void setup(Vector<float>*);
		virtual void context(Vector<float>*);
		virtual void run(Vector<float>*, Vector<float>*);
		bool operator == (const Process&);
};

#endif