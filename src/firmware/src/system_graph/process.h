#ifndef SYS_GRAPH_PROCESS
#define SYS_GRAPH_PROCESS

template <class T> class Process {
	private:
		T* driver;

	public:
		Process<T>();
		~Process<T>();
		void reset();
		void clear();
		float* state();
		float* run(float*);
		void print();
};

#endif