#ifndef SYS_GRAPH_VECTOR
#define SYS_GRAPH_VECTOR

template <typename T> class Vector {
	private:
		int ptr;
		int dim;
		int length;
		T* buffer;

	public:
		Vector<T>();
		Vector<T>(int);
		Vector<T>(int, int);
		Vector<T>(T*, int);
		void reset(int);
		void reset(int, int);
		void clear();
		int size();
		void push(T);
		T& operator[](int);
		void operator=(Vector<T>&);
		void from_vec(T*, int);
		void append(T*, int);
		void print();
};

#endif