#ifndef SYS_GRAPH_VECTOR
#define SYS_GRAPH_VECTOR

template <typename T> class Vector {
	private:
		int ptr;
		int length;
		T* buffer;

	public:
		Vector<T>();
		Vector<T>(int);
		Vector<T>(T*, int);
		~Vector<T>();
		void reset(int);
		void clear();
		int size();
		void push(T);
		T& operator[](int);
		void from_vec(Vector<T>);
		void print();
};

#endif