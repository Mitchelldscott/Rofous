#ifndef SYS_GRAPH_VECTOR
#define SYS_GRAPH_VECTOR

template <typename T> class Vector {
	private:
		int length;
		T* buffer;

	public:
		Vector<T>(int);
		~Vector<T>();
		void reset(int);
		void clear();
		int size();
		T& operator[](int);
		void print();
};

#endif