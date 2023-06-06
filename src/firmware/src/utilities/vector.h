#ifndef SYS_GRAPH_VECTOR
#define SYS_GRAPH_VECTOR

template <typename T> class Vector {
	private:
		int length;
		T* buffer;

	public:
		Vector<T>();
		Vector<T>(int);
		Vector<T>(T*, int);
		// ~Vector<T>();
		void reset(int);
		void clear();
		int size();
		void push(T);
		T& operator[](int);
		// void operator=(Vector<float>);
		void operator=(Vector<T>&);
		void from_vec(T*, int);
		void append(Vector<T>*);
		void append(T*, int);
		void slice(T*, int, int);
		int find(T);
		void insert(T*, int, int);
		void insert(Vector<T>, int);
		void print();
};

#endif