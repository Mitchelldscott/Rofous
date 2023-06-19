#ifndef SYS_GRAPH_VECTOR
#define SYS_GRAPH_VECTOR


/*
	Class for lists of objects, the goal is to
	provide a buffer with less type and memory
	restrictions.
*/
template <typename T> class Vector {
	private:
		int length;
		T* buffer;

	public:
		// constructors
		Vector<T>();
		Vector<T>(int);
		// Vector<T*>(int);
		Vector<T>(T*, int);
		~Vector<T>();

		// modifiers
		void clear();
		void push(T);
		void reset(int);
		void from_array(T*, int);
		void append(T*, int);
		void append(Vector<T>);
		void append(Vector<T>*);
		void insert(T*, int, int);
		void insert(Vector<T>, int);

		// accessers
		int size();
		int find(T);
		T* as_array();
		void slice(T*, int, int);

		// operators
		T& operator[](int);
		void operator=(Vector<T>&);
		// void operator=(Vector<T>);

		// print
		void print();
};

#endif