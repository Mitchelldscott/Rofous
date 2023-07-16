#include <Arduino.h>
#include "utilities/vector.h"
#include "syncor/process.h"
#include "syncor/syncor_node.h"

template <typename T> Vector<T>::Vector() {
	length = 0;
	buffer = NULL;
}

template <typename T> Vector<T>::~Vector() {
	delete buffer;
}

template <typename T> Vector<T>::Vector(int size) {
	/*
		  Constructor for Vector with length = size.
		@param:
			size: (int) length of the buffer with type T
	*/
	length = size;
	buffer = new T[size];
	clear();
}

template <typename T> Vector<T>::Vector(T* data, int size) {
	/*
		  Constructor for Vector with length = size and non-zero data.
		@param:
			data: (T*) data to fill buffer with
			size: (int) length of the buffer with type T
	*/
	length = size;
	buffer = data;
}

///// modifiers /////

template <typename T> void Vector<T>::clear() {
	/*
		  Clear data in the buffer (set to 0).
	*/
	memset(buffer, 0, length * sizeof(T));
}

template <typename T> void Vector<T>::push(T item) {
	/*
		  Add a single item T to the buffer.
		Basically and append wraper. (maybe rename this)
		@param:
			item: (T) data to add to buffer
	*/
	T tmp[length + 1];
	memcpy(tmp, buffer, length * sizeof(T));
	tmp[length] = item;
	from_array(tmp, length+1);
}

template <typename T> void Vector<T>::reset(int size) {
	/*
		  Resize buffer and set data to zero.
		@param:
			data: (T*) data to fill buffer with
			size: (int) length of the buffer with type T
	*/
	if (buffer != NULL) {
		delete buffer;
	}
	length = size;
	buffer = new T[size];
	clear();
}

template <typename T> void Vector<T>::from_array(T* data, int size) {
	/*
		  reset the buffer to size n with data T*.
		@param:
			data: (T*) data to fill buffer with
			size: (int) length of the buffer with type T
	*/
	reset(size);
	memcpy(buffer, data, size * sizeof(T));
}

template <typename T> void Vector<T>::append(T* data, int n) {
	/*
		  Add n values to the buffer. Stores the current buffer
		calls reset and then copies buffers into resized buffer.
		@param:
			data: (T*) data to fill buffer with
			size: (int) length of the buffer with type T
	*/
	T tmp[length];
	memcpy(tmp, buffer, length * sizeof(T));
	reset(length + n);
	memcpy(buffer, tmp, (length - n) * sizeof(T));
	memcpy(&buffer[(length - n)], data, n * sizeof(T));
}

template <typename T> void Vector<T>::append(Vector<T> data) {
	/*
		  Add n values to the buffer. Stores the current buffer and
		data to add a temp. Calls reset and then copies buffer into resized buffer.
		@param:
			data: (Vector<T>*) data to fill buffer with
	*/
	int n = length;
	int m = data.size();

	T tmp1[n];
	T* tmp2 = data.as_array();

	reset(n + m);
	for (int i = 0; i < n; i++) {
		buffer[i] = tmp1[i];
	}
	for (int i = 0; i < m; i++) {
		buffer[i+n] = tmp2[i];
	}
	// memcpy(buffer, tmp1, n * sizeof(T));
	// memcpy(&buffer[n], tmp2, m * sizeof(T));
}

template <typename T> void Vector<T>::append(Vector<T>* data) {
	/*
		  Add n values to the buffer. Stores the current buffer and
		data to add a temp. Calls reset and then copies buffer into resized buffer.
		@param:
			data: (Vector<T>*) data to fill buffer with
	*/
	int n = length;
	int m = data->size();

	T tmp1[n];
	T* tmp2 = data->as_array();

	reset(n + m);
	memcpy(buffer, tmp1, n * sizeof(T));
	memcpy(&buffer[n], tmp2, m * sizeof(T));
}

template <typename T> void Vector<T>::insert(T* data, int index, int size) {
	/*
		  Add n values to the buffer starting at index. If buffer is not
		large enough it will be extended. This works much better if
		the vector is already large enough (append can call reset).
		@param:
			data: (T*) data to fill buffer with
			index: (int) index to start insertion
			size: (int) number of items to insert
	*/
	if (index + size > length) {
		Vector<T> empty((index + size) - length);
		append(&empty);
	}
	for (int i = 0; i < size; i++) {
		buffer[i + index] = data[i];
	}
}

template <typename T> void Vector<T>::insert(Vector<T> data, int index) {
	/*
		  Add n values to the buffer starting at index. If buffer is not
		large enough it will be extended. This works much better if
		the vector is already large enough (append can call reset).
		@param:
			data: (Vector<T>*) data to fill buffer with
			index: (int) index to start insertion
	*/
	int n = data.size();
	if (index + n > length) {
		Vector<T> empty((index + n) - length);
		append(&empty);
	}
	for (int i = 0; i < n; i++) {
		buffer[i + index] = data[i];
	}
}

///// accessors /////

template <typename T> int Vector<T>::size() {
	/*
		  Get the size of buffer (not necessarily elements available)
		@return
			length: (int) size of buffer
	*/
	return length;
}

template <typename T> int Vector<T>::find(T data) {
	/*
		  Get the first index of an item in the buffer
		@return
			data: (T) item to search for (-1 if not found, maybe causes template issues)
	*/
	for (int i = 0; i < length; i++) {
		if (buffer[i] == data) {
			return i;
		}
	}
	return -1;
}

template <typename T> T* Vector<T>::as_array() {
	return buffer;
}

template <typename T> void Vector<T>::slice(T* data, int start, int n) {
	/*
		  Get a slice of the buffer, must have valid indices.
		@return
			data: (T*) buffer to put slice
			start: (int) start index of buffer
			n: (int) number of items in slice
	*/
	if (start >= 0 && start + n <= length) {
		memcpy(data, &buffer[start], n * sizeof(T));
	}
	else {
		memset(data, 0, n * sizeof(T));
	}
}

///// operators /////

template <typename T> T& Vector<T>::operator[](int index) {
	/*
		  [] Operator overload
		@param
			index: (int) index of item in buffer to access
		@return
			item: (T&) item at index
		@exit
			when index is invalid 
	*/
	if (length > index && index >= 0) {
		return buffer[index];
	}
	exit(0);
}

template <typename T> void Vector<T>::operator=(Vector<T>& data) {
	/*
		  = Operator overload. Will reset this vector to the
		same size as data.
		@param
			data: (Vector<T>&) data to copy
	*/
	from_array(data.as_array(), data.size());
}

/// print

template <> void Vector<int>::print() {
	if (length == 0) {
		Serial.printf("Vectori [%i]\n", length);
		return;
	}
	Serial.printf("Vectori [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%i, ", buffer[i]);
	}
	Serial.printf("%i]\n", buffer[length-1]);
}

template <> void Vector<float>::print() {
	if (length == 0) {
		Serial.printf("Vectorf [%i]\n", length);
		return;
	}
	Serial.printf("Vectorf [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%f, ", buffer[i]);
	}
	Serial.printf("%f]\n", buffer[length-1]);
}

template <> void Vector<Process*>::print() {
	if (length == 0) {
		Serial.printf("VectorP [%i]\n", length);
		return;
	}
	Serial.printf("VectorP [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		buffer[i]->print();
		Serial.print(", ");
	}
	buffer[length-1]->print();
	Serial.println("]");
}

// need this so the linker will have a compiled version of Vector<typename> (otherwise undefined reference)
template class Vector<int>;
template class Vector<float>; 
template class Vector<Process*>;
template class Vector<SynCorNode*>;
// template class Vector<Vector<int>>;
// template class Vector<Vector<float>>; 






