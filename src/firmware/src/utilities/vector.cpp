#include <Arduino.h>
#include "utilities/vector.h"
#include "system_graph/process.h"
#include "system_graph/graph_node.h"

template <typename T> Vector<T>::Vector() {
	length = 0;
	buffer = NULL;
}

// template <typename T> Vector<T>::~Vector() {
// 	if (buffer != NULL) {
// 		delete buffer;
// 		buffer = NULL;
// 	}
// }

template <typename T> Vector<T>::Vector(int size) {
	length = size;
	buffer = new T[size];
}

template <typename T> Vector<T>::Vector(T* data, int size) {
	length = size;
	buffer = data;
	// buffer = new T[size];
	// for (int i = 0; i < length; i++) {
	// 	buffer[i] = data[i];
	// }
}

template <typename T> void Vector<T>::reset(int size) {
	if (buffer != NULL) {
		delete buffer;
	}
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> void Vector<T>::clear() {
	memset(buffer, 0, length * sizeof(T));
}

template <typename T> int Vector<T>::size() {
	return length;
}

template <typename T> void Vector<T>::push(T item) {
	append(&item , 1);
}

template <typename T> T& Vector<T>::operator[](int index) {
	if (length > index && index >= 0) {
		return buffer[index];
	}
	exit(0);
}

// template <> void Vector<float>::operator=(Vector<float> v) {
// 	if (buffer == NULL) {
// 		reset(v.size());
// 	}

// 	int n_elements = min(length, v.size());
// 	if (n_elements > 0) {
// 		for (int i = 0; i < n_elements; i++) {
// 			buffer[i] = v[i];
// 		}
// 	}
// }

template <typename T> void Vector<T>::operator=(Vector<T>& v) {
	if (buffer == NULL) {
		reset(v.size());
	}

	int n_elements = min(length, v.size());
	if (n_elements > 0) {
		for (int i = 0; i < n_elements; i++) {
			buffer[i] = v[i];
		}
	}
}

template <typename T> void Vector<T>::from_vec(T* data, int n) {
	if (length == n) {
		memcpy(buffer, data, n * sizeof(T));
	}
	else {
		reset(n);
		memcpy(buffer, data, n * sizeof(T));
	}
}

template <typename T> void Vector<T>::append(Vector<T>* data) {
	int n = data->size();
	T tmp[length + n];
	memcpy(tmp, buffer, length * sizeof(T));
	reset(length + n);
	data->slice(&tmp[length], 0, n);
	memcpy(buffer, tmp, length * sizeof(T));
}

template <typename T> void Vector<T>::append(T* data, int n) {
	T tmp[length];
	memcpy(tmp, buffer, length * sizeof(T));
	reset(length + n);
	memcpy(buffer, tmp, (length - n) * sizeof(T));
	memcpy(&buffer[(length - n)], data, n * sizeof(T));
}

template <typename T> void Vector<T>::slice(T* data, int start, int end) {
	if (start < end && end < length) {
		memcpy(data, &buffer[start], (end - start) * sizeof(T));
	}
}

template <typename T> int Vector<T>::find(T data) {
	for (int i = 0; i < length; i++) {
		if (buffer[i] == data) {
			return i;
		}
	}
	return -1;
}

template <typename T> void Vector<T>::insert(T* data, int index, int n) {
	if (index + n > length) {
		Vector<T> empty((index + n) - length);
		append(&empty);
	}
	for (int i = 0; i < n; i++) {
		buffer[i + index] = data[i];
	}
}

template <typename T> void Vector<T>::insert(Vector<T> data, int index) {
	int n = data.size();
	if (index + n > length) {
		Vector<T> empty((index + n) - length);
		append(&empty);
	}
	for (int i = 0; i < n; i++) {
		buffer[i + index] = data[i];
	}
}

template <> void Vector<int>::print() {
	Serial.printf("Vectori [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%i, ", buffer[i]);
	}
	Serial.printf("%i]\n", buffer[length-1]);
}

template <> void Vector<float>::print() {
	Serial.printf("Vectorf [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%f, ", buffer[i]);
	}
	Serial.printf("%f]\n", buffer[length-1]);
}

// template <class T> void Vector<T>::print() {
// 	Serial.printf("\tVectorT [%i]\n", length);
// 	for (int i = 0; i < length; i++) {
// 		buffer[i].print();
// 	}
// }

template class Vector<int>;
template class Vector<float>; // need this so the linker will have a compiled version of Vector<float> (otherwise undefined reference)
template class Vector<Process*>;
template class Vector<GraphNode*>;
// template class Vector<Vector<int>>;
// template class Vector<Vector<float>>; 






