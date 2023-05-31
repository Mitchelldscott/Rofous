#include <Arduino.h>
#include "system_graph/vector.h"

template <typename T> Vector<T>::Vector() {
	dim = 0;
	length = 0;
	buffer = NULL;
}

template <typename T> Vector<T>::Vector(int size) {
	dim = 1;
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> Vector<T>::Vector(int rows, int cols) {
	dim = 2;
	length = rows;
	T empty_row(cols);
	buffer = (T*)calloc(rows, sizeof(empty_row));
	for (int i = 0; i < rows; i++) {
		buffer[i] = empty_row;
	}
}

template <typename T> Vector<T>::Vector(T* data, int size) {
	dim = 1;
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
	for (int i = 0; i < length; i++) {
		buffer[i] = data[i];
	}
}

template <typename T> void Vector<T>::reset(int size) {
	if (buffer != NULL) {
		free(buffer);
	}
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> void Vector<T>::reset(int rows, int cols) {
	if (buffer != NULL) {
		free(buffer);
	}
	length = rows;
	T empty_row(cols);
	buffer = (T*)calloc(rows, sizeof(empty_row));
	for (int i = 0; i < rows; i++) {
		buffer[i] = empty_row;
	}
}

template <> void Vector<float>::clear() {
	memset(buffer, 0, length * sizeof(float));
}

template <> void Vector<int>::clear() {
	memset(buffer, 0, length * sizeof(int));
}

template <typename T> void Vector<T>::clear() {
	for (int i = 0; i < length; i++) {
		buffer[i].clear();
	}
}

template <typename T> int Vector<T>::size() {
	return length;
}

template <typename T> void Vector<T>::push(T item) {
	if (ptr >= length) {
		ptr = 0;
	}

	buffer[ptr] = item;
	ptr++;
}

template <typename T> T& Vector<T>::operator[](int index) {
	if (length > index && index >= 0) {
		return buffer[index];
	}
	exit(0);
}

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

template <typename T> void Vector<T>::append(T* data, int n) {
	T tmp[length];
	memcpy(tmp, buffer, length * sizeof(T));
	reset(length + n);
	memcpy(buffer, tmp, (length - n) * sizeof(T));
	memcpy(&buffer[(length - n)], data, n * sizeof(T));
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

template <class T> void Vector<T>::print() {
	Serial.printf("\tVectorT [%i]\n", length);
	for (int i = 0; i < length; i++) {
		buffer[i].print();
	}
}

template class Vector<float>; // need this so the linker will have a compiled version of Vector<float> (otherwise undefined reference)
template class Vector<Vector<int>>;
template class Vector<Vector<float>>; 






