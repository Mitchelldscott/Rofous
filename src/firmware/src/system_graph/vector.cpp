#include <Arduino.h>
#include "system_graph/vector.h"

template <typename T> Vector<T>::Vector() {
	length = 0;
	buffer = NULL;
}

template <typename T> Vector<T>::Vector(int size) {
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> Vector<T>::Vector(T* data, int size) {
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
	for (int i = 0; i < length; i++) {
		buffer[i] = data[i];
	}
}

template <typename T> Vector<T>::~Vector() {
	free(buffer);
}

template <typename T> void Vector<T>::reset(int size) {
	if (buffer != NULL) {
		free(buffer);
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

template <class T> void Vector<T>::print() {
	for (int i = 0; i < length; i++) {
		buffer[i].print();
	}
}

template <> void Vector<float>::from_vec(Vector<float> data) {
	if (length != data.size()) {
		reset(data.size());
	}

	for (int i = 0; i < length; i++) {
		buffer[i] = data[i];
	}
}

template <> void Vector<int>::print() {
	Serial.printf("Vector[%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%i, ", buffer[i]);
	}
	Serial.print(buffer[length-1]);
	Serial.println("]");
}

template <> void Vector<float>::print() {
	Serial.printf("Vector[%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.printf("%f, ", buffer[i]);
	}
	Serial.print(buffer[length-1]);
	Serial.println("]");
}


template class Vector<float>; // need this so the linker will have a compiled version of Vector<float> (otherwise undefined reference)
template class Vector<Vector<int>>;
template class Vector<Vector<float>>; 














