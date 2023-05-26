#include <Arduino.h>
#include "system_graph/vector.h"

template <typename T> Vector<T>::Vector(int size) {
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> Vector<T>::~Vector() {
	free(buffer);
}

template <typename T> void Vector<T>::reset(int size) {
	free(buffer);
	length = size;
	buffer = (T*)calloc(size, sizeof(T));
}

template <typename T> void Vector<T>::clear() {
	memset(buffer, 0, length * sizeof(T));
}

template <typename T> int Vector<T>::size() {
	return length;
}

template <typename T> T& Vector<T>::operator[](int index) {
	if (length > index) {
		return buffer[index];
	}
	exit(0);
}

template <typename T> void Vector<T>::print() {
	Serial.printf("Vector[%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		Serial.print(buffer[i]);
		Serial.print(", ");
	}
	Serial.print(buffer[length-1]);
	Serial.println("]");
}

template class Vector<float>; // need this so the linker will have a compiled version of Vector<float> (otherwise undefined reference)
template class Vector<Vector<float>>;