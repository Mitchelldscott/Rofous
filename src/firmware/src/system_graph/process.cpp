#include <Arduino.h>
#include "system_graph/process.h"

// Drivers
#include "sensors/lsm6dsox.h"

template <class T> Process<T>::Process() {
	driver = new T();
	driver->setup();
}

template <class T> Process<T>::~Process() {
	free(driver);
}

template <class T> void Process<T>::reset() {
	driver->reset();
}

template <class T> void Process<T>::clear() {
	driver->clear();
}

template <class T> float* Process<T>::state() {
	return driver->state();
}

template <class T> float* Process<T>::run(float* input) {
	return driver->run(input);
}

template <class T> void Process<T>::print() {
	driver->print();
}

template class Process<LSM6DSOX>;