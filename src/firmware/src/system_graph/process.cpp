#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/process.h"

// Drivers
#include "sensors/lsm6dsox.h"

template <class T> Process<T>::Process() {
	driver = new T();
}

template <class T> Process<T>::Process(Vector<float> config) {
	driver = new T();
	driver->setup(config);
}

template <class T> Process<T>::~Process() {
	free(driver);
}

template <class T> void Process<T>::setup(Vector<float> config) {
	driver->setup(config);
}

template <class T> void Process<T>::reset() {
	driver->reset();
}

template <class T> void Process<T>::clear() {
	driver->clear();
}

template <class T> Vector<float> Process<T>::state() {
	return driver->state();
}

template <class T> Vector<float> Process<T>::run(Vector<Vector<float>> input) {
	return driver->run(input);
}

template <class T> void Process<T>::print() {
	driver->print();
}

// enum class DriverTypes {
// 	LSM6DSOX,
// };

template class Process<DriverTypes::LSM6DSOX>;






