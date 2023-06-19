#include <Arduino.h>
#include "utilities/blink.h"
#include "utilities/vector.h"


Vector<float> incremental_vector_fill(int target_length) {
	Vector<float> data(target_length);
	Serial.println("Filling vector");
	Serial.print("Pre-op    \t"); data.print();
	for (int i = 0; i < target_length; i++) {
		data[i] = i;
	}
	Serial.print("Post-op    \t"); data.print();
	return data;
}

void incremental_index_fill_test(int target_length) {
	Serial.println("Incremental Fill vector");
	Vector<float> data = incremental_vector_fill(target_length);
	data.clear();
	Serial.print("Post-Clear\t"); data.print();
}

void from_array_fill_test(Vector<float> data, int target_length) {
	float test_data[target_length];

	for (int i = 0; i < target_length; i++) {
		test_data[i] = i;
	}
	Serial.println("From array vector");
	Serial.print("Pre-Fill    \t"); data.print();
	data.from_array(test_data, target_length);
	Serial.print("Post-Fill    \t"); data.print();
	data.clear();
	Serial.print("Post-Clear\t"); data.print();
}

void assign_operator_vector_test(Vector<float> v1, Vector<float> v2) {
	Serial.println("Assign vector");
	Serial.print("v1 Pre-op\t"); v1.print();
	Serial.print("v2       \t"); v2.print();
	v1 = v2;
	v2.reset(0);
	Serial.print("v1 Post-op\t"); v1.print();
	Serial.print("v2 Reset\t"); v2.print();
}

void reset_and_fill_test(int target_length) {
	Serial.println("Reset and Fill vector");
	Vector<float> data = incremental_vector_fill(target_length);
	data.reset(target_length);
	Serial.print("Post-Reset    \t"); data.print();

}

void vec_append_test(Vector<float> data) {
	Serial.println("Appending to vector");
	Serial.print("Pre-append dest\t"); data.print();
	Vector<float> v1 = incremental_vector_fill(data.size());
	Serial.print("Pre-append src\t"); v1.print();
	data.append(v1);
	Serial.print("Post-append\t"); data.print();
	data.clear();
	Serial.print("Post-Clear\t"); data.print();
}

void iterative_test(int size, int depth) {
	Vector<float> data(size);
	for (int i = 0; i < depth; i++) {
		Vector<float> v(size);
		data = v;
		Serial.printf("Iteration depth %i\n", i);
	}
}

void recursive_test(Vector<float>* data, int size, int depth) {
	Vector<float> v(size);
	for (int i = 0; i < size; i++) {
		v[i] = float(depth);
	}
	*data = v;
	if (depth <= 0) {
		Serial.println("Recursion max depth");
		return;
	}
	Serial.printf("Recursion depth %i\n", depth);
	recursive_test(data, size, depth - 1);
	Serial.printf("Recursion return depth %i\n", depth);
	return;
}


Vector<float> test_return() {

	float tmp[9] = {1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 1.0, 1.0, 1.0};
	Vector<float> v(9);
	v.from_array(tmp, 9);
	return v;
}

int main() {
	setup_blink();
	while(!Serial){}

	Serial.println("=== Starting Vector tests ===");
	Serial.println("=== Incremental fill Vector tests ===");

	int n = 5;
	incremental_index_fill_test(n);
	Vector<float> v = incremental_vector_fill(n);
	assign_operator_vector_test(Vector<float>(n), v);
	reset_and_fill_test(n);
	Vector<float> v2 = incremental_vector_fill(n);
	vec_append_test(v2);

	iterative_test(100, 100);

	Vector<float> data(0);
	recursive_test(&data, 10, 100);
	data.print();

	Serial.println("=== Finished Vector tests ===");
}