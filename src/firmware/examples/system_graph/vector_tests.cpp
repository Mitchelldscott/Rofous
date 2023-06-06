#include <Arduino.h>
#include "utilities/vector.h"


Vector<float> test_return() {

	// float accel[3] = {1.0, 2.0, 3.0};
	// float gyro[3] = {-1.0, -2.0, -3.0};
	// float mag[3] = {1.0, 1.0, 1.0};
	float tmp[9] = {1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 1.0, 1.0, 1.0};
	Vector<float> v(9);
	v.from_vec(tmp, 9);
	return v;
}

int main() {
	while(!Serial){}

	Serial.println("=== Starting Vector tests ===");

	int n = 10;
	Vector<float> v(n);

	Serial.printf("Initializing %i\n", n);
	v.print();
	Serial.println("Adding 0-N");
	for (int i = 0; i < n; i++) {
		v[i] = i;
	}
	v.print();

	Vector<float> v1(n);
	Serial.println("Copying vector");
	v1 = v;
	v1.print();

	Serial.println("Clearing");
	v.clear();
	v.print();

	n = int(n * 1.5);

	Serial.printf("====\nInitializing %i\n", n);
	v.reset(n);
	v.print();
	Serial.println("Adding 0-N");
	for (int i = 0; i < n; i++) {
		v[i] = i;
	}
	v.print();
	Serial.println("Clearing");
	v.clear();
	v.print();

	n = 2;
	int m = 5;

	Serial.printf("====\nInitializing %i from float*\n", m);
	Vector<float> v3(1);
	float test_data[m];

	for (int i = 0; i < m; i++) {
		test_data[i] = i;
	}

	v3.from_vec(test_data, m);
	v3.print();

	Serial.println("Appending v3 to v3");

	v3.append(test_data, m);
	v3.print();

	Serial.println("Testing From return");
	Vector<float> v4 = test_return();
	v4.print();


	Serial.println("=== Finished Vector tests ===");
}