#include <Arduino.h>
#include "system_graph/vector.h"


Vector<Vector<float>> test_return() {

	float accel[3] = {1.0, 2.0, 3.0};
	float gyro[3] = {-1.0, -2.0, -3.0};
	float mag[3] = {1.0, 1.0, 1.0};
	Vector<Vector<float>> v(3, 3);
	v[0].from_vec(accel, 3);
	v[1].from_vec(gyro, 3);
	v[2].from_vec(mag, 3);
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

	Vector<Vector<float>> v2(n, m);

	Serial.printf("====\nInitializing %ix%i\n", n, m);
	v2.print();

	Serial.println("Adding 0-N");
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			v2[i][j] = i;
		}
	}
	
	v2.print();
	Serial.println("Clearing");
	v2.clear();
	v2.print();

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
	Vector<Vector<float>> v4 = test_return();
	v4.print();

	Serial.println("=== Finished Vector tests ===");
}