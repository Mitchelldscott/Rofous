#include <Arduino.h>
#include "system_graph/vector.h"

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
	Serial.println("Clearing");
	v.clear();
	v.print();

	n = n * 2;

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

	Vector<Vector<float>> v2(n);

	Serial.printf("====\nInitializing %ix%i\n", n, m);
	v2.reset(n);
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

	Serial.println("=== Finished Vector tests ===");

}