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

	Serial.println("=== Finished Vector tests ===");

}