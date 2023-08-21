#include "utilities/vector.h"

/// print

template <> void Vector<int>::print() {
	if (length == 0) {
		printf("Vectori [%i]\n", length);
		return;
	}
	printf("Vectori [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		printf("%i, ", buffer[i]);
	}
	printf("%i]\n", buffer[length-1]);
}

template <> void Vector<float>::print() {
	if (length == 0) {
		printf("Vectorf [%i]\n", length);
		return;
	}
	printf("Vectorf [%i]: [", length);
	for (int i = 0; i < length-1; i++) {
		printf("%f, ", buffer[i]);
	}
	printf("%f]\n", buffer[length-1]);
}
