
#ifndef CURO_LINALG_H
#define CURO_LINALG_H

float wrap_angle(float angle) {
	while (angle >= PI) {
		angle -= 2 * PI;
	}

	while (angle < -PI) {
		angle += 2 * PI;
	}

	return angle;
}

void rotate2D(float* v, float* v_tf, float angle) {
	v_tf[0] = (v[0] * cos(angle)) - (v[1] * sin(angle));
	v_tf[1] = (v[0] * sin(angle)) + (v[1] * cos(angle));
}

float norm(float* v, int n) {
	float sum = 0;
	for (int i = 0; i < n; i++) {
		sum += v[i] * v[i];
	}
	return sqrt(sum);
}

float vector_product(float* a, float* b, int n) {
	int ret = 0;
	for (int i = 0; i < n; i++) {
		ret += a[i] * b[i];
	}

	return ret;
}

float cross_product2D(float* a, float* b) {
	return (a[0] * b[1]) - (a[1] * b[0]);
}

void weighted_vector_addition(float* a, float* b, float k1, float k2, int n, float* output) {
	for (int i = 0; i < n; i++) {
		output[i] = (k1 * a[i]) + (k2 * b[i]);
	}
}

#endif