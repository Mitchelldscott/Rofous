#ifndef CURO_LINALG
#define CURO_LINALG

float nd_norm(float* v, int n);

float wrap_angle(float angle);

float cross_product2D(float* a, float* b);

void rotate2D(float* v, float* v_tf, float angle);

float vector_product(float* a, float* b, int n);

void weighted_vector_addition(float* a, float* b, float k1, float k2, int n, float* output);

#endif