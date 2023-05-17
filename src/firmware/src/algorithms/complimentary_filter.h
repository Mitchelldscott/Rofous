#include "algorithms/linear_algebra.h"

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

class ComplimentaryFilter {
private:
    float K = 0.4;

public:
    ComplimentaryFilter() {}

    void set_gain(float k) {
        K = k;
    }

    float get_gain() {
        return K;
    }


    void filter(float* accel, float* gyro, float* mag, float dt, float* estimate) {
        float q_accel[3];
        float q_omega[3];
        float axy_norm = nd_norm(accel, 2);
        float mag_norm = nd_norm(mag, 3);

        // Calulate attitude using accelerations + magnetometer and trig
        q_accel[0] = atan2(accel[1], axy_norm);
        q_accel[1] = atan2(-accel[0], accel[1]);

        float opposite = ((mag[2]*sin(q_accel[0])) - (mag[1]*cos(q_accel[0]))) / mag_norm;
        float hypotenuse = (mag[0]*cos(q_accel[1])) + (sin(q_accel[1]) * (mag[1]*cos(q_accel[0])) + (mag[2]*sin(q_accel[0]))) / mag_norm;

        q_accel[2] = atan2(opposite, hypotenuse);

        // integrate omegas over time
        weighted_vector_addition(estimate, gyro, 1, dt, 3, q_omega);

        weighted_vector_addition(q_accel, q_omega, K, 1-K, 3, estimate);
    }
};

#endif