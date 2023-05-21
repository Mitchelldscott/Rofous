#include "algorithms/linear_algebra.h"

#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#define ATTITUDE_DIM 3

class ComplimentaryFilter {
private:
    float K = 0.4;
    float axz_norm = 0;
    float ayz_norm = 0;
    float mag_norm = 0;
    float q_gyro[ATTITUDE_DIM];
    float q_accel[ATTITUDE_DIM];
    float bias_accel[ATTITUDE_DIM];


public:
    ComplimentaryFilter() {}

    void set_gain(float k) {
        K = k;
    }

    float get_gain() {
        return K;
    }


    void filter(float* accel, float* gyro, float* mag, float dt, float* estimate) {
        float axz[2] = {accel[0], accel[2]};
        float ayz[2] = {accel[1], accel[2]};
        axz_norm = nd_norm(axz, ATTITUDE_DIM - 1);
        ayz_norm = nd_norm(ayz, ATTITUDE_DIM - 1);
        mag_norm = nd_norm(mag, ATTITUDE_DIM);

        // Calulate attitude using accelerations + magnetometer and trig
        q_accel[0] = atan2(accel[1], axz_norm);
        q_accel[1] = atan2(-accel[0], ayz_norm);

        float opposite = ((mag[2]*sin(q_accel[0])) - (mag[1]*cos(q_accel[0]))) / mag_norm;
        float adjacent = ((mag[0]*cos(q_accel[1])) + (sin(q_accel[1]) * (mag[1]*cos(q_accel[0])) + (mag[2]*sin(q_accel[0])))) / mag_norm;

        q_accel[2] = atan2(opposite, adjacent);
        if (isnan(q_accel[2])) {
            q_accel[2] = 0;
        }

        // integrate gyro measurements over time
        weighted_vector_addition(estimate, gyro, 1, dt, 3, q_gyro);

        // fuse the accel and gyro estimates
        weighted_vector_addition(q_accel, q_gyro, K, 1-K, 3, estimate);
    }
};

#endif