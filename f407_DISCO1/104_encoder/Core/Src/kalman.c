/*
 * kalman.c
 *
 * Created on: Sep 6, 2025
 * Author: conne
 *
 * Source file for Kalman filter implementation.
 */
#include "kalman.h"

// Time step for the filter, representing the sampling period in seconds.
const float dt = 0.025;
// Global Kalman filter instances for pitch, roll, and yaw angles.
KalmanFilter kalman_pitch, kalman_roll, kalman_yaw;

/**
 * @brief Initializes a Kalman filter instance.
 * @param kf Pointer to the KalmanFilter structure to be initialized.
 */
void Kalman_Init(KalmanFilter *kf) {
    kf->angle = 0.0;
    kf->bias = 0.0;
    // Initialize the error covariance matrix (P).
    kf->P[0][0] = 1.0;  kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;  kf->P[1][1] = 1.0;
}

/**
 * @brief Updates the Kalman filter with new sensor data.
 * @param kf Pointer to the KalmanFilter structure.
 * @param newAngle New angle measurement from the accelerometer (e.g., in degrees).
 * @param newRate New angular rate measurement from the gyroscope (e.g., in deg/s).
 * @param dt Time step since the last update.
 * @return The updated and filtered angle.
 */
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // 1. Prediction Step
    // Update the state estimate using the gyroscope data.
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update the error covariance matrix (P) based on the state transition model.
    // The constants (0.001 and 0.003) represent the process noise Q.
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + 0.001);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += 0.003;

    // 2. Correction Step
    // Calculate the innovation covariance (S) using the measurement noise R (0.03).
    kf->S = kf->P[0][0] + 0.03;
    // Calculate the Kalman gain (K).
    kf->K[0] = kf->P[0][0] / kf->S;
    kf->K[1] = kf->P[1][0] / kf->S;

    // Calculate the measurement residual (y).
    kf->y = newAngle - kf->angle;
    // Update the state estimate (angle and bias) using the Kalman gain.
    kf->angle += kf->K[0] * kf->y;
    kf->bias += kf->K[1] * kf->y;

    // Update the error covariance matrix after the correction.
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= kf->K[0] * P00_temp;
    kf->P[0][1] -= kf->K[0] * P01_temp;
    kf->P[1][0] -= kf->K[1] * P00_temp;
    kf->P[1][1] -= kf->K[1] * P01_temp;

    return kf->angle;
}
