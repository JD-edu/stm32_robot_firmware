/*
 * kalman.h
 *
 * Created on: Sep 6, 2025
 * Author: conne
 *
 * Header file for Kalman filter implementation.
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

// External variable for the time step, defined in kalman.c
extern const float dt;

// Kalman filter state structure
typedef struct {
    float angle;  // Estimated angle
    float bias;   // Gyroscope bias
    float rate;   // Gyroscope raw measurement

    float P[2][2];  // Error covariance matrix
    float K[2];     // Kalman gain
    float y;        // Measurement residual
    float S;        // Innovation covariance
} KalmanFilter;

// External declarations for the global Kalman filter instances
extern KalmanFilter kalman_pitch, kalman_roll, kalman_yaw;

// Function prototypes
void Kalman_Init(KalmanFilter *kf); // Initializes a Kalman filter instance
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt); // Updates the filter with new sensor data

#endif /* INC_KALMAN_H_ */
