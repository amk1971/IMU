#ifndef AHRS_KALMAN_H_
#define AHRS_KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AHRS Kalman-style filter (decoupled scalar Kalman per angle).
 *
 * These functions do not alter your existing code; they are standalone and
 * keep internal state across calls. Call AHRS_KalmanFilter_init() once at
 * startup if you want to set an initial attitude (otherwise it starts at 0,0,0).
 *
 * Units:
 *  - accel_*_g : accelerometer in g (e.g. 1.0 = 1g)
 *  - gyro_*_dps: gyroscope in degrees per second
 *  - mag_*_T   : magnetometer in Tesla
 *  - dt        : time step in seconds (if <=0, 1/SAMPLE_FREQ is used by the implementation)
 *
 * Outputs (by pointer):
 *  - out_pitch_deg, out_roll_deg, out_yaw_deg  : angles in degrees
 */

void AHRS_KalmanFilter_init(float init_pitch_deg, float init_roll_deg, float init_yaw_deg);

void AHRS_KalmanFilter_update(float accel_x_g, float accel_y_g, float accel_z_g,
                              float gyro_x_dps, float gyro_y_dps, float gyro_z_dps,
                              float mag_x_T, float mag_y_T, float mag_z_T,
                              float dt,
                              float *out_pitch_deg, float *out_roll_deg, float *out_yaw_deg);
float atan2_approx(float y, float x);

//float atan_approx(float x);

#ifdef __cplusplus
}
#endif

#endif // AHRS_KALMAN_H_
