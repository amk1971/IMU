#include "ahrs_kalman.h"
#include <string.h>

/*
 * AHRS Kalman-style filter (math-lib-free) -- improved stability for pitch/roll.
 *
 * Changes in this version (vs prior):
 * - Robust atan/atan2 approximations that handle large inputs (avoids huge angle measurements
 *   when denom is very small).
 * - Proper conversion of body angular rates (p,q,r) to Euler angle rates
 *   (phi_dot, theta_dot, psi_dot) before integrating. This fixes the root cause of
 *   runaway pitch/roll when gyros are integrated directly as if they were Euler rates.
 * - Safeguards: small-cos protection, clamping of P and state to avoid numeric blowup,
 *   skip measurement update if measurement is invalid (NaN/huge).
 *
 * Keep the same public API in ahrs_kalman.h.
 */

/* internal state */
static float state_angle_rad[3];   /* [0]=pitch (theta), [1]=roll (phi), [2]=yaw (psi) */
static float P[3];                 /* variance for each angle */
static float Q[3];                 /* process noise variance */
static float R_accel;
static float R_mag;
static int initialized = 0;

/* fallback SAMPLE_FREQ if not defined elsewhere */
#ifndef SAMPLE_FREQ
#define SAMPLE_FREQ 100.0f
#endif

/* constants */
static const float PI = 3.14159265358979323846f;
static const float TWO_PI = 6.28318530717958647692f;
static const float HALF_PI = 1.57079632679489661923f;

/* ---------- Small helper math approximations (no math.h) ---------- */

static inline float my_fabs(float x) { return x < 0.0f ? -x : x; }

/* fast sqrt approximation using 3 Newton-Raphson iterations */
static float sqrt_approx(float x) {
    if (x <= 0.0f) return 0.0f;
    float g = x >= 1.0f ? x : 1.0f;
    g = 0.5f * (g + x / g);
    g = 0.5f * (g + x / g);
    g = 0.5f * (g + x / g);
    return g;
}

/* Improved atan approximation with handling for |x| > 1 using identity:
   atan(x) = sign(x) * PI/2 - atan(1/x)  for large |x|.
   For |x| <= 1 we use the fast polynomial approx previously used.
*/
static float atan_approx(float x) {
    float ax = my_fabs(x);
    if (ax <= 1.0f) {
        /* polynomial approx valid near zero */
        return (PI * 0.25f) * x - x * (ax - 1.0f) * (0.2447f + 0.0663f * ax);
    } else {
        /* use reciprocal identity for better stability on large inputs */
        float inv = 1.0f / x;
        float a_inv = (PI * 0.25f) * inv - inv * (my_fabs(inv) - 1.0f) * (0.2447f + 0.0663f * my_fabs(inv));
        if (x > 0.0f) return HALF_PI - a_inv;
        else return -HALF_PI - a_inv;
    }
}

/* Robust atan2 approximation that avoids direct division by tiny x by choosing
   the formulation depending on magnitudes of x and y.
*/
float atan2_approx(float y, float x) {
    if (x == 0.0f) {
        if (y > 0.0f) return HALF_PI;
        if (y < 0.0f) return -HALF_PI;
        return 0.0f;
    }
    float ax = my_fabs(x), ay = my_fabs(y);

    /* if |y| <= |x| compute atan(y/x) with quadrant handling */
    if (ay <= ax) {
        float a = atan_approx(y / x);
        if (x > 0.0f) {
            return a;
        } else {
            return (y >= 0.0f) ? (a + PI) : (a - PI);
        }
    } else {
        /* |y| > |x|: compute using identity atan2(y,x) = sign(y)*(PI/2) - atan(x/y) */
        float a = atan_approx(x / y);
        return (y > 0.0f) ? (HALF_PI - a) : (-HALF_PI - a);
    }
}

/* Reduce angle to range [-PI, PI] */
static float wrap_pi(float a) {
    while (a > PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

/* sine/cosine approximations with simple range handling */
static void reduce_angle_quadrant(float *x, int *sign_sin, int *sign_cos) {
    float a = wrap_pi(*x);
    int s_sin = 1, s_cos = 1;

    if (a > HALF_PI) {
        a = PI - a;
        s_sin = 1; s_cos = -1;
    } else if (a < -HALF_PI) {
        a = -PI - a;
        s_sin = -1; s_cos = -1;
    }
    *x = a;
    *sign_sin = s_sin; *sign_cos = s_cos;
}

static float sin_approx(float x) {
    int ss=1, sc=1;
    reduce_angle_quadrant(&x, &ss, &sc);
    float x2 = x*x;
    float s = x * (1.0f - x2 * (1.0f/6.0f) + x2 * x2 * (1.0f/120.0f));
    return ss * s;
}

static float cos_approx(float x) {
    int ss=1, sc=1;
    reduce_angle_quadrant(&x, &ss, &sc);
    float x2 = x*x;
    float c = 1.0f - x2 * 0.5f + x2 * x2 * (1.0f/24.0f);
    return sc * c;
}

/* ---------- Sensor -> angle helpers using approximations ---------- */

static void accel_to_pitch_roll(float ax, float ay, float az, float *pitch_rad, float *roll_rad) {
    float denom = sqrt_approx(ay*ay + az*az);
    if (denom < 1e-6f) denom = 1e-6f;
    *pitch_rad = atan2_approx(-ax, denom);
    *roll_rad  = atan2_approx(ay, az);
}

static float mag_to_yaw(float mx, float my, float mz, float pitch, float roll) {
    float sp = sin_approx(pitch);
    float cp = cos_approx(pitch);
    float sr = sin_approx(roll);
    float cr = cos_approx(roll);

    float Xh = mx * cp + mz * sp;
    float Yh = mx * sr * sp + my * cr - mz * sr * cp;

    if (my_fabs(Xh) < 1e-9f && my_fabs(Yh) < 1e-9f) {
        return state_angle_rad[2];
    }
    float heading = atan2_approx(-Yh, Xh);
    return wrap_pi(heading);
}

/* ---------- Public API implementation ---------- */

void AHRS_KalmanFilter_init(float init_pitch_deg, float init_roll_deg, float init_yaw_deg)
{
    state_angle_rad[0] = init_pitch_deg * (PI / 180.0f);
    state_angle_rad[1] = init_roll_deg  * (PI / 180.0f);
    state_angle_rad[2] = init_yaw_deg   * (PI / 180.0f);

    P[0] = 0.05f; P[1] = 0.05f; P[2] = 0.1f;
    Q[0] = 0.001f; Q[1] = 0.001f; Q[2] = 0.05f;
    R_accel = 0.03f; R_mag = 0.08f;

    initialized = 1;
}

/* helper: simple numeric validity check (no math.h) */
static int is_valid_f(float v) {
    /* NaN check: NaN != NaN */
    if (v != v) return 0;
    /* clamp extreme values */
    if (my_fabs(v) > 1e8f) return 0;
    return 1;
}

void AHRS_KalmanFilter_update(float accel_x_g, float accel_y_g, float accel_z_g,
                              float gyro_x_dps, float gyro_y_dps, float gyro_z_dps,
                              float mag_x_T, float mag_y_T, float mag_z_T,
                              float dt,
                              float *out_pitch_deg, float *out_roll_deg, float *out_yaw_deg)
{
    if (!initialized) AHRS_KalmanFilter_init(0.0f, 0.0f, 0.0f);
    if (!(dt > 0.0f)) dt = 1.0f / SAMPLE_FREQ;

    /* convert gyro deg/s -> rad/s */
    const float deg2rad = PI / 180.0f;
    float p = gyro_x_dps * deg2rad; /* body rate about X */
    float q = gyro_y_dps * deg2rad; /* about Y */
    float r = gyro_z_dps * deg2rad; /* about Z */

    /* map state indices to Euler conventional names */
    float theta = state_angle_rad[0]; /* pitch */
    float phi   = state_angle_rad[1]; /* roll  */
    float psi   = state_angle_rad[2]; /* yaw   */

    /* ---------- Prediction: convert body rates (p,q,r) -> Euler angle rates ---------- */
    /* Equations:
       phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
       theta_dot = q*cos(phi) - r*sin(phi)
       psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)
       Use approximated sin/cos and protect against cos(theta) near zero.
    */
    float sp = sin_approx(phi);
    float cp = cos_approx(phi);
    float st = sin_approx(theta);
    float ct = cos_approx(theta);
    /* protect cos(theta) */
    float safe_ct = (my_fabs(ct) < 1e-6f) ? (ct >= 0.0f ? 1e-6f : -1e-6f) : ct;
    float tan_t = st / safe_ct;

    float phi_dot = p + q * sp * tan_t + r * cp * tan_t;
    float theta_dot = q * cp - r * sp;
    float psi_dot = (q * sp) / safe_ct + (r * cp) / safe_ct;

    /* integrate */
    phi   += phi_dot * dt;
    theta += theta_dot * dt;
    psi   += psi_dot * dt;

    /* store back */
    state_angle_rad[0] = theta;
    state_angle_rad[1] = phi;
    state_angle_rad[2] = wrap_pi(psi);

    /* grow uncertainty a bit (process noise) */
    P[0] += Q[0];
    P[1] += Q[1];
    P[2] += Q[2];

    /* clamp P to reasonable bounds to avoid numerical explosion */
    for (int i = 0; i < 3; ++i) {
        if (!(P[i] > 0.0f) || P[i] > 1e6f) P[i] = 1e6f;
        if (P[i] < 1e-12f) P[i] = 1e-12f;
    }

    /* ---------- Measurement from accelerometer (pitch & roll) ---------- */
    float meas_pitch, meas_roll;
    accel_to_pitch_roll(accel_x_g, accel_y_g, accel_z_g, &meas_pitch, &meas_roll);

    /* validate measurements */
    if (is_valid_f(meas_pitch) && is_valid_f(meas_roll)) {
        float K_pitch = P[0] / (P[0] + R_accel + 1e-12f);
        state_angle_rad[0] = state_angle_rad[0] + K_pitch * (meas_pitch - state_angle_rad[0]);
        P[0] = (1.0f - K_pitch) * P[0];

        float K_roll = P[1] / (P[1] + R_accel + 1e-12f);
        state_angle_rad[1] = state_angle_rad[1] + K_roll * (meas_roll - state_angle_rad[1]);
        P[1] = (1.0f - K_roll) * P[1];
    } else {
        /* skip accel update if invalid */
    }

    /* ---------- Measurement from magnetometer (tilt-compensated yaw) ---------- */
    float meas_yaw = mag_to_yaw(mag_x_T, mag_y_T, mag_z_T, state_angle_rad[0], state_angle_rad[1]);

    if (is_valid_f(meas_yaw)) {
        float innov_yaw = meas_yaw - state_angle_rad[2];
        if (innov_yaw > PI) innov_yaw -= TWO_PI;
        if (innov_yaw < -PI) innov_yaw += TWO_PI;

        float K_yaw = P[2] / (P[2] + R_mag + 1e-12f);
        state_angle_rad[2] = state_angle_rad[2] + K_yaw * innov_yaw;
        P[2] = (1.0f - K_yaw) * P[2];
        state_angle_rad[2] = wrap_pi(state_angle_rad[2]);
    }

    /* final safety: if any state goes NaN or extremely large, clamp/reset mildly */
    for (int i = 0; i < 3; ++i) {
        if (!is_valid_f(state_angle_rad[i])) state_angle_rad[i] = 0.0f;
        if (my_fabs(state_angle_rad[i]) > 1e6f) state_angle_rad[i] = wrap_pi(state_angle_rad[i]);
    }

    if (out_pitch_deg) *out_pitch_deg = state_angle_rad[0] * (180.0f / PI);
    if (out_roll_deg)  *out_roll_deg  = state_angle_rad[1] * (180.0f / PI);
    if (out_yaw_deg)   *out_yaw_deg   = state_angle_rad[2] * (180.0f / PI);
}
