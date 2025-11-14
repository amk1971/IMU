
#include "Madgwick.h"
#include "math.h"



float fast_sqrt(float n) {
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = n * 0.5F;
    y  = n;
    i  = *(long *)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float *)&i;

    // 1 iteration of Newton-Raphson
    y  = y * (threehalfs - (x2 * y * y));

    return n * y;
}

static inline float fast_abs(float x) {
    return x < 0 ? -x : x;
}

static inline float fast_sign(float x) {
    return (x >= 0) ? 1.0f : -1.0f;
}
float fast_asin(float x)
{
    float absx = fast_abs(x);
    if (absx > 1.0f) absx = 1.0f;

    float root = fast_sqrt(1.0f - absx);

    float result =
        1.5707963f -
        root * (1.5707288f - 0.2121144f*absx + 0.0742610f*absx*absx - 0.0187293f*absx*absx*absx);

    return fast_sign(x) * result;
}


float fast_atan2(float y, float x)
{
    if (x == 0.0f) {
        return (y > 0 ? 1.5707963f : -1.5707963f); // π/2
    }

    float abs_y = fast_abs(y) + 1e-10f;
    float r, angle;

    if (x >= 0) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.78539816f - 0.78539816f * r; // π/4
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 2.35619449f - 0.78539816f * r; // 3π/4
    }

    return (y < 0) ? -angle : angle;
}




Vector3 normalize (Vector3 v){
	float sum = fast_sqrt(v.x*v.x + v.y* v.y + v.z * v.z);

	Vector3 result = {v.x/sum, v.y/sum , v.z/sum};

	return result;
}

Quaternion quat_multiply(Quaternion q, Quaternion r) {
    Quaternion res;
    res.w = q.w*r.w - q.x*r.x - q.y*r.y - q.z*r.z;
    res.x = q.w*r.x + q.x*r.w + q.y*r.z - q.z*r.y;
    res.y = q.w*r.y - q.x*r.z + q.y*r.w + q.z*r.x;
    res.z = q.w*r.z + q.x*r.y - q.y*r.x + q.z*r.w;
    return res;
}



Quaternion madgwick_update(Quaternion q, Vector3 gyro, Vector3 accel, float dt, float beta){

	accel = normalize(accel);

	gyro.x *= 0.01745329251f;
	gyro.y *= 0.01745329251f;
	gyro.z *= 0.01745329251f;

    // Predicted gravity from quaternion
    float f1 = 2*(q.x*q.z - q.w*q.y) - accel.x;
    float f2 = 2*(q.w*q.x + q.y*q.z) - accel.y;
    float f3 = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z - accel.z;

    // Gradient of f w.r.t q (simplified version)
       float grad_w = -2*f2*q.x + 2*f3*q.w;
       float grad_x =  2*f1*q.z + 2*f2*q.w - 2*f3*q.x;
       float grad_y =  2*f2*q.z - 2*f3*q.y - 2*f1*q.w;
       float grad_z = -2*f1*q.x + 2*f2*q.y + 2*f3*q.z;

       float norm = fast_sqrt(grad_w*grad_w + grad_x*grad_x + grad_y*grad_y + grad_z*grad_z);
       grad_w /= norm; grad_x /= norm; grad_y /= norm; grad_z /= norm;

       // Gyro quaternion
       Quaternion q_gyro = {0, gyro.x, gyro.y, gyro.z};

       // Quaternion derivative from gyro
       Quaternion q_dot = quat_multiply(q, q_gyro);
       q_dot.w *= 0.5f; q_dot.x *= 0.5f; q_dot.y *= 0.5f; q_dot.z *= 0.5f;

       // Add correction from gradient descent
         q_dot.w -= beta * grad_w;
         q_dot.x -= beta * grad_x;
         q_dot.y -= beta * grad_y;
         q_dot.z -= beta * grad_z;

         // Integrate to get new quaternion
         q.w += q_dot.w * dt;
         q.x += q_dot.x * dt;
         q.y += q_dot.y * dt;
         q.z += q_dot.z * dt;

         // Normalize quaternion
         norm = fast_sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
         q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;

         return q;
}


void quat_to_euler_fast(float w, float x, float y, float z,
                        float *roll, float *pitch, float *yaw)
{
    // --- Roll (x-axis rotation) ---
    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    *roll = fast_atan2(sinr_cosp, cosr_cosp);

    // --- Pitch (y-axis rotation) ---
    float sinp = 2.0f * (w*y - z*x);

    if (fast_abs(sinp) >= 1.0f)
        *pitch = fast_sign(sinp) * 1.5707963f;   // clamp to ±90°
    else
        *pitch = fast_asin(sinp);

    // --- Yaw (z-axis rotation) ---
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    *yaw = fast_atan2(siny_cosp, cosy_cosp);

    float rad2deg = 180.0f / M_PI;

    *roll  *= rad2deg;
    *pitch *= rad2deg;
    *yaw   *= rad2deg;
}


