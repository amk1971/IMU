
#ifndef MADGWICK_H
#define MADGWICK_H


typedef struct {
	float w,x,y,z;
} Quaternion;


typedef struct {
	float x,y,z;
} Vector3;

Quaternion madgwick_update(Quaternion q, Vector3 gyro, Vector3 accel, float dt, float beta);


Vector3 normalize (Vector3 v);

Quaternion quat_multiply(Quaternion q, Quaternion r);

void quat_to_euler_fast(float w, float x, float y, float z,
                        float *roll, float *pitch, float *yaw);

#endif
