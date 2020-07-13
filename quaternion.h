#include "math.h"

void quat_add_vec(float a[3], float b[3], float result[3]);
void quat_sub_vec(float a[3], float b[3], float result[3]);
void quat_scale_vec(float s, float v[3], float result[3]);
float quat_norm_vec(float v[3]);
void quat_vector_rotation(float q[4], float v[3], float result[3]);
void quat_integration_approx(float q[4], float omega[3], float dt, float result[4]);
void quat_get_control_anguler_velocity(float q_c[4], float q_d[4], float s, float k, float result[3]);
void quat_rotate_a_to_b(float a[3], float b[3], float result[4]);