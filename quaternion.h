#include "math.h"

void quat_add_vec(float *a, float *b, float *result);
void quat_sub_vec(float *a, float *b, float *result);
void quat_scale_vec(float s, float *v, float *result);
float quat_norm_vec(float *v);
void quat_vector_rotation(float *q, float *v, float *result);
void quat_integration_approx(float *q, float *omega, float dt, float *result);
void quat_get_control_anguler_velocity(float *q_c, float *q_d, float s, float k, float *result);
void quat_rotate_a_to_b(float *a, float *b, float *result);
void quat_rotate_a_to_b(float *a, float *b, float *result);