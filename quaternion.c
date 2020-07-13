// ----------------------------------------------------
// Quaternion calculate functions
// ----------------------------------------------------
// FPU搭載マイコン向けの四元数演算関数群．
// 拡張子をcppに変更することでC++環境でも利用可能．
// 引数に渡した変数は，関数内部で変更されない．
//
// ベクトルの扱いは3次元配列とする．ベクトルv[3]と四元数q[4]のベクトル部の演算を行う場合は，
// add_vec(v, &q[1], result);
// のようにしてベクトル部の先頭アドレスを伝えてあげれば良い．
// 特別な記載が無い限り，名前の末尾にvecが付いている関数の引数は3次元配列(Vector3)．
// それ以外は4次元配列(Quaternion)．
//
// 計算結果がスカラとなる関数は，関数の返り値として計算結果を返す．
// 計算結果がベクトルや四元数になる関数は，計算結果を入れる配列を事前に確保しておき，
// 関数のresult引数に配列の先頭アドレスを渡すとそこに格納される．例えば以下のようにする．
// 
// float s = 2.0f;
// float q[4] = {1.0f, 2.0f, 3.0f, 4.0f};
// float result[4];
// quat_scale(s, q, result);
// 結果: [2.0, 4.0, 6.0, 8.0]
//
// memcpyはbyte指定であることに注意

#include <stdint.h>
#include <string.h>

#include "math.h"

const float PI = 3.1415926535f;
const float EPSILON = 1e-7;


float acos_safe(float x) {
    if ( fabsf(x) >= 1.0f ) {
        if (x > 0.0f) {
            return 0.0f;
        } else {
            return PI;
        }
    } else {
        return acosf(x);
    }
}

// resultにゼロベクトルを入れる
void quat_zero_vec(float result[3]) {
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
}

// resultに恒等四元数を入れる
void quat_identity(float result[4]) {
    result[0] = 1.0f;
    quat_zero_vec(&result[1]);
}

float quat_dot_vec(float a[3], float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float quat_dot(float a[4], float b[4]) {
    return a[0]*b[0] + quat_dot_vec(&a[1], &b[1]);
}

void quat_cross_vec(float a[3], float b[3], float result[3]) {
    // quat_cross_vec(a, b, a) みたいなことをした時に計算結果が
    // おかしくならないように，計算結果を一時変数に確保する
    float tmp[3];
    tmp[0] = a[1]*b[2] - a[2]*b[1];
    tmp[1] = a[2]*b[0] - a[0]*b[2];
    tmp[2] = a[0]*b[1] - a[1]*b[0];

    memcpy(result, tmp, 4*3);
}

void quat_add_vec(float a[3], float b[3], float result[3]) {
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];
}

void quat_add(float a[4], float b[4], float result[4]) {
    result[0] = a[0] + b[0];
    quat_add_vec(&a[1], &b[1], &result[1]);
}

void quat_sub_vec(float a[3], float b[3], float result[3]) {
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

void quat_sub(float a[4], float b[4], float result[4]) {
    result[0] = a[0] - b[0];
    quat_sub_vec(&a[1], &b[1], &result[1]);
}

void quat_scale_vec(float s, float v[3], float result[3]) {
    result[0] = s * v[0];
    result[1] = s * v[1];
    result[2] = s * v[2];
}

void quat_scale(float s, float q[4], float result[4]) {
    result[0] = s * q[0];
    quat_scale_vec(s, &q[1], &result[1]);
}

// result = s*a + b
void quat_scale_add_vec(float s, float a[3], float b[3], float result[3]) {
    // FMAが使える場合
    //result[0] = fma(s, a[0], b[0]);
    //result[1] = fma(s, a[1], b[1]);
    //result[2] = fma(s, a[2], b[2]);

    // FMAが使えない場合
    float tmp[3];
    quat_scale_vec(s, a, tmp);
    quat_add_vec(tmp, b, result);
}

// result = s*a + b
void quat_scale_add(float s, float a[4], float b[4], float result[4]) {
    // FMAが使える場合
    //result[0] = fma(s, a[0], b[0]);
    //quat_scale_add_vec(s, &a[1], &b[1], &result[1]);

   // FMAが使えない場合
    float tmp[4];
    quat_scale(s, a, tmp);
    quat_add(tmp, b, result);
}

void quat_negate_vec(float v[3], float result[3]) {
    result[0] = -v[0];
    result[1] = -v[1];
    result[2] = -v[2];
}

void quat_negate(float q[4], float result[4]) {
    result[0] = -q[0];
    quat_negate_vec(&q[1], &result[1]);
}

void quat_conj(float q[4], float result[4]) {
    result[0] = q[0];
    quat_negate_vec(&q[1], &result[1]);
}

float quat_norm_vec(float v[3]) {
    return sqrtf( quat_dot_vec(v, v) );
}

float quat_norm(float q[4]) {
    return sqrtf( quat_dot(q, q) );
}

// ベクトルを正規化
// 零ベクトルを入力した場合には零ベクトルを返す．
void quat_normalize_vec(float v[3], float result[3]) {
    float norm = quat_norm_vec(v);
    if (norm < EPSILON) {
        quat_zero_vec(result);
    } else {
        quat_scale_vec(1.0f / norm, v, result);
    }
}

// qを正規化
void quat_normalize(float q[4], float result[4]) {
    float inv_sqrt = 1.0f / quat_norm(q);
    quat_scale(inv_sqrt, q, result);
}

// 純虚四元数同士の積
// a: Vector3
// b: Vector3
// result: Quaternion
void quat_mul_vec(float a[3], float b[3], float result[4]) {
    result[0] = -quat_dot_vec(a, b);
    quat_cross_vec(a, b, &result[1]);
}

// ハミルトン積
// a, b, result: Quaternion
void quat_mul(float a[4], float b[4], float result[4]) {
    float tmp0[3];
    float tmp1[3];
    float term1[4];
    float term2[4];
    quat_scale_vec(a[0], &b[1], tmp0);
    quat_scale_vec(b[0], &a[1], tmp1);
    term1[0] = a[0] * b[0];
    quat_add_vec(tmp0, tmp1, &term1[1]);
    quat_mul_vec(&a[1], &b[1], term2);
    quat_add(term1, term2, result);
}

// versorの対数函数
// 恒等四元数において特異点
// q: Quaternion
// result: Vector3
void quat_ln_versor(float q[4], float result[3]) {
    float norm_vec = quat_norm_vec(&q[1]);
    float coef;
    if (norm_vec < EPSILON) {
        quat_zero_vec(result);
    } else {
        coef = acos_safe(q[0]) / norm_vec;
        quat_scale_vec(coef, &q[1], result);
    }
}

// versorの冪函数
void quat_pow_versor(float q[4], float t, float result[4]) {
    float tmp;
    float norm_vec = quat_norm_vec(&q[1]);
    if (norm_vec < EPSILON) {
        quat_identity(result);
    } else {
        tmp = t * acos_safe(q[0]);
        result[0] = cosf(tmp);
        quat_scale_vec( sinf(tmp) / norm_vec, &q[1], &result[1] );
    }
}

// 位置ベクトルの回転 (q v q*)
// q: Quaternion
// v: Vector3
// result: Vector3
void quat_vector_rotation(float q[4], float v[3], float result[3]) {
    float cross[3];
    float term1[3];
    float term2[3];
    float term3[3];
    float term4[3];
    quat_cross_vec(&q[1], v, cross);
    quat_scale_vec(q[0], v, term1);
    quat_scale_vec(2.0f, cross, term2);
    quat_scale_vec( quat_dot_vec(&q[1], v), &q[1], term3 );
    quat_cross_vec(&q[1], cross, term4);
    quat_add_vec(term1, term2, term1);
    quat_add_vec(term3, term4, term2);
    quat_scale_add_vec(q[0], term1, term2, result);
}

// 座標系の回転 (q* v q)
// q: Quaternion
// v: Vector3
// result: Vector3
void quat_frame_rotation(float q[4], float v[3], float result[3]) {
    float cross[3];
    float term1[3];
    float term2[3];
    float term3[3];
    float term4[3];
    quat_cross_vec(&q[1], v, cross);
    quat_scale_vec(q[0], v, term1);
    quat_scale_vec(2.0f, cross, term2);
    quat_scale_vec( quat_dot_vec(&q[1], v), &q[1], term3 );
    quat_cross_vec(&q[1], cross, term4);
    quat_sub_vec(term1, term2, term1);
    quat_add_vec(term3, term4, term2);
    quat_scale_add_vec(q[0], term1, term2, result);
}

// 位置ベクトル回転を積分
// q: Quaternion
// omega: Vector3
// result: Quaternion
// result = q + 0.5*dt*ωq
void quat_integration_approx(float q[4], float omega[3], float dt, float result[4]) {
    float tmp0[3];
    float tmp1[4];
    quat_scale_vec(q[0], omega, tmp0);
    quat_mul_vec(omega, &q[1], tmp1);
    quat_add_vec(tmp0, &tmp1[1], &tmp1[1]);
    quat_scale(dt*0.5f, tmp1, tmp1);
    quat_add(q, tmp1, tmp1);
    quat_normalize(tmp1, result);
}

// 現在の姿勢と目標姿勢から制御角速度を求める
// q_c: Quaternion 現在の姿勢(current)
// q_d: Quaternion 目標姿勢(destination)
// s: dtの変化率（0 < s）
// k: dtの最小値（0 < k）
// result: Vector3 制御角速度[rad/s]
void quat_get_control_anguler_velocity(float q_c[4], float q_d[4], float s, float k, float result[3]) {
    float dot = quat_dot(q_c, q_d);
    float dt;
    float q_dl[4];  // 関数外の値を書換えたくないのでローカルに確保
    float tmp0[4];
    float tmp1[3];
    float tmp2[3];
    float tmp3[3];
    
    if (dot < 0.0f) {
        dot = -dot;
        quat_negate(q_d, q_dl);
    } else {
        memcpy(q_dl, q_d, 4*4);
    }
    
    dt = s * dot + k;
    if (dot > 0.995f) {  // 姿勢変化が少ない時は近似式を使う
        quat_scale_vec(q_c[0], &q_dl[1], tmp1);
        quat_scale_vec(q_dl[0], &q_c[1], tmp2);
        quat_cross_vec(&q_dl[1], &q_c[1], tmp3);
        quat_sub_vec(tmp1, tmp2, tmp1);
        quat_sub_vec(tmp1, tmp3, tmp1);
        quat_scale_vec(2.0f/dt, tmp1, result);
    } else {
        quat_conj(q_c, tmp0);
        quat_mul(q_dl, tmp0, tmp0);
        quat_ln_versor(tmp0, tmp1);
        quat_scale_vec(2.0f/dt, tmp1, result);
    }
}

// 軸ベクトルとその周りの回転角[rad]から四元数を求める
// axis: Vector3
// angle: [rad]
// result: Quaternion
void quat_from_axis_angle(float axis[3], float angle, float result[4]) {
    float norm = quat_norm_vec(axis);
    float tmp;
    
    if (norm < EPSILON) {
        quat_identity(result);
    } else {
        tmp = copysignf( fmodf(angle, 2.0f * PI), angle );  // limit to (-2π, 2π)
        tmp = 0.5f * tmp;
        result[0] = cosf(tmp);
        quat_scale_vec( sinf(tmp) / norm, axis, &result[1] );
    }
}

// ベクトルaをbの位置まで回転させる四元数を求める．
// 0 <= t <= 1
// a, b: Vector3
// result: Quaternion
void quat_rotate_a_to_b(float a[3], float b[3], float t, float result[4]) {
    float tmp_a[3];
    float tmp_b[3];
    float axis[3];
    float angle;
    quat_normalize_vec(a, tmp_a);
    quat_normalize_vec(b, tmp_b);
    quat_cross_vec(tmp_a, tmp_b, axis);
    angle = acos_safe( quat_dot_vec(tmp_a, tmp_b) );
    quat_from_axis_angle(axis, angle * t, result);
}

// ------------------ 他の回転パラメータへの変換関数 -------------------- //

// 位置ベクトル回転(q v q*)を表す方向余弦行列をVersorに変換．
// 座標系回転(q* v q)を表す方向余弦行列を変換する場合には，
// let q = conj( from_dcm(dcm) );
// とする．
void quat_from_dcm(float m[3][3], float result[4]) {
    uint32_t index = 0;
    float tmp = 0.0f;
    float coef;
    float num_list[4] = {
         m[0][0] + m[1][1] + m[2][2],
         m[0][0] - m[1][1] - m[2][2],
        -m[0][0] + m[1][1] - m[2][2],
        -m[0][0] - m[1][1] + m[2][2]
    };

    // num_listの中から，最大値を取るインデックスを探す
    for (uint32_t i=0; i<4; i++) {
        if (num_list[i] > tmp) {
            tmp = num_list[i];
            index = i;
        }
    }

    // 最大値を係数として使ってVersorを計算
    tmp = sqrtf(num_list[index] + 1.0f);
    coef = 1.0f / (2.0f * tmp);
    switch (index) {
        case 0:
            result[0] = 0.5f * tmp;
            result[1] = (m[2][1] - m[1][2]) * coef;
            result[2] = (m[0][2] - m[2][0]) * coef;
            result[3] = (m[1][0] - m[0][1]) * coef;
            break;
        case 1: 
            result[1] = 0.5f * tmp;
            result[0] = (m[2][1] - m[1][2]) * coef;
            result[2] = (m[0][1] + m[1][0]) * coef;
            result[3] = (m[0][2] + m[2][0]) * coef;
            break;
        case 2:
            result[2] = 0.5f * tmp;
            result[0] = (m[0][2] - m[2][0]) * coef;
            result[1] = (m[0][1] + m[1][0]) * coef;
            result[3] = (m[1][2] + m[2][1]) * coef;
            break;
        case 3:
            result[3] = 0.5f * tmp;
            result[0] = (m[1][0] - m[0][1]) * coef;
            result[1] = (m[0][2] + m[2][0]) * coef;
            result[2] = (m[1][2] + m[2][1]) * coef;
            break;
        default:
            break;
    }
}

// 位置ベクトル回転(q v q*)を表すVersorを，方向余弦行列に変換．
// 座標系回転(q* v q)を表すVersorを変換する場合には，
// let dcm = to_dcm( conj(q) );
// とする．
void quat_to_dcm(float q[4], float result[3][3]) {
    // Compute these value only once.
    float q0_q0 = q[0] * q[0];
    float q0_q1 = q[0] * q[1];
    float q0_q2 = q[0] * q[2];
    float q0_q3 = q[0] * q[3];
    float q1_q2 = q[1] * q[2];
    float q1_q3 = q[1] * q[3];
    float q2_q3 = q[2] * q[3];

    result[0][0] = 2.0f * (q0_q0 + q[1]*q[1]) - 1.0f;
    result[0][1] = 2.0f * (q1_q2 - q0_q3);
    result[0][2] = 2.0f * (q1_q3 + q0_q2);
    result[1][0] = 2.0f * (q1_q2 + q0_q3);
    result[1][1] = 2.0f * (q0_q0 + q[2]*q[2]) - 1.0f;
    result[1][2] = 2.0f * (q2_q3 - q0_q1);
    result[2][0] = 2.0f * (q1_q3 - q0_q2);
    result[2][1] = 2.0f * (q2_q3 + q0_q1);
    result[2][2] = 2.0f * (q0_q0 + q[3]*q[3]) - 1.0f;
}