// ----------------------------------------------------
// Quaternion calculate functions
// ----------------------------------------------------
// FPU搭載マイコン向けの四元数演算関数群
// 拡張子をcppに変更することでC++環境でも利用可能
// 引数に渡した変数は，関数内部で変更されない．
//
// Mbed Compiler向けに作成したので，所々mathライブラリの関数名が違う可能性あり．
// 例：abs(x) <---> fabs(x)
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
// float s = 2.0;
// float q[4] = {1.0f, 2.0f, 3.0f, 4.0f};
// float result[4];
// quat_scale(s, q, result);


#include "math.h"

const float PI = 3.1415926535f;
const float EPSILON = 1e-6;


float acos_safe(float x) {
    if ( abs(x) >= 1.0f ) {
        if (x > 0.0f) {
            return 0.0f;
        } else {
            return PI;
        }
    } else {
        return acosf(x);
    }
}

void quat_copy_vec(float *v, float *result) {
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}

// q の値をそのままresultにコピー
void quat_copy(float *q, float *result) {
    result[0] = q[0];
    quat_copy_vec(&q[1], &result[1]);
}

// resultにゼロベクトルを入れる
void quat_zero_vec(float *result) {
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
}

// resultに恒等四元数を入れる
void quat_identity(float *result) {
    result[0] = 1.0f;
    quat_zero_vec(&result[1]);
}

float quat_dot_vec(float *a, float *b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float quat_dot(float *a, float *b) {
    return a[0]*b[0] + quat_dot_vec(&a[1], &b[1]);
}

void quat_cross_vec(float *a, float *b, float *result) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void quat_add_vec(float *a, float *b, float *result) {
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];
}

void quat_add(float *a, float *b, float *result) {
    result[0] = a[0] + b[0];
    quat_add_vec(&a[1], &b[1], &result[1]);
}

void quat_sub_vec(float *a, float *b, float *result) {
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

void quat_sub(float *a, float *b, float *result) {
    result[0] = a[0] - b[0];
    quat_sub_vec(&a[1], &b[1], &result[1]);
}

void quat_scale_vec(float s, float *v, float *result) {
    result[0] = s * v[0];
    result[1] = s * v[1];
    result[2] = s * v[2];
}

void quat_scale(float s, float *q, float *result) {
    result[0] = s * q[0];
    quat_scale_vec(s, &q[1], &result[1]);
}

// result = s*a + b
void quat_scale_add_vec(float s, float *a, float *b, float *result) {
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
void quat_scale_add(float s, float *a, float *b, float *result) {
    // FMAが使える場合
    //result[0] = fma(s, a[0], b[0]);
    //quat_scale_add_vec(s, &a[1], &b[1], &result[1]);

   // FMAが使えない場合
    float tmp[4];
    quat_scale(s, a, tmp);
    quat_add(tmp, b, result);
}

void quat_conj(float *q, float *result) {
    result[0] = q[0];
    quat_scale_vec(-1.0f, &q[1], &result[1]);
}

float quat_norm_vec(float *v) {
    return sqrtf( quat_dot_vec(v, v) );
}

float quat_norm(float *q) {
    return sqrtf( quat_dot(q, q) );
}

// ベクトルを正規化
// 零ベクトルを入力した場合には零ベクトルを返す．
void quat_normalize_vec(float *v, float *result) {
    float norm = quat_norm_vec(v);
    if (norm < EPSILON) {
        quat_zero_vec(result);
    } else {
        quat_scale_vec(1.0f / norm, v, result);
    }
}

// qを正規化
void quat_normalize(float *q, float *result) {
    float inv_sqrt = 1.0f / quat_norm(q);
    quat_scale(inv_sqrt, q, result);
}

// 純虚四元数同士の積
// a: Vector3
// b: Vector3
// result: Quaternion
void quat_mul_vec(float *a, float *b, float *result) {
    result[0] = -quat_dot_vec(a, b);
    quat_cross_vec(a, b, &result[1]);
}

// ハミルトン積
// a: Quaternion
// b: Quaternion
// result: Quaternion
void quat_mul(float *a, float *b, float *result) {
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

// 対数函数
// 恒等四元数において特異点
// q: Quaternion
// result: Vector3
void quat_ln_versor(float *q, float *result) {
    float norm_vec = quat_norm_vec(&q[1]);
    if (norm_vec < EPSILON) {
        quat_identity(result);
    } else {
        float coef = acosf(q[0]) / norm_vec;
        quat_scale_vec(coef, &q[1], result);
    }
}

// 位置ベクトルの回転
// q: Quaternion
// v: Vector3
// result: Vector3
void quat_vector_rotation(float *q, float *v, float *result) {
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

// 座標系の回転
// q: Quaternion
// v: Vector3
// result: Vector3
void quat_frame_rotation(float *q, float *v, float *result) {
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
void quat_integration_approx(float *q, float *omega, float dt, float *result) {
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
void quat_get_control_anguler_velocity(float *q_c, float *q_d, float s, float k, float *result) {
    float dot = quat_dot(q_c, q_d);
    float dt;
    float q_dl[4];  // 関数外の値を書換えたくないのでローカルに確保
    float tmp0[4];
    float tmp1[3];
    float tmp2[3];
    float tmp3[3];
    
    if (dot < 0.0f) {
        dot = -1.0f * dot;
        quat_scale(-1.0f, q_d, q_dl);
    } else {
        quat_copy(q_d, q_dl);
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
void quat_from_axis_angle(float *axis, float angle, float *result) {
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
void quat_rotate_a_to_b(float *a, float *b, float *result) {
    float tmp_a[3];
    float tmp_b[3];
    float axis[3];
    float angle;
    quat_normalize_vec(a, tmp_a);
    quat_normalize_vec(b, tmp_b);
    quat_cross_vec(tmp_a, tmp_b, axis);
    angle = acos_safe( quat_dot_vec(tmp_a, tmp_b) );
    quat_from_axis_angle(axis, angle, result);
}