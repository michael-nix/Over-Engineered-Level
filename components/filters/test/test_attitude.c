#include "matrix.h"
#include "mex.h"

#include <math.h>
#include <string.h>

#define double_EPSILON 5e-7f
#define EYE3           {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

static double dt = 1.0f;
static double kp = 1.0f;
static double ki = 1.0f;

void initialize_attitude_filters(double new_dt, double new_kp, double new_ki)
{
    dt = new_dt;
    kp = new_kp;
    ki = new_ki;
}

static inline void skew(double a[3], double result[3][3])
{
    result[0][0] = 0;
    result[0][1] = -a[2];
    result[0][2] = a[1];

    result[1][0] = a[2];
    result[1][1] = 0;
    result[1][2] = -a[0];

    result[2][0] = -a[1];
    result[2][1] = a[0];
    result[2][2] = 0;
}

// `result = a x b`
static inline void cross(double a[3], double b[3], double result[3])
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// `result = a.' * b`
static inline void dot(double a[3], double b[3], double* result)
{
    *result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// `result = A * B`
static inline void mat_mmult(
    double a[3][3], double b[3][3], double result[3][3])
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = 0;
            for (size_t k = 0; k < 3; k++)
            {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// `result = A + B`
static inline void mat_madd(double a[3][3], double b[3][3], double result[3][3])
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

// `result = A - B`
static inline void mat_msub(double a[3][3], double b[3][3], double result[3][3])
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

// `result = b * A`
static inline void mat_smult(double a[3][3], double b, double result[3][3])
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] * b;
        }
    }
}

// `result = A * b`
static inline void mat_vmult(double a[3][3], double b[3], double result[3])
{
    for (size_t i = 0; i < 3; i++)
    {
        result[i] = 0;
        for (size_t j = 0; j < 3; j++)
        {
            result[i] += a[i][j] * b[j];
        }
    }
}

// `result = A.'`
static inline void mat_trans(double a[3][3], double result[3][3])
{
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[j][i];
        }
    }
}

// `result = I \ A`
static inline void mat_inv(double a[3][3], double result[3][3])
{
    double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    double det_inv = 1.0f / det;

    result[0][0] = det_inv * (a[1][1] * a[2][2] - a[1][2] * a[2][1]);
    result[0][1] = det_inv * (a[0][2] * a[2][1] - a[0][1] * a[2][2]);
    result[0][2] = det_inv * (a[0][1] * a[1][2] - a[0][2] * a[1][1]);

    result[1][0] = det_inv * (a[1][2] * a[2][0] - a[1][0] * a[2][2]);
    result[1][1] = det_inv * (a[0][0] * a[2][2] - a[0][2] * a[2][0]);
    result[1][2] = det_inv * (a[0][2] * a[1][0] - a[0][0] * a[1][2]);

    result[2][0] = det_inv * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    result[2][1] = det_inv * (a[0][1] * a[2][0] - a[0][0] * a[2][1]);
    result[2][2] = det_inv * (a[0][0] * a[1][1] - a[0][1] * a[1][0]);
}

// `result = b * a`
static inline void vec_smult(double a[3], double b, double result[3])
{
    result[0] = a[0] * b;
    result[1] = a[1] * b;
    result[2] = a[2] * b;
}

// `result = a - b`
static inline void vec_vsub(double a[3], double b[3], double result[3])
{
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

void mahoney_filter(
    double R[3][3], double b[3], double down[3], double a[3], double w[3])
{
    double mag = 0.0f;
    dot(a, a, &mag);
    mag = 1.0f / sqrtf(mag);

    double a_norm[3] = {0};
    vec_smult(a, mag, a_norm);

    // down_prd = R.' * down
    double v_temp[3] = {0};
    double T1[3][3] = {0};
    mat_trans(R, T1);
    mat_vmult(T1, down, v_temp);

    // w_prd = cross(a, down_prd)
    double w_pred[3] = {0};
    cross(a_norm, v_temp, w_pred);

    // b = b - dt * ki * w_prd
    vec_smult(w_pred, dt * ki, v_temp);
    vec_vsub(b, v_temp, b);

    // Rp = 0.5 * dt * (skew(w - b) + kp * skew(w_prd))
    vec_smult(w_pred, kp, w_pred);
    skew(w_pred, T1);

    double wf[3] = {0};
    double T2[3][3] = {0};
    vec_vsub(w, b, wf);
    skew(wf, T2);

    mat_madd(T1, T2, T1);
    mat_smult(T1, 0.5f * dt, T1);

    // R = R * (eye(3) + Rp) \ (eye(3) - Rp)
    static double I3f[3][3] = EYE3;
    double T3[3][3] = {0};
    mat_madd(I3f, T1, T2);
    mat_msub(I3f, T1, T3);

    mat_inv(T3, T1);
    mat_mmult(T2, T1, T3);
    mat_mmult(R, T3, T1);

    memcpy(R, T1, sizeof(R[0]) * 3);
}

void mdm_filter(double R[3][3], double down[3], double a[3], double w[3])
{
    double mag = 0.0f;
    dot(a, a, &mag);
    mag = 1.0f / sqrtf(mag);

    double a_norm[3] = {0};
    vec_smult(a, mag, a_norm);

    // down_prd = R.' * down
    double v_temp[3] = {0};
    double T1[3][3] = {0};
    mat_trans(R, T1);
    mat_vmult(T1, down, v_temp);

    // w_prd = cross(a, down_prd)
    double w_pred[3] = {0};
    cross(a_norm, v_temp, w_pred);

    // Rp = 0.5 * dt * (ki * skew(w) + kp * skew(w_prd))
    vec_smult(w_pred, kp, w_pred);
    skew(w_pred, T1);

    double T2[3][3] = {0};
    vec_smult(w, ki, w);
    skew(w, T2);

    mat_madd(T1, T2, T1);
    mat_smult(T1, 0.5f * dt, T1);

    // R = R * (eye(3) + Rp) \ (eye(3) - Rp)
    static double I3f[3][3] = EYE3;
    double T3[3][3] = {0};
    mat_madd(I3f, T1, T2);
    mat_msub(I3f, T1, T3);

    mat_inv(T3, T1);
    mat_mmult(T2, T1, T3);
    mat_mmult(R, T3, T1);

    memcpy(R, T1, sizeof(R[0]) * 3);
}

void get_current_direction(double R[3][3], double dir[3], double result[3])
{
    double Rt[3][3] = {0};
    mat_trans(R, Rt);
    mat_vmult(Rt, dir, result);
}

void get_past_direction(double R[3][3], double dir[3], double result[3])
{
    mat_vmult(R, dir, result);
}
