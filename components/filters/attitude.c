#include "esp_log.h"

#include "attitude.h"
#include <math.h>

#define FLOAT_EPSILON 5e-7f

static float dt = 1.0f;
static float kp = 1.0f;
static float ki = 1.0f;

esp_err_t initialize_attitude_filters(float new_dt, float new_kp, float new_ki)
{
    dt = new_dt;
    kp = new_kp;
    ki = new_ki;

    return ESP_OK;
}

static inline esp_err_t skew(float a[3], float result[3][3])
{
    if (NULL == a || NULL == result)
    {
        ESP_LOGE(__func__,
            "Can't compute skew, at least one input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    result[0][0] = 0;
    result[0][1] = -a[2];
    result[0][2] = a[1];

    result[1][0] = a[2];
    result[1][1] = 0;
    result[1][2] = -a[0];

    result[2][0] = -a[1];
    result[2][1] = a[0];
    result[2][2] = 0;

    return ESP_OK;
}

// `result = a x b`
static inline esp_err_t cross(float a[3], float b[3], float result[3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute cross product, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    if (a == result || b == result)
    {
        ESP_LOGE(__func__,
            "Input parameters cannot be the same, cannot compute cross "
            "product in place!");

        return ESP_ERR_INVALID_ARG;
    }

    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];

    return ESP_OK;
}

// `result = a.' * b`
static inline esp_err_t dot(float a[3], float b[3], float* result)
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute dot product, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    *result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    return ESP_OK;
}

// `result = A * B`
static inline esp_err_t mat_mmult(
    float a[3][3], float b[3][3], float result[3][3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix multiplication, one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    if (a == result || b == result)
    {
        ESP_LOGE(__func__,
            "Input parameters cannot be the same, cannot compute "
            "matrix multiplication in place!");

        return ESP_ERR_INVALID_ARG;
    }

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

    return ESP_OK;
}

// `result = A + B`
static inline esp_err_t mat_madd(
    float a[3][3], float b[3][3], float result[3][3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix addition, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] + b[i][j];
        }
    }

    return ESP_OK;
}

// `result = A - B`
static inline esp_err_t mat_msub(
    float a[3][3], float b[3][3], float result[3][3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix subtraction, at least one "
                           "input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] - b[i][j];
        }
    }

    return ESP_OK;
}

// `result = b * A`
static inline esp_err_t mat_smult(float a[3][3], float b, float result[3][3])
{
    if (NULL == a || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix scalar multiplication, one "
                           "input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[i][j] * b;
        }
    }

    return ESP_OK;
}

// `result = A * b`
static inline esp_err_t mat_vmult(float a[3][3], float b[3], float result[3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__,
            "Can't compute matrix-vector multiplication, at least "
            "one input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    if (b == result)
    {
        ESP_LOGE(__func__,
            "Input parameters cannot be the same, cannot compute "
            "matrix-vector multiplication in place!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 3; i++)
    {
        result[i] = 0;
        for (size_t j = 0; j < 3; j++)
        {
            result[i] += a[i][j] * b[j];
        }
    }

    return ESP_OK;
}

// `result = A.'`
static inline esp_err_t mat_trans(float a[3][3], float result[3][3])
{
    if (NULL == a || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix transpose, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    if (a == result)
    {
        ESP_LOGE(__func__,
            "Input parameters cannot be the same, cannot compute "
            "matrix transpose in place!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            result[i][j] = a[j][i];
        }
    }

    return ESP_OK;
}

// `result = I \ A`
static inline esp_err_t mat_inv(float a[3][3], float result[3][3])
{
    if (NULL == a || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute matrix inverse, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    if (a == result)
    {
        ESP_LOGE(__func__, "Input parameters cannot be the same, cannot "
                           "compute matrix inverse in place!");

        return ESP_ERR_INVALID_ARG;
    }

    float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (fabsf(det) < FLOAT_EPSILON)
    {
        ESP_LOGE(__func__, "Can't compute matrix inverse, matrix is singular!");

        return ESP_ERR_INVALID_STATE;
    }

    float det_inv = 1.0f / det;

    result[0][0] = det_inv * (a[1][1] * a[2][2] - a[1][2] * a[2][1]);
    result[0][1] = det_inv * (a[0][2] * a[2][1] - a[0][1] * a[2][2]);
    result[0][2] = det_inv * (a[0][1] * a[1][2] - a[0][2] * a[1][1]);

    result[1][0] = det_inv * (a[1][2] * a[2][0] - a[1][0] * a[2][2]);
    result[1][1] = det_inv * (a[0][0] * a[2][2] - a[0][2] * a[2][0]);
    result[1][2] = det_inv * (a[0][2] * a[1][0] - a[0][0] * a[1][2]);

    result[2][0] = det_inv * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    result[2][1] = det_inv * (a[0][1] * a[2][0] - a[0][0] * a[2][1]);
    result[2][2] = det_inv * (a[0][0] * a[1][1] - a[0][1] * a[1][0]);

    return ESP_OK;
}

// `result = b * a`
static inline esp_err_t vec_smult(float a[3], float b, float result[3])
{
    if (NULL == a || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute vector scalar multiplication, at "
                           "least one input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    result[0] = a[0] * b;
    result[1] = a[1] * b;
    result[2] = a[2] * b;

    return ESP_OK;
}

// `result = a - b`
static inline esp_err_t vec_vsub(float a[3], float b[3], float result[3])
{
    if (NULL == a || NULL == b || NULL == result)
    {
        ESP_LOGE(__func__, "Can't compute vector subtraction, at least one "
                           "input parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];

    return ESP_OK;
}

esp_err_t mahoney_filter(
    float R[3][3], float b[3], float down[3], float a[3], float w[3])
{
    if (NULL == R || NULL == b || NULL == down || NULL == a || NULL == w)
    {
        ESP_LOGE(__func__, "Can't estimate attitude, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    float mag = 0.0f;
    dot(a, a, &mag);
    if (0.0 == mag)
    {
        ESP_LOGE(__func__, "Accelerometer reading is zero!");

        return ESP_ERR_INVALID_ARG;
    }
    mag = 1.0f / sqrtf(mag);

    float a_norm[3] = {0};
    vec_smult(a, mag, a_norm);

    // down_prd = R.' * down
    float v_temp[3] = {0};
    float T1[3][3] = {0};
    mat_trans(R, T1);
    mat_vmult(T1, down, v_temp);

    // w_prd = cross(a, down_prd)
    float w_pred[3] = {0};
    cross(a_norm, v_temp, w_pred);

    // b = b - dt * ki * w_prd
    vec_smult(w_pred, dt * ki, v_temp);
    vec_vsub(b, v_temp, b);

    // Rp = 0.5 * dt * (skew(w - b) + kp * skew(w_prd))
    vec_smult(w_pred, kp, w_pred);
    skew(w_pred, T1);

    float wf[3] = {0};
    float T2[3][3] = {0};
    vec_vsub(w, b, wf);
    skew(wf, T2);

    // R = R * (eye(3) + Rp) \ (eye(3) - Rp)
    mat_madd(T1, T2, T1);
    mat_smult(T1, 0.5f * dt, T1);

    static float I3f[3][3] = EYE3;
    float T3[3][3] = {0};
    mat_madd(I3f, T1, T2);
    mat_msub(I3f, T1, T3);

    mat_inv(T3, T1);
    mat_mmult(T2, T1, T3);
    mat_mmult(R, T3, T1);

    memcpy(R, T1, sizeof(R[0]) * 3);

    return ESP_OK;
}

esp_err_t mdm_filter(float R[3][3], float down[3], float a[3], float w[3])
{
    if (NULL == R || NULL == down || NULL == a || NULL == w)
    {
        ESP_LOGE(__func__, "Can't estimate attitude, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    float mag = 0.0f;
    dot(a, a, &mag);
    if (0.0 == mag)
    {
        ESP_LOGE(__func__, "Accelerometer reading is zero!");

        return ESP_ERR_INVALID_ARG;
    }
    mag = 1.0f / sqrtf(mag);

    float a_norm[3] = {0};
    vec_smult(a, mag, a_norm);

    // down_prd = R.' * down
    float v_temp[3] = {0};
    float T1[3][3] = {0};
    mat_trans(R, T1);
    mat_vmult(T1, down, v_temp);

    // w_prd = cross(a, down_prd)
    float w_pred[3] = {0};
    cross(a_norm, v_temp, w_pred);

    // Rp = 0.5 * dt * (skew(w) + kp * skew(w_prd))
    vec_smult(w_pred, kp, w_pred);
    skew(w_pred, T1);

    float T2[3][3] = {0};
    skew(w, T2);

    // R = R * (eye(3) + Rp) \ (eye(3) - Rp)
    mat_madd(T1, T2, T1);
    mat_smult(T1, 0.5f * dt, T1);

    static float I3f[3][3] = EYE3;
    float T3[3][3] = {0};
    mat_madd(I3f, T1, T2);
    mat_msub(I3f, T1, T3);

    mat_inv(T3, T1);
    mat_mmult(T2, T1, T3);
    mat_mmult(R, T3, T1);

    memcpy(R, T1, sizeof(R[0]) * 3);

    return ESP_OK;
}

esp_err_t get_current_direction(float R[3][3], float dir[3], float result[3])
{
    if (NULL == R || NULL == dir || NULL == result)
    {
        ESP_LOGE(__func__, "Can't get current direction, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    float Rt[3][3] = {0};
    mat_trans(R, Rt);
    mat_vmult(Rt, dir, result);

    return ESP_OK;
}
