#pragma once

#include "esp_err.h"

#define EYE3 {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}

/* `initialize_attitude_filters` - initializes the parameters of the attitude
   filters, including the time step, `dt`, and the proportional and integral
   gains, `kp` and `ki`.

   #### Parameters:
    - `new_dt` - the time step to use in the attitude filters, in seconds,
    - `new_kp` - the proportional gain to use in the attitude filters,
    - `new_ki` - the integral gain to use in the attitude filters.
*/
esp_err_t initialize_attitude_filters(float new_dt, float new_kp, float new_ki);

/* `mahoney_filter` - a single iteration of a Mahoney filter, using only
   acceleration, `a`, and angular velocity, `w`, to estimate current attitude.

   Equivalent MATLAB code would be:
```matlab
    a = a / norm(a)
    down_prd = R.' * down

    w_prd = cross(a, down_prd)
    b = b - dt * ki * w_prd

    Rp = 0.5 * dt * (skew(w - b) + kp * skew(w_prd))
    R = R * (eye(3) + Rp) \ (eye(3) - Rp)
```

   #### Parameters:
    - `R` - current attitude estimate, as a rotation matrix,
    - `b` - current gyroscope bias estimate,
    - `down` - the down vector in the original reference frame,
    - `a` - current acceleration reading, as a 3D vector,
    - `w` - current angular velocity reading, as a 3D vector.

   #### Returns:
    - `R` is updated in place with the new attitude estimate,
    - `b` is updated in place with the new gyroscope bias estimate,
    - `ESP_ERR_INVALID_ARG` if any input parameter is NULL, or if the
      accelerometer reading is zero,
    - `ESP_OK` otherwise.
*/
esp_err_t mahoney_filter(
    float R[3][3], float b[3], float down[3], float a[3], float w[3]);

/* `mdm_filter` - a single iteration of an MDM filter, using only
   acceleration, `a`, and angular velocity, `w`, to estimate current attitude.

   Equivalent MATLAB code would be:
```matlab
    a = a / norm(a)
    down_prd = R.' * down
    w_prd = cross(a, down_prd)

    Rp = 0.5 * dt * (skew(w) + kp * skew(w_prd))
    R = R * (eye(3) + Rp) \ (eye(3) - Rp)
```

   #### Parameters:
    - `R` - current attitude estimate, as a rotation matrix,
    - `down` - the down vector in the original reference frame,
    - `a` - current acceleration reading, as a 3D vector, pre-filtered to
consist primarily of normal acceleration,
    - `w` - current angular velocity reading, as a 3D vector, pre-filtered to
remove constant bias.

   #### Returns:
    - `R` is updated in place with the new attitude estimate,
    - `ESP_ERR_INVALID_ARG` if any input parameter is NULL, or if the
      accelerometer reading is zero,
    - `ESP_OK` otherwise.
*/
esp_err_t mdm_filter(float R[3][3], float down[3], float a[3], float w[3]);

/* `get_current_direction` - gets the current direction of a vector in the
   reference frame defined by `R`.

   This is the inverse operation of `get_past_direction`.

   For example, if you want to get a down vector in the current   reference
   frame, you would call this function with `R` (which maps the original
   reference frame to the current reference frame), the original down vector,
   and a result vector to hold the current down vector.

   #### Parameters:
    - `R` - current attitude estimate, as a rotation matrix,
    - `dir` - the direction of interest in the original reference frame,
    - `result` - the direction of interest in the current reference frame.

   #### Returns:
    - `result` is updated in place with the current direction of interest,
    - `ESP_ERR_INVALID_ARG` if any input parameter is NULL,
    - `ESP_OK` otherwise.
*/
esp_err_t get_current_direction(float R[3][3], float dir[3], float result[3]);

/* `get_past_direction` - gets the direction of a vector in the original
   reference frame, given its direction in the current reference frame.

   This is the inverse operation of `get_current_direction`.

   #### Parameters:
    - `R` - current attitude estimate, as a rotation matrix,
    - `dir` - the direction of interest in the current reference frame,
    - `result` - the direction of interest in the original reference frame.

   #### Returns:
    - `result` is updated in place with the past direction of interest,
    - `ESP_ERR_INVALID_ARG` if any input parameter is NULL,
    - `ESP_OK` otherwise.
*/
esp_err_t get_past_direction(float R[3][3], float dir[3], float result[3]);
