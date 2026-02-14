#include "esp_log.h"

#include "filters.h"

#include <math.h>
#include <string.h>

#define PI 3.141592653589793f

/* `get_current_interval_index` - returns the index at the start of the last
   interval added to the `IntervalBuffer`.

   If you want to analyze data that was just placed into the table, this will
   return the correct index for the beginning of the interval you would like to
   analyze.

   #### Parameters:
     - `buffer` - a pointer to the `IntervalBuffer` you're working with.

   #### Returns:
     - `size_t` - the correct index of the `IntervalBuffer`'s `data`.
 */
static size_t get_current_interval_index(IntervalBuffer const* buffer)
{
    return (buffer->start + buffer->capacity - buffer->nsamples) %
           buffer->capacity;
}

/* `get_delayed_previous_interval_index` - returns an index partway into the
   interval previous to the last full interval in the `IntervalBuffer`.

   While `get_current_interval_buffer_interval_index` returns the index of the
   last full interval in a `IntervalBuffer`, this will return an index that
   points to an additional `delay`'s worth of data in the interval previous to
   that.

   #### Parameters:
     - `buffer` - a pointer to the `IntervalBuffer` you're working with,
     - `delay` - the number of data points you'd like to use from the interval
   previous to the last full interval.
 */
static size_t get_delayed_previous_interval_index(
    IntervalBuffer const* buffer, size_t delay)
{
    return (buffer->start + buffer->capacity - buffer->nsamples - delay) %
           buffer->capacity;
}

esp_err_t get_current_interval_from_buffer(IntervalBuffer* buffer, uint8_t* out)
{
    if ((NULL == buffer) || (NULL == out))
    {
        ESP_LOGE(__func__, "Pointer to input buffer or data is NULL");

        return ESP_ERR_INVALID_ARG;
    }

    size_t idx = get_current_interval_index(buffer);
    size_t start = idx * buffer->nbytes;

    uint8_t* in = ((uint8_t*)buffer->data) + start;
    memcpy(out, in, buffer->nsamples * buffer->nbytes);

    return ESP_OK;
}

esp_err_t pop_interval_from_buffer_with_delay(
    IntervalBuffer* buffer, uint8_t* out, size_t delay)
{
    if ((NULL == buffer) || (NULL == out))
    {
        ESP_LOGE(__func__, "Pointer to input buffer or data is NULL");

        return ESP_ERR_INVALID_ARG;
    }

    size_t index = get_delayed_previous_interval_index(buffer, delay);
    buffer->start = (buffer->start + buffer->nsamples) % buffer->capacity;
    buffer->ready = false;

    uint8_t* tdata = buffer->data;
    uint8_t* first = &tdata[index * buffer->nbytes];

    size_t stop = buffer->capacity - index;
    stop = stop >= buffer->nsamples ? buffer->nsamples : stop;

    size_t nbytes = stop * buffer->nbytes;
    memcpy(out, first, nbytes);

    if (stop == buffer->nsamples)
        return ESP_OK;

    uint8_t* second = &tdata[0];
    out = &out[nbytes];

    stop = buffer->nsamples - stop;
    nbytes = stop * buffer->nbytes;
    memcpy(out, second, nbytes);

    return ESP_OK;
}

IntervalBuffer initialize_interval_buffer(
    size_t nintervals, size_t nsamples, size_t nbytes)
{
    IntervalBuffer t = {0};
    t.nintervals = nintervals;
    t.nsamples = nsamples;
    t.nbytes = nbytes;
    t.capacity = nintervals * nsamples;
    t.data = calloc(t.capacity, nbytes);
    t.ready = false;

    return t;
}

esp_err_t add_item_to_buffer(IntervalBuffer* buffer, uint8_t* data)
{
    if ((NULL == buffer) || (NULL == data))
    {
        ESP_LOGE(__func__, "Pointer to input buffer or data is NULL");

        return ESP_ERR_INVALID_ARG;
    }

    if (buffer->end >= buffer->capacity)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t end_idx = buffer->end * buffer->nbytes;
    uint8_t* udata = (uint8_t*)buffer->data;
    uint8_t* buffer_end = &udata[end_idx];
    memcpy(buffer_end, data, buffer->nbytes);

    buffer->end = (buffer->end + 1) % buffer->capacity;

    if (buffer->start == buffer->end)
    {
        buffer->ready = true;
    }
    else
    {
        buffer->ready = false;
    }

    return ESP_OK;
}

esp_err_t add_interval_to_buffer(IntervalBuffer* buffer, uint8_t* data)
{
    if ((NULL == buffer) || (NULL == data))
    {
        ESP_LOGE(__func__, "Pointer to input buffer or data is NULL");

        return ESP_ERR_INVALID_ARG;
    }

    if ((buffer->end % buffer->nsamples) != 0)
    {
        ESP_LOGE(__func__, "Can't add row to IntervalBuffer, buffer end must "
                           "be the end of an interval.");

        return ESP_ERR_INVALID_ARG;
    }

    size_t end_idx = buffer->end * buffer->nbytes;
    uint8_t* udata = (uint8_t*)buffer->data;
    uint8_t* buffer_end = &udata[end_idx];
    size_t nbytes = buffer->nbytes * buffer->nsamples;
    memcpy(buffer_end, data, nbytes);

    buffer->end = (buffer->end + buffer->nsamples) % buffer->capacity;

    if (buffer->start == buffer->end)
    {
        buffer->ready = true;
    }
    else
    {
        buffer->ready = false;
    }

    return ESP_OK;
}

esp_err_t initialize_sos_data(
    SOSData* sos_data, float* initial_data, size_t nsos)
{
    if (NULL == sos_data || NULL == initial_data)
    {
        ESP_LOGE(__func__, "Can't initialize SOS data, at least one input "
                           "parameter is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    for (size_t idx = 0; idx < nsos; idx++)
    {
        sos_data[idx].inputs[0] = initial_data[idx];
        sos_data[idx].inputs[1] = initial_data[idx];
        sos_data[idx].inputs[2] = initial_data[idx];
    }

    return ESP_OK;
}

esp_err_t get_butter_sos(
    float fc, float fs, uint8_t order, SecondOrderSection* sos)
{
    if ((0 == order) || (NULL == sos))
    {
        ESP_LOGE(
            __func__, "Either filter order is zero, or SOS pointer is NULL!");

        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t m = 1; m <= order / 2; m++)
    {
        float pr = cos((2 * m + order - 1) * PI / 2.0f / order);

        float wc = 2.0f * PI * fc / fs;
        wc = 2.0f * fs * tan(wc / fs / 2.0f);
        float w0 = wc / 2.0f;
        float w02 = w0 * w0;

        float a = 1.0f - wc * pr + w02;
        float bc = -2.0f * (1.0f - w02);
        float d = 1.0f + wc * pr + w02;

        bc = bc / a;
        d = d / a;
        w02 = w02 / a;

        sos[m - 1].alpha[0] = bc;
        sos[m - 1].alpha[1] = d;
        sos[m - 1].beta[0] = w02;
        sos[m - 1].beta[1] = 2.0f * w02;
        sos[m - 1].beta[2] = w02;
    }

    return ESP_OK;
}

float filter_sos(SecondOrderSection* sos, SOSData* data, float input)
{
    if ((NULL == data) || (NULL == sos))
    {
        ESP_LOGE(__func__, "Either second order section or its data is NULL!");

        return NAN;
    }

    data->inputs[2] = data->inputs[1];
    data->inputs[1] = data->inputs[0];
    data->inputs[0] = input;

    float output = 0;
    output += data->inputs[2] * sos->beta[2];
    output += data->inputs[1] * sos->beta[1];
    output += data->inputs[0] * sos->beta[0];
    output -= data->outputs[1] * sos->alpha[1];
    output -= data->outputs[0] * sos->alpha[0];

    data->outputs[1] = data->outputs[0];
    data->outputs[0] = output;

    return output;
}
