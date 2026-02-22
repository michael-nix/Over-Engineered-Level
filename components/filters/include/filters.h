#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define FILTER_GROUP_DELAY_SAMPLES 250.0f // 4th order Butterworth, 2 Hz cutoff
#define FILTER_ORDER               4      // must be multiple of 2
#define FILTER_CUTOFF_FREQUENCY_HZ 2.0f
#define FILTER_NUM_SOS_SECTIONS    (FILTER_ORDER / 2)

/* `IntervalBuffer` - A type of ring buffer when you need to batch process data
  collected over the course of specified intervals, and compare them over time.

   Useful when you receive and need to process a constant amount of data in a
  given timeframe; e.g., in semi-realtime applications when you capture 20
  samples of data in a ten millisecond interval, and then process the last two
  hundred and fifty milliseconds of data while retaining most of the data.
  Slightly easier to amortize buffer operations over bundles of data, and
   slightly easier to access specific intervals, or access data within a
  specific interval.

  NOTE: if you need data from `x` total samples in the past, the buffer must be
  initialized with enough capacity to hold that much data, plus at least one
  extra interval.  That is:

    `capacity = ceil(x / nsamples) + 1`

    #### Struct Elements:
     - `nintervals` - the number of intervals in your table,
     - `nsamples` - the number of samples in your table,
     - `capacity` - the total capacity of your table, `nintervals` * `nsamples`,
     - `start` - the index of the first piece of data placed in the table,
     - `end` - one more than the index of the last item in your table,
     - `data` - a pointer to the underlying data.

    #### See also:
     - `initialize_interval_buffer`
     - `add_item_to_buffer`
     - `add_interval_to_buffer`
     - `get_current_interval_index`
     - `get_delayed_previous_interval_index`
     - `pop_interval_from_buffer_with_delay`
*/
typedef struct interval_buffer
{
    size_t nintervals; // total number of intervals in the buffer
    size_t nsamples;   // total number of samples in an interval
    size_t nbytes;     // number of bytes in a single sample
    size_t capacity;   // total number of elements; nintervals * nsamples
    size_t start;      // beginning index of filled data;
    size_t end;        // final index of filled data;

    void* data; // pointer to the data
    bool ready; // buffer full of good data; ready for processing;
} IntervalBuffer;

/* `SecondOrderSection` - a digital biquad filter that can be cascaded to
   perform any even order filter operation, whre `alpha` are the denominator
   coefficients, and `beta` are the numerator coefficients of the filter.

   #### See Also:
    - `SOSData` - a container for input and output data to be used when
   filtering using second order sections,
    - `get_butter_sos` - returns a Butterworth filter of even order in terms of
   second order sections,
    - `filter_sos` - filters your data using `SecondOrderSections` and
   `SOSData`.
*/
typedef struct sos
{
    float alpha[2];
    float beta[3];
} SecondOrderSection;

/* `SOSData` - a container that holds filter inputs and outputs for use with
   `SecondOrderSection`s when filtering data.
*/
typedef struct sos_data
{
    float outputs[2];
    float inputs[3];
} SOSData;

/* `initialize_interval_buffer` - sets default values and initializes memory for
   an `IntervalBuffer`.

   Inintializes an `IntervalBuffer` given the number of samples in an interval
   of data, whose total capacity will be given by `nintervals * nsamples`.  The
   `data` initialized will be continuous along each interval; i.e. incrementing
   a single `data` index is the same as incrementing a sample index while
   keeping a interval index constant, unless you're at the end of a interval.
   Each element of the `data` will have a size of `nbytes`.

   NOTE: if you need data from `x` total samples in the past, the buffer must be
   initialized with enough capacity to hold that much data, plus at least one
   extra interval.  That is:

    `capacity = ceil(x / nsamples) + 1`

   #### Parameters:
     - `nintervals` - the number of intervals in the table,
     - `nsamples` - the number of samples in the table,
     - `nbytes` - the size, in bytes, of each element of the table.

   #### Returns:
     - Initialized interval buffer.
*/
IntervalBuffer initialize_interval_buffer(
    size_t nintervals, size_t nsamples, size_t nbytes);

/* `pop_interval_from_buffer_with_delay` - removes a single interval from an
   `IntervalBuffer`, accounting for wrap-around.

   Places `nsamples` elements of data from an `IntervalBuffer` into an output
   array, `out`. Advances the starting index of the `IntervalBuffer`, removing
   the interval from the buffer.

   #### Parameters:
    - `buffer` - pointer to the `IntervalBuffer` to read data from,
    - `out` - pointer to the `uint8_t` array you'd like to copy data in to,
    - `delay` - the number of data points you'd like to use from the interval
   previous to the last full interval.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if any of the input pointers are NULL,
    - `ESP_OK` otherwise.
*/
esp_err_t pop_interval_from_buffer_with_delay(
    IntervalBuffer* buffer, uint8_t* out, size_t delay);

/* `get_current_interval_from_buffer` - copies the most recent data placed in
   the `IntervalBuffer` into a new buffer, `out`.  Does not update the `stop` or
   `start` indices of the `IntervalBuffer`.

   #### Parameters:
    - `buffer` - a poiner to the `IntervalBuffer` to copy data from,
    - `out` - a pointer to the output buffer where you want to copy data to.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if any of the input pointers are NULL,
    - `ESP_OK` otherwise.
*/
esp_err_t get_current_interval_from_buffer(
    IntervalBuffer* buffer, uint8_t* out);

/* `add_item_to_buffer` - adds a single item to an `IntervalBuffer`.

   If the `buffer` is full after adding an item, sets the `ready` flag to true.

   #### Parameters:
    - `buffer` - a pointer to an initialized `IntervalBuffer`; also defines the
   size of `data`,
    - `data` - a pointer to a single item of the right size for the `buffer`.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` when `buffer` is NULL, `data` is NULL, or the
   `buffer` end has overrun its capacity,
    - `ESP_OK` when the item is successfully added.
*/
esp_err_t add_item_to_buffer(IntervalBuffer* buffer, uint8_t* data);

/* `add_interval_to_buffer` - add an entire interval to an `IntervalBuffer` at
   once.

   Copies `buffer->nbytes * buffer->nsamples` worth of `data` into `buffer`.
   Updates the endpoint of the `buffer`, and sets the `ready` flag to true if
   the `buffer` is full and ready to be processed.

   #### Parameters:
    - `buffer` - a pointer to an `IntervalBuffer` that will store the interval,
    - `data` - a pointer to the data you want to copy to the `buffer`.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` when `buffer` is NULL, `data` is NULL, or the
   `buffer`'s current interval is not at a proper interval boundary,
    - `ESP_OK` when the interval is successfully added.
*/
esp_err_t add_interval_to_buffer(IntervalBuffer* buffer, uint8_t* data);

/* `get_butter_sos` - returns a Butterworth filter in terms of
   `SecondOrderSection`s for a given cutoff frequency, `fc`, sample rate, `fs`,
   and order, `order`.

   The number of second order sections initialized will be `order / 2`.  This
   function does not allocate memory for `SecondOrderSection`s, so a poiner to
   `order / 2` will need to be passed to this function.  If your filter `order`
   is not an even number, you will get a filter with an order of one less than
   requested.

   #### Parameters:
    - `fc` - the cutoff frequency of your filter in Hz,
    - `fs` - the sample rate of your data in Hz,
    - `order` - the order of your filter; should be an even number,
    - `sos` - a pointer to the `order / 2` `SecondOrderSections` that will hold
   this filter.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` when the order is zero or `sos` is NULL,
    - `ESP_OK` if the filter is successfully created.

   #### See Also:
    - `filter_sos` - filter data using a single `SecondOrderSection`.
*/
esp_err_t get_butter_sos(
    float fc, float fs, uint8_t order, SecondOrderSection* sos);

/* `filter_sos` - filters `input` and past `data` using a single
   `SecondOrderSection`.

   Direct form 1 implementation of a single digital biquad filter.  If you have
   a filter implemented as a series of `SecondOrderSections`, you loop through
   each of them, filter using `filter_sos`, though that only needs a single
   instance of `SOSData`.

   For example:
   ```
       float input = // get input data

       SecondOrderSection sos[ORDER / 2] = {0};
       get_butter_sos(CUTOFF, SAMPLE, ORDER, sos);

       SOSData data = // initialize data
       for (size_t idx = 0; idx < ORDER / 2; idx++)
       {
           input = filter_sos(sos, &data. input);
       }
   ```

   #### Parameters:
    - `sos` - a pointer to a single `SecondOrderSection` that defines this piece
   of the filter,
    - `data` - a pointer to a single `SOSData` container that holds all filter
   data for your filter,
    - `input` - a single float value that is the latest data you are filtering.

   #### Returns:
    - `NAN` when `data` or `sos` are NULL,
    - the output of the filter otherwise.
*/
float filter_sos(SecondOrderSection* sos, SOSData* data, float input);

/* `initialize_sos_data` - initializes an array of `SOSData` containers with
   initial data.

   This isuseful when you want to initialize your filter with a steady state
   value, so that you don't have to wait for the filter to settle.

   NOTE: set up for low pass filters only.

   #### Parameters:
    - `sos_data` - a pointer to an array of `SOSData` containers to be
   initialized,
    - `initial_data` - a pointer to an array of floats that will be used to
   initialize the `SOSData`.  Each element of this array will be used to
   initialize the inputs of a single `SOSData` container; i.e. the first three
   elements will be used to initialize the first `SOSData`, the next three will
   be used to initialize the second `SOSData`, and so on.
    - `nsos` - the number of `SecondOrderSection`s in your filter; i.e. the
   number of `SOSData` containers you need to initialize.

   #### Returns:
    - `ESP_ERR_INVALID_ARG` if any input pointer is NULL,
    - `ESP_OK` if the data is successfully initialized.
*/
esp_err_t initialize_sos_data(
    SOSData* sos_data, float* initial_data, size_t nsos);
