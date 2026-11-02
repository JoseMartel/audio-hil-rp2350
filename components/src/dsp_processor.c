#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "protocol.h"
#include "dsp_processor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static arm_rfft_fast_instance_f32 rfft_inst;
static float* window_coeffs = NULL;
static float* fft_input = NULL;
static float* fft_output = NULL;
static float* magnitudes = NULL;
static size_t dsp_fft_size = 0;
static uint32_t dsp_sample_rate = 0;
static uint32_t k_min = 0;
static uint32_t k_max = 0;

static float current_noise_threshold = 2.0f;
static float current_min_freq = 50.0f;
static float current_max_freq = 3000.0f;

int dsp_processor_init(size_t fft_size, uint32_t sample_rate) {
    dsp_fft_size = fft_size;
    dsp_sample_rate = sample_rate;

    // Initialize CMSIS-DSP Real FFT
    arm_status status = arm_rfft_fast_init_f32(&rfft_inst, (uint16_t)fft_size);
    if (status != ARM_MATH_SUCCESS) {
        return -1;
    }

    // Allocate memory buffers
    window_coeffs = malloc(fft_size * sizeof(float));
    fft_input = malloc(fft_size * sizeof(float));
    fft_output = malloc(fft_size * sizeof(float));
    magnitudes = malloc((fft_size / 2) * sizeof(float));

    if (!window_coeffs || !fft_input || !fft_output || !magnitudes) {
        dsp_processor_deinit();
        return -2;
    }

    // Precompute Hann window: w[n] = 0.5 * (1 - cos(2 * pi * n / (N - 1)))
    for (size_t n = 0; n < fft_size; n++) {
        window_coeffs[n] = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)n / (float)(fft_size - 1)));
    }

    dsp_processor_set_thresholds(current_noise_threshold, current_min_freq, current_max_freq);

    return 0;
}

int dsp_processor_set_thresholds(float noise_threshold, float min_freq, float max_freq) {
    if (dsp_fft_size == 0 || dsp_sample_rate == 0) return -1;
    if (min_freq >= max_freq) return -2;

    current_noise_threshold = noise_threshold;
    current_min_freq = min_freq;
    current_max_freq = max_freq;

    float bin_resolution = (float)dsp_sample_rate / (float)dsp_fft_size;
    k_min = (uint32_t)(current_min_freq / bin_resolution);
    k_max = (uint32_t)(current_max_freq / bin_resolution);

    if (k_min < 1) k_min = 1;
    if (k_max >= dsp_fft_size / 2) k_max = dsp_fft_size / 2 - 1;

    return 0;
}

void dsp_processor_get_thresholds(float* out_noise_threshold, float* out_min_freq, float* out_max_freq) {
    if (out_noise_threshold) *out_noise_threshold = current_noise_threshold;
    if (out_min_freq) *out_min_freq = current_min_freq;
    if (out_max_freq) *out_max_freq = current_max_freq;
}

float dsp_processor_process_ext(const float* input_samples, 
                                float* out_max_mag, 
                                uint32_t* out_peak_bin, 
                                uint8_t* out_status) {
    if (!window_coeffs || !fft_input || !fft_output || !magnitudes) {
        if (out_status) *out_status = HIL_STATUS_SILENCE;
        return 0.0f;
    }

    // 1. Apply Hann window
    arm_mult_f32(input_samples, window_coeffs, fft_input, dsp_fft_size);

    // 2. Perform Real FFT (direction: 0 for forward)
    arm_rfft_fast_f32(&rfft_inst, fft_input, fft_output, 0);

    // 3. Compute magnitude spectrum
    // fft_output format:
    // fft_output[0] = Real(DC), fft_output[1] = Real(Nyquist)
    // fft_output[2*i] = Real(bin i), fft_output[2*i+1] = Imag(bin i)
    magnitudes[0] = fabsf(fft_output[0]);
    arm_cmplx_mag_f32(fft_output + 2, magnitudes + 1, dsp_fft_size / 2 - 1);

    // 4. Find peak magnitude within the search range [k_min, k_max]
    float search_max_val = 0.0f;
    uint32_t search_peak_idx = 0;
    uint32_t search_len = k_max - k_min + 1;

    arm_max_f32(magnitudes + k_min, search_len, &search_max_val, &search_peak_idx);

    uint32_t peak_bin = k_min + search_peak_idx;
    float max_val = search_max_val;

    if (out_max_mag) *out_max_mag = max_val;
    if (out_peak_bin) *out_peak_bin = peak_bin;

    // Threshold check: ignore small magnitudes below noise floor
    if (max_val < current_noise_threshold) {
        if (out_status) *out_status = HIL_STATUS_SILENCE;
        return 0.0f;
    }

    // Boundary check: A valid fundamental frequency peak inside [k_min, k_max] must be a true local maximum.
    // If peak_bin is at the boundary and the adjacent out-of-band neighbor is higher, the actual tone
    // is outside the valid detection band (e.g. < 50 Hz or > 3000 Hz) and is merely leaking in.
    if (peak_bin == k_min && k_min > 0 && magnitudes[k_min - 1] >= max_val) {
        if (out_status) *out_status = HIL_STATUS_OUT_OF_BOUNDS;
        return 0.0f;
    }
    if (peak_bin == k_max && (k_max + 1) < (dsp_fft_size / 2) && magnitudes[k_max + 1] >= max_val) {
        if (out_status) *out_status = HIL_STATUS_OUT_OF_BOUNDS;
        return 0.0f;
    }

    // 5. Quadratic interpolation around peak bin for sub-bin resolution
    float d = 0.0f;
    if (peak_bin > k_min && peak_bin < k_max) {
        float y1 = magnitudes[peak_bin - 1];
        float y2 = magnitudes[peak_bin];
        float y3 = magnitudes[peak_bin + 1];

        float denom = y1 - 2.0f * y2 + y3;
        if (fabsf(denom) > 1e-5f) {
            d = 0.5f * (y1 - y3) / denom;
        }
    }

    float bin_resolution = (float)dsp_sample_rate / (float)dsp_fft_size;
    float est_freq = ((float)peak_bin + d) * bin_resolution;

    // Final boundary check on the interpolated continuous frequency
    if (est_freq < (current_min_freq - 2.0f) || est_freq > (current_max_freq + 2.0f)) {
        if (out_status) *out_status = HIL_STATUS_OUT_OF_BOUNDS;
        return 0.0f;
    }

    if (out_status) *out_status = HIL_STATUS_OK;
    return est_freq;
}

float dsp_processor_process(const float* input_samples) {
    return dsp_processor_process_ext(input_samples, NULL, NULL, NULL);
}

void dsp_processor_deinit(void) {
    if (window_coeffs) {
        free(window_coeffs);
        window_coeffs = NULL;
    }
    if (fft_input) {
        free(fft_input);
        fft_input = NULL;
    }
    if (fft_output) {
        free(fft_output);
        fft_output = NULL;
    }
    if (magnitudes) {
        free(magnitudes);
        magnitudes = NULL;
    }
}
