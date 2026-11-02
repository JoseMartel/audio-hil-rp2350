#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the DSP processor (FFT instance, window coefficients, frequency bins).
 * 
 * @param fft_size FFT size (must be a power of 2, e.g. 1024, 2048).
 * @param sample_rate Audio sampling rate in Hz (e.g. 16000).
 * @return int 0 on success, negative value on error.
 */
int dsp_processor_init(size_t fft_size, uint32_t sample_rate);

/**
 * @brief Process an audio buffer and estimate the fundamental frequency.
 * 
 * @param input_samples Pointer to float array of size fft_size containing normalized samples [-1.0, 1.0].
 * @return float The estimated fundamental frequency in Hz, or 0.0f if silence/no tone is detected.
 */
float dsp_processor_process(const float* input_samples);

/**
 * @brief Extended DSP process function providing full spectral peak metrics for HIL testing.
 * 
 * @param input_samples Pointer to normalized input samples [-1.0, 1.0].
 * @param out_max_mag Optional pointer to store the maximum spectral magnitude peak.
 * @param out_peak_bin Optional pointer to store the discrete FFT bin index of the peak.
 * @param out_status Optional pointer to store HIL_STATUS_OK or HIL_STATUS_SILENCE.
 * @return float The estimated fundamental frequency in Hz, or 0.0f if silence.
 */
float dsp_processor_process_ext(const float* input_samples, 
                                float* out_max_mag, 
                                uint32_t* out_peak_bin, 
                                uint8_t* out_status);

/**
 * @brief Update the DSP thresholds dynamically (useful for HIL noise floor sensitivity sweeps).
 * 
 * @param noise_threshold Cutoff magnitude. Peaks below this report silence.
 * @param min_freq Minimum valid fundamental frequency in Hz (e.g. 50.0f).
 * @param max_freq Maximum valid fundamental frequency in Hz (e.g. 3000.0f).
 * @return int 0 on success, negative value on error.
 */
int dsp_processor_set_thresholds(float noise_threshold, float min_freq, float max_freq);

/**
 * @brief Retrieve currently configured thresholds and parameters.
 */
void dsp_processor_get_thresholds(float* out_noise_threshold, float* out_min_freq, float* out_max_freq);

/**
 * @brief Deinitialize the DSP processor and free allocated memory.
 */
void dsp_processor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DSP_PROCESSOR_H
