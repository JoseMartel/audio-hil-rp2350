#ifndef HIL_HAL_H
#define HIL_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Hardware-in-the-Loop Audio HAL over USB CDC.
 * 
 * @param sample_rate Expected audio sampling rate (e.g. 16000).
 * @param fft_size Expected number of audio samples per block (e.g. 2048).
 * @return int 0 on success, negative value on error.
 */
int hil_hal_init(uint32_t sample_rate, size_t fft_size);

/**
 * @brief Poll USB CDC and process incoming bytes into the HIL packet parser.
 * Handles configuration and ping commands internally.
 * 
 * @return true if a new audio block has been completely received and validated.
 * @return false otherwise.
 */
bool hil_hal_poll(void);

/**
 * @brief Check if a new injected audio block is ready.
 */
bool hil_hal_is_buffer_ready(void);

/**
 * @brief Retrieve the latest injected audio samples (normalized to [-1.0, 1.0]).
 * 
 * @param out_samples Output buffer of size fft_size.
 * @param block If true, blocks execution until a valid audio block is received via USB.
 * @param out_seq_id Pointer to store the sequence ID associated with this injected block.
 * @return true if samples were retrieved, false on timeout/error.
 */
bool hil_hal_get_samples(float* out_samples, bool block, uint16_t* out_seq_id);

/**
 * @brief Send an estimation result packet back to the host via USB CDC.
 * 
 * @param seq_id Sequence ID matching the injected block.
 * @param status Status code (HIL_STATUS_OK, HIL_STATUS_SILENCE, etc.).
 * @param est_freq Calculated fundamental frequency in Hz.
 * @param max_mag Peak FFT magnitude.
 * @param peak_bin Discrete FFT bin index.
 * @param dsp_latency_us Processing time in microseconds.
 * @return int 0 on success, negative value on error.
 */
int hil_hal_send_estimate(uint16_t seq_id, 
                          uint8_t status, 
                          float est_freq, 
                          float max_mag, 
                          uint32_t peak_bin, 
                          uint32_t dsp_latency_us);

/**
 * @brief Send a generic raw HIL packet to the host.
 */
int hil_hal_send_packet(uint8_t msg_type, uint16_t seq_id, const void* payload, uint16_t payload_len);

/**
 * @brief Clean up and reset HIL HAL state.
 */
void hil_hal_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HIL_HAL_H
