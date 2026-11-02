#ifndef FREQUENCY_ESTIMATOR_H
#define FREQUENCY_ESTIMATOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the frequency estimation HIL coordinator.
 * 
 * Sets up both the HIL HAL (USB CDC stream parser) and the DSP Processor (CMSIS-DSP FFT).
 * 
 * @param sample_rate Audio sampling rate in Hz (e.g. 16000).
 * @param fft_size FFT processing buffer size (e.g. 2048).
 * @return int 0 on success, negative value on error.
 */
int frequency_estimator_init(uint32_t sample_rate, size_t fft_size);

/**
 * @brief Start the HIL estimator system.
 */
void frequency_estimator_start(void);

/**
 * @brief Stop the HIL estimator system.
 */
void frequency_estimator_stop(void);

/**
 * @brief Run one step of the HIL estimator:
 * Blocks until an injected audio block is received via USB CDC, runs the CMSIS-DSP
 * frequency estimator, measures processing latency, and sends the response back to the host.
 * 
 * @param out_freq Optional pointer to receive estimated frequency.
 * @param out_latency_us Optional pointer to receive DSP calculation latency.
 * @return bool true if a block was processed and replied to, false on error.
 */
bool frequency_estimator_step(float* out_freq, uint32_t* out_latency_us);

/**
 * @brief Non-blocking poll of the HIL estimator.
 * Polls USB CDC; if a block has completed, processes it immediately and transmits the response.
 * 
 * @param out_processed true if a block was processed during this poll.
 * @return bool true if healthy, false on error.
 */
bool frequency_estimator_poll(bool* out_processed);

/**
 * @brief Deinitialize and release estimator resources.
 */
void frequency_estimator_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // FREQUENCY_ESTIMATOR_H
