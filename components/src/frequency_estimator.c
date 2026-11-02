#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "protocol.h"
#include "hil_hal.h"
#include "dsp_processor.h"
#include "frequency_estimator.h"

static float* samples_buffer = NULL;
static size_t current_fft_size = 0;
static uint32_t current_sample_rate = 0;
static bool is_running = false;

int frequency_estimator_init(uint32_t sample_rate, size_t fft_size) {
    current_sample_rate = sample_rate;
    current_fft_size = fft_size;

    // 1. Initialize HIL USB HAL Layer
    int status = hil_hal_init(sample_rate, fft_size);
    if (status != 0) {
        return -1;
    }

    // 2. Initialize CMSIS-DSP Processor
    status = dsp_processor_init(fft_size, sample_rate);
    if (status != 0) {
        hil_hal_deinit();
        return -2;
    }

    // 3. Allocate intermediate sample buffer
    if (samples_buffer) free(samples_buffer);
    samples_buffer = malloc(fft_size * sizeof(float));
    if (!samples_buffer) {
        dsp_processor_deinit();
        hil_hal_deinit();
        return -3;
    }

    is_running = true;
    return 0;
}

void frequency_estimator_start(void) {
    is_running = true;
}

void frequency_estimator_stop(void) {
    is_running = false;
}

static bool process_and_respond(uint16_t seq_id, float* out_freq, uint32_t* out_latency_us) {
    uint32_t start_time = time_us_32();

    float max_mag = 0.0f;
    uint32_t peak_bin = 0;
    uint8_t status = HIL_STATUS_OK;

    // Run CMSIS-DSP processing pipeline
    float estimated_freq = dsp_processor_process_ext(samples_buffer, &max_mag, &peak_bin, &status);

    uint32_t end_time = time_us_32();
    uint32_t latency_us = end_time - start_time;

    // Transmit response packet immediately back through USB CDC
    hil_hal_send_estimate(seq_id, status, estimated_freq, max_mag, peak_bin, latency_us);

    if (out_freq) *out_freq = estimated_freq;
    if (out_latency_us) *out_latency_us = latency_us;

    return true;
}

bool frequency_estimator_step(float* out_freq, uint32_t* out_latency_us) {
    if (!samples_buffer || !is_running) {
        return false;
    }

    uint16_t seq_id = 0;
    // Block until an audio injection block arrives
    bool got_samples = hil_hal_get_samples(samples_buffer, true, &seq_id);
    if (!got_samples) {
        return false;
    }

    return process_and_respond(seq_id, out_freq, out_latency_us);
}

bool frequency_estimator_poll(bool* out_processed) {
    if (out_processed) *out_processed = false;
    if (!samples_buffer || !is_running) {
        return false;
    }

    uint16_t seq_id = 0;
    bool got_samples = hil_hal_get_samples(samples_buffer, false, &seq_id);
    if (got_samples) {
        if (out_processed) *out_processed = true;
        return process_and_respond(seq_id, NULL, NULL);
    }

    return true;
}

void frequency_estimator_deinit(void) {
    is_running = false;
    if (samples_buffer) {
        free(samples_buffer);
        samples_buffer = NULL;
    }
    dsp_processor_deinit();
    hil_hal_deinit();
}
