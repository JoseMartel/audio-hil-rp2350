#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "frequency_estimator.h"

#define SAMPLE_RATE 16000
#define FFT_SIZE    2048

int main(void) {
    // 1. Initialize Pico SDK Standard I/O (USB CDC)
    stdio_init_all();

    // 2. Disable CR/LF translation to guarantee byte-for-byte binary purity
    stdio_set_translate_crlf(&stdio_usb, false);

    // 3. Initialize HIL Frequency Estimator (USB CDC HAL + CMSIS-DSP Real FFT)
    int status = frequency_estimator_init(SAMPLE_RATE, FFT_SIZE);
    if (status != 0) {
        // Blink LED or halt if initialization failed
        while (true) {
            sleep_ms(500);
        }
    }

    // 4. Start HIL execution engine
    frequency_estimator_start();

    // 5. Main Event Loop:
    // Listens for USB CDC binary commands from Python host.
    // Handles PING, GET_CONFIG, SET_CONFIG, and INJECT_AUDIO.
    // Automatically estimates fundamental frequency and returns results with latency profiling.
    while (true) {
        float freq = 0.0f;
        uint32_t latency_us = 0;
        frequency_estimator_step(&freq, &latency_us);
    }

    return 0;
}
