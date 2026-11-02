#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "protocol.h"
#include "dsp_processor.h"
#include "hil_hal.h"

// Parser State Machine
typedef enum {
    PARSE_MAGIC_0 = 0,
    PARSE_MAGIC_1,
    PARSE_MAGIC_2,
    PARSE_MAGIC_3,
    PARSE_HEADER_REST,
    PARSE_PAYLOAD,
    PARSE_CRC_0,
    PARSE_CRC_1
} parser_state_t;

static uint32_t hal_sample_rate = 16000;
static size_t hal_fft_size = 2048;

// Double buffer for audio samples: normalized float [-1.0, 1.0]
static float* sample_buffer = NULL;
static volatile bool sample_buffer_ready = false;
static volatile uint16_t current_seq_id = 0;

// Parser working variables
static parser_state_t p_state = PARSE_MAGIC_0;
static hil_header_t rx_header;
static uint8_t* rx_payload_buf = NULL;
static size_t rx_payload_max = 0;
static size_t rx_bytes_received = 0;
static uint16_t rx_crc = 0;

static void hil_hal_reset_parser(void) {
    p_state = PARSE_MAGIC_0;
    rx_bytes_received = 0;
}

int hil_hal_init(uint32_t sample_rate, size_t fft_size) {
    hal_sample_rate = sample_rate;
    hal_fft_size = fft_size;

    // Ensure pristine binary transfers without CR/LF modification
    stdio_set_translate_crlf(&stdio_usb, false);

    // Free buffers if already allocated
    if (sample_buffer) free(sample_buffer);
    if (rx_payload_buf) free(rx_payload_buf);

    // Allocate float sample buffer
    sample_buffer = malloc(fft_size * sizeof(float));
    if (!sample_buffer) {
        return -1;
    }

    // Allocate payload reception buffer (must hold up to fft_size * sizeof(float) = 8192 bytes)
    rx_payload_max = fft_size * sizeof(float);
    if (rx_payload_max < 256) rx_payload_max = 256;
    rx_payload_buf = malloc(rx_payload_max);
    if (!rx_payload_buf) {
        free(sample_buffer);
        sample_buffer = NULL;
        return -2;
    }

    sample_buffer_ready = false;
    current_seq_id = 0;
    hil_hal_reset_parser();

    return 0;
}

int hil_hal_send_packet(uint8_t msg_type, uint16_t seq_id, const void* payload, uint16_t payload_len) {
    hil_header_t hdr;
    hdr.magic[0] = HIL_MAGIC_BYTE_0;
    hdr.magic[1] = HIL_MAGIC_BYTE_1;
    hdr.magic[2] = HIL_MAGIC_BYTE_2;
    hdr.magic[3] = HIL_MAGIC_BYTE_3;
    hdr.msg_type = msg_type;
    hdr.seq_id = seq_id;
    hdr.payload_len = payload_len;

    // Calculate CRC-16 over Header + Payload
    uint16_t crc = hil_crc16((const uint8_t*)&hdr, sizeof(hdr));
    if (payload && payload_len > 0) {
        crc = hil_crc16_update(crc, (const uint8_t*)payload, payload_len);
    }

    // Direct binary write via stdio_usb driver
    if (stdio_usb.out_chars) {
        stdio_usb.out_chars((const char*)&hdr, sizeof(hdr));
        if (payload && payload_len > 0) {
            stdio_usb.out_chars((const char*)payload, payload_len);
        }
        stdio_usb.out_chars((const char*)&crc, sizeof(crc));
        if (stdio_usb.out_flush) {
            stdio_usb.out_flush();
        }
    } else {
        // Fallback to standard fwrite
        fwrite(&hdr, 1, sizeof(hdr), stdout);
        if (payload && payload_len > 0) {
            fwrite(payload, 1, payload_len, stdout);
        }
        fwrite(&crc, 1, sizeof(crc), stdout);
        fflush(stdout);
    }

    return 0;
}

static void hil_hal_send_error(uint16_t seq_id, uint8_t error_code, uint8_t original_cmd) {
    hil_rsp_error_t err;
    err.seq_id = seq_id;
    err.error_code = error_code;
    err.original_cmd = original_cmd;
    hil_hal_send_packet(HIL_RSP_ERROR, seq_id, &err, sizeof(err));
}

int hil_hal_send_estimate(uint16_t seq_id, 
                          uint8_t status, 
                          float est_freq, 
                          float max_mag, 
                          uint32_t peak_bin, 
                          uint32_t dsp_latency_us) {
    hil_rsp_estimate_t rsp;
    rsp.seq_id = seq_id;
    rsp.status = status;
    rsp.reserved = 0;
    rsp.estimated_freq = est_freq;
    rsp.max_magnitude = max_mag;
    rsp.peak_bin = peak_bin;
    rsp.dsp_latency_us = dsp_latency_us;

    return hil_hal_send_packet(HIL_RSP_ESTIMATE, seq_id, &rsp, sizeof(rsp));
}

// Dispatches a fully received and verified packet
static bool hil_hal_dispatch_packet(void) {
    // 1. Verify CRC16
    uint16_t computed_crc = hil_crc16((const uint8_t*)&rx_header, sizeof(rx_header));
    if (rx_header.payload_len > 0) {
        computed_crc = hil_crc16_update(computed_crc, rx_payload_buf, rx_header.payload_len);
    }

    if (computed_crc != rx_crc) {
        hil_hal_send_error(rx_header.seq_id, HIL_ERR_CRC_MISMATCH, rx_header.msg_type);
        return false;
    }

    // 2. Process Command
    switch (rx_header.msg_type) {
        case HIL_CMD_PING:
            // Echo ping
            hil_hal_send_packet(HIL_RSP_PONG, rx_header.seq_id, rx_payload_buf, rx_header.payload_len);
            break;

        case HIL_CMD_GET_CONFIG: {
            hil_rsp_config_t cfg;
            cfg.sample_rate = hal_sample_rate;
            cfg.fft_size = (uint32_t)hal_fft_size;
            dsp_processor_get_thresholds(&cfg.noise_threshold, &cfg.min_freq, &cfg.max_freq);
            hil_hal_send_packet(HIL_RSP_CONFIG, rx_header.seq_id, &cfg, sizeof(cfg));
            break;
        }

        case HIL_CMD_SET_CONFIG: {
            if (rx_header.payload_len >= sizeof(hil_cmd_set_config_t)) {
                const hil_cmd_set_config_t* set_cfg = (const hil_cmd_set_config_t*)rx_payload_buf;
                dsp_processor_set_thresholds(set_cfg->noise_threshold, set_cfg->min_freq, set_cfg->max_freq);
                hil_hal_send_packet(HIL_RSP_ACK, rx_header.seq_id, NULL, 0);
            } else {
                hil_hal_send_error(rx_header.seq_id, HIL_ERR_INVALID_LENGTH, rx_header.msg_type);
            }
            break;
        }

        case HIL_CMD_RESET:
            sample_buffer_ready = false;
            hil_hal_reset_parser();
            hil_hal_send_packet(HIL_RSP_ACK, rx_header.seq_id, NULL, 0);
            break;

        case HIL_CMD_INJECT_PCM16: {
            size_t expected_bytes = hal_fft_size * sizeof(int16_t);
            if (rx_header.payload_len != expected_bytes) {
                hil_hal_send_error(rx_header.seq_id, HIL_ERR_INVALID_LENGTH, rx_header.msg_type);
                break;
            }
            // Normalize int16_t PCM [-32768, 32767] to float [-1.0, 1.0]
            const int16_t* pcm16 = (const int16_t*)rx_payload_buf;
            for (size_t i = 0; i < hal_fft_size; i++) {
                sample_buffer[i] = (float)pcm16[i] / 32768.0f;
            }
            current_seq_id = rx_header.seq_id;
            sample_buffer_ready = true;
            return true; // Audio block ready!
        }

        case HIL_CMD_INJECT_FLOAT32: {
            size_t expected_bytes = hal_fft_size * sizeof(float);
            if (rx_header.payload_len != expected_bytes) {
                hil_hal_send_error(rx_header.seq_id, HIL_ERR_INVALID_LENGTH, rx_header.msg_type);
                break;
            }
            // Copy float samples directly
            memcpy(sample_buffer, rx_payload_buf, expected_bytes);
            current_seq_id = rx_header.seq_id;
            sample_buffer_ready = true;
            return true; // Audio block ready!
        }

        default:
            hil_hal_send_error(rx_header.seq_id, HIL_ERR_UNKNOWN_CMD, rx_header.msg_type);
            break;
    }

    return false;
}

// Feed a single byte into the stream parser
static bool hil_hal_process_byte(uint8_t byte) {
    switch (p_state) {
        case PARSE_MAGIC_0:
            if (byte == HIL_MAGIC_BYTE_0) {
                rx_header.magic[0] = byte;
                p_state = PARSE_MAGIC_1;
            }
            break;

        case PARSE_MAGIC_1:
            if (byte == HIL_MAGIC_BYTE_1) {
                rx_header.magic[1] = byte;
                p_state = PARSE_MAGIC_2;
            } else if (byte == HIL_MAGIC_BYTE_0) {
                // Stay in state 1
            } else {
                p_state = PARSE_MAGIC_0;
            }
            break;

        case PARSE_MAGIC_2:
            if (byte == HIL_MAGIC_BYTE_2) {
                rx_header.magic[2] = byte;
                p_state = PARSE_MAGIC_3;
            } else if (byte == HIL_MAGIC_BYTE_0) {
                p_state = PARSE_MAGIC_1;
            } else {
                p_state = PARSE_MAGIC_0;
            }
            break;

        case PARSE_MAGIC_3:
            if (byte == HIL_MAGIC_BYTE_3) {
                rx_header.magic[3] = byte;
                rx_bytes_received = 0;
                p_state = PARSE_HEADER_REST;
            } else if (byte == HIL_MAGIC_BYTE_0) {
                p_state = PARSE_MAGIC_1;
            } else {
                p_state = PARSE_MAGIC_0;
            }
            break;

        case PARSE_HEADER_REST: {
            uint8_t* hdr_ptr = ((uint8_t*)&rx_header) + 4;
            size_t remaining_header_bytes = sizeof(hil_header_t) - 4; // 5 bytes: msg_type (1), seq_id (2), payload_len (2)
            hdr_ptr[rx_bytes_received++] = byte;

            if (rx_bytes_received >= remaining_header_bytes) {
                // Header complete. Check payload size bounds.
                if (rx_header.payload_len > rx_payload_max) {
                    hil_hal_send_error(rx_header.seq_id, HIL_ERR_BUFFER_OVERFLOW, rx_header.msg_type);
                    p_state = PARSE_MAGIC_0;
                } else if (rx_header.payload_len == 0) {
                    p_state = PARSE_CRC_0;
                } else {
                    rx_bytes_received = 0;
                    p_state = PARSE_PAYLOAD;
                }
            }
            break;
        }

        case PARSE_PAYLOAD:
            rx_payload_buf[rx_bytes_received++] = byte;
            if (rx_bytes_received >= rx_header.payload_len) {
                p_state = PARSE_CRC_0;
            }
            break;

        case PARSE_CRC_0:
            rx_crc = (uint16_t)byte;
            p_state = PARSE_CRC_1;
            break;

        case PARSE_CRC_1:
            rx_crc |= ((uint16_t)byte) << 8;
            p_state = PARSE_MAGIC_0;
            // Fully framed packet: dispatch
            return hil_hal_dispatch_packet();

        default:
            p_state = PARSE_MAGIC_0;
            break;
    }

    return false;
}

bool hil_hal_poll(void) {
    bool block_ready = false;

    // Use buffered block reads from stdio_usb for high throughput
    char chunk[128];
    int count = 0;
    if (stdio_usb.in_chars) {
        count = stdio_usb.in_chars(chunk, sizeof(chunk));
    }

    if (count > 0) {
        for (int i = 0; i < count; i++) {
            if (hil_hal_process_byte((uint8_t)chunk[i])) {
                block_ready = true;
            }
        }
    } else {
        // Fallback single character poll
        int c = getchar_timeout_us(0);
        while (c >= 0) {
            if (hil_hal_process_byte((uint8_t)c)) {
                block_ready = true;
            }
            c = getchar_timeout_us(0);
        }
    }

    return block_ready;
}

bool hil_hal_is_buffer_ready(void) {
    return sample_buffer_ready;
}

bool hil_hal_get_samples(float* out_samples, bool block, uint16_t* out_seq_id) {
    if (block) {
        while (!sample_buffer_ready) {
            hil_hal_poll();
            tight_loop_contents();
        }
    } else {
        hil_hal_poll();
        if (!sample_buffer_ready) {
            return false;
        }
    }

    // Copy to caller's buffer
    memcpy(out_samples, sample_buffer, hal_fft_size * sizeof(float));
    if (out_seq_id) *out_seq_id = current_seq_id;

    sample_buffer_ready = false;
    return true;
}

void hil_hal_deinit(void) {
    if (sample_buffer) {
        free(sample_buffer);
        sample_buffer = NULL;
    }
    if (rx_payload_buf) {
        free(rx_payload_buf);
        rx_payload_buf = NULL;
    }
    sample_buffer_ready = false;
    hil_hal_reset_parser();
}
