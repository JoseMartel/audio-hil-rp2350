#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 4-byte Magic Synchronization Header: "HIL1" (0x314C4948)
#define HIL_MAGIC_BYTE_0    'H'
#define HIL_MAGIC_BYTE_1    'I'
#define HIL_MAGIC_BYTE_2    'L'
#define HIL_MAGIC_BYTE_3    '1'

// Command IDs (Host -> RP2350)
#define HIL_CMD_PING            0x01
#define HIL_CMD_GET_CONFIG      0x02
#define HIL_CMD_SET_CONFIG      0x03
#define HIL_CMD_INJECT_PCM16    0x10
#define HIL_CMD_INJECT_FLOAT32  0x11
#define HIL_CMD_RESET           0x20

// Response IDs (RP2350 -> Host)
#define HIL_RSP_ACK             0x80
#define HIL_RSP_PONG            0x81
#define HIL_RSP_CONFIG          0x82
#define HIL_RSP_ESTIMATE        0x90
#define HIL_RSP_ERROR           0xFF

// Estimation Status Codes
#define HIL_STATUS_OK           0x00
#define HIL_STATUS_SILENCE      0x01
#define HIL_STATUS_OUT_OF_BOUNDS 0x02

// Protocol Error Codes
#define HIL_ERR_NONE            0x00
#define HIL_ERR_INVALID_MAGIC   0x01
#define HIL_ERR_CRC_MISMATCH    0x02
#define HIL_ERR_UNKNOWN_CMD     0x03
#define HIL_ERR_BUFFER_OVERFLOW 0x04
#define HIL_ERR_INVALID_LENGTH  0x05

#pragma pack(push, 1)

// Packet Header (9 bytes)
typedef struct {
    uint8_t  magic[4];       // "HIL1"
    uint8_t  msg_type;       // HIL_CMD_* or HIL_RSP_*
    uint16_t seq_id;         // Sequence counter
    uint16_t payload_len;    // Bytes of payload following this header
} hil_header_t;

// Payload for HIL_RSP_ESTIMATE
typedef struct {
    uint16_t seq_id;          // Sequence counter echoed
    uint8_t  status;          // HIL_STATUS_*
    uint8_t  reserved;        // Alignment padding
    float    estimated_freq;  // Fundamental frequency in Hz (0.0 if silence)
    float    max_magnitude;   // Peak FFT magnitude
    uint32_t peak_bin;        // FFT bin of dominant frequency
    uint32_t dsp_latency_us;  // Execution time in microseconds
} hil_rsp_estimate_t;

// Payload for HIL_RSP_CONFIG
typedef struct {
    uint32_t sample_rate;     // Sampling rate in Hz (e.g. 16000)
    uint32_t fft_size;        // FFT buffer size (e.g. 2048)
    float    noise_threshold; // Silence cutoff magnitude
    float    min_freq;        // Minimum search frequency in Hz (e.g. 50.0)
    float    max_freq;        // Maximum search frequency in Hz (e.g. 3000.0)
} hil_rsp_config_t;

// Payload for HIL_CMD_SET_CONFIG
typedef struct {
    float    noise_threshold;
    float    min_freq;
    float    max_freq;
} hil_cmd_set_config_t;

// Payload for HIL_RSP_ERROR
typedef struct {
    uint16_t seq_id;
    uint8_t  error_code;
    uint8_t  original_cmd;
} hil_rsp_error_t;

#pragma pack(pop)

/**
 * @brief Calculate CRC-16-CCITT (polynomial 0x1021, init 0xFFFF).
 * Matches Python binascii.crc_hqx(data, 0xFFFF).
 */
static inline uint16_t hil_crc16_update(uint16_t crc, const uint8_t *data, size_t length) {
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static inline uint16_t hil_crc16(const uint8_t *data, size_t length) {
    return hil_crc16_update(0xFFFF, data, length);
}

#ifdef __cplusplus
}
#endif

#endif // HIL_PROTOCOL_H
