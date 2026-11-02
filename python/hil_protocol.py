"""
Hardware-in-the-Loop (HIL) Protocol Definitions and Packet Codec.
Compatible with RP2350 audio-hil firmware.
"""

import struct
import binascii
from dataclasses import dataclass
from typing import Tuple, Optional

# Synchronization Magic (4 bytes: "HIL1")
MAGIC = b"HIL1"

# Command IDs (Host -> Device)
CMD_PING           = 0x01
CMD_GET_CONFIG     = 0x02
CMD_SET_CONFIG     = 0x03
CMD_INJECT_PCM16   = 0x10
CMD_INJECT_FLOAT32 = 0x11
CMD_RESET          = 0x20

# Response IDs (Device -> Host)
RSP_ACK      = 0x80
RSP_PONG     = 0x81
RSP_CONFIG   = 0x82
RSP_ESTIMATE = 0x90
RSP_ERROR    = 0xFF

# Estimation Status Codes
STATUS_OK            = 0x00
STATUS_SILENCE       = 0x01
STATUS_OUT_OF_BOUNDS = 0x02

STATUS_NAMES = {
    STATUS_OK: "OK",
    STATUS_SILENCE: "SILENCE / NO_TONE",
    STATUS_OUT_OF_BOUNDS: "OUT_OF_BOUNDS"
}

# Error Codes
ERR_NONE            = 0x00
ERR_INVALID_MAGIC   = 0x01
ERR_CRC_MISMATCH    = 0x02
ERR_UNKNOWN_CMD     = 0x03
ERR_BUFFER_OVERFLOW = 0x04
ERR_INVALID_LENGTH  = 0x05

HEADER_FORMAT = "<4sBHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 9 bytes


@dataclass
class HILEstimateResult:
    seq_id: int
    status: int
    estimated_freq: float
    max_magnitude: float
    peak_bin: int
    dsp_latency_us: int
    round_trip_ms: float = 0.0

    @property
    def status_str(self) -> str:
        return STATUS_NAMES.get(self.status, f"UNKNOWN(0x{self.status:02X})")


@dataclass
class HILDeviceConfig:
    sample_rate: int
    fft_size: int
    noise_threshold: float
    min_freq: float
    max_freq: float


def compute_crc16(data: bytes) -> int:
    """Calculates CRC-16-CCITT matching RP2350 implementation."""
    return binascii.crc_hqx(data, 0xFFFF)


def encode_packet(msg_type: int, seq_id: int, payload: bytes = b"") -> bytes:
    """Encodes a full HIL packet: Header + Payload + CRC16."""
    header = struct.pack(HEADER_FORMAT, MAGIC, msg_type, seq_id, len(payload))
    crc = compute_crc16(header + payload)
    return header + payload + struct.pack("<H", crc)


def decode_packet(raw_data: bytes) -> Tuple[Optional[int], Optional[int], Optional[bytes]]:
    """
    Decodes and validates a raw HIL packet.
    Returns: (msg_type, seq_id, payload) or (None, None, None) on validation error.
    """
    if len(raw_data) < HEADER_SIZE + 2:
        return None, None, None

    magic, msg_type, seq_id, payload_len = struct.unpack_from(HEADER_FORMAT, raw_data, 0)
    if magic != MAGIC:
        return None, None, None

    if len(raw_data) < HEADER_SIZE + payload_len + 2:
        return None, None, None

    payload = raw_data[HEADER_SIZE : HEADER_SIZE + payload_len]
    (rx_crc,) = struct.unpack_from("<H", raw_data, HEADER_SIZE + payload_len)

    computed_crc = compute_crc16(raw_data[: HEADER_SIZE + payload_len])
    if rx_crc != computed_crc:
        raise ValueError(f"CRC mismatch: expected 0x{computed_crc:04X}, received 0x{rx_crc:04X}")

    return msg_type, seq_id, payload


def unpack_estimate_result(payload: bytes) -> HILEstimateResult:
    """Unpacks a HIL_RSP_ESTIMATE payload (20 bytes)."""
    fmt = "<HBBffII"
    if len(payload) < struct.calcsize(fmt):
        raise ValueError(f"Payload too short for estimate response ({len(payload)} bytes)")
    seq_id, status, _, est_freq, max_mag, peak_bin, dsp_lat = struct.unpack_from(fmt, payload, 0)
    return HILEstimateResult(
        seq_id=seq_id,
        status=status,
        estimated_freq=est_freq,
        max_magnitude=max_mag,
        peak_bin=peak_bin,
        dsp_latency_us=dsp_lat
    )


def unpack_config(payload: bytes) -> HILDeviceConfig:
    """Unpacks a HIL_RSP_CONFIG payload (20 bytes)."""
    fmt = "<IIfff"
    if len(payload) < struct.calcsize(fmt):
        raise ValueError(f"Payload too short for config response ({len(payload)} bytes)")
    sample_rate, fft_size, noise_th, min_f, max_f = struct.unpack_from(fmt, payload, 0)
    return HILDeviceConfig(
        sample_rate=sample_rate,
        fft_size=fft_size,
        noise_threshold=noise_th,
        min_freq=min_f,
        max_freq=max_f
    )
