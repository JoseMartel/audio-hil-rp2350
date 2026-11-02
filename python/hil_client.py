"""
HIL USB Serial Client for RP2350.
Supports auto-port detection, packet framing, and an offline mock mode for CI/testing.
"""

import time
import serial
import serial.tools.list_ports
import numpy as np
from typing import Optional, List, Tuple

from hil_protocol import (
    CMD_PING, CMD_GET_CONFIG, CMD_SET_CONFIG, CMD_INJECT_PCM16, CMD_INJECT_FLOAT32, CMD_RESET,
    RSP_PONG, RSP_CONFIG, RSP_ESTIMATE, RSP_ACK, RSP_ERROR,
    HILEstimateResult, HILDeviceConfig,
    encode_packet, decode_packet, unpack_estimate_result, unpack_config,
    STATUS_OK, STATUS_SILENCE
)
from signal_generator import to_pcm16_bytes, to_float32_bytes


def auto_detect_pico_port() -> Optional[str]:
    """Scans system serial ports for Raspberry Pi Pico (VID: 0x2E8A)."""
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x2E8A:
            return port.device
    return None


class MockRP2350Simulator:
    """Software simulation of RP2350 CMSIS-DSP frequency estimator for offline testing."""
    def __init__(self, sample_rate: int = 16000, fft_size: int = 2048):
        self.sample_rate = sample_rate
        self.fft_size = fft_size
        self.noise_threshold = 2.0
        self.min_freq = 50.0
        self.max_freq = 3000.0
        self.window = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(fft_size) / (fft_size - 1)))

    def estimate(self, samples: np.ndarray, seq_id: int) -> HILEstimateResult:
        # 1. Hann window
        windowed = samples * self.window
        # 2. Real FFT
        rfft = np.fft.rfft(windowed, n=self.fft_size)
        # 3. Magnitude spectrum
        mag = np.abs(rfft)

        bin_res = self.sample_rate / self.fft_size
        k_min = max(1, int(self.min_freq / bin_res))
        k_max = min(self.fft_size // 2 - 1, int(self.max_freq / bin_res))

        search_slice = mag[k_min : k_max + 1]
        peak_idx = np.argmax(search_slice)
        peak_bin = k_min + peak_idx
        max_mag = float(search_slice[peak_idx])

        if max_mag < self.noise_threshold:
            return HILEstimateResult(
                seq_id=seq_id,
                status=STATUS_SILENCE,
                estimated_freq=0.0,
                max_magnitude=max_mag,
                peak_bin=peak_bin,
                dsp_latency_us=380
            )

        # Boundary check: A true local peak must not have a higher adjacent bin outside the search slice
        from hil_protocol import STATUS_OUT_OF_BOUNDS
        if peak_bin == k_min and k_min > 0 and mag[k_min - 1] >= max_mag:
            return HILEstimateResult(
                seq_id=seq_id,
                status=STATUS_OUT_OF_BOUNDS,
                estimated_freq=0.0,
                max_magnitude=max_mag,
                peak_bin=peak_bin,
                dsp_latency_us=380
            )
        if peak_bin == k_max and (k_max + 1) < (self.fft_size // 2) and mag[k_max + 1] >= max_mag:
            return HILEstimateResult(
                seq_id=seq_id,
                status=STATUS_OUT_OF_BOUNDS,
                estimated_freq=0.0,
                max_magnitude=max_mag,
                peak_bin=peak_bin,
                dsp_latency_us=380
            )

        # Quadratic interpolation
        d = 0.0
        if k_min < peak_bin < k_max:
            y1 = mag[peak_bin - 1]
            y2 = mag[peak_bin]
            y3 = mag[peak_bin + 1]
            denom = y1 - 2.0 * y2 + y3
            if abs(denom) > 1e-5:
                d = 0.5 * (y1 - y3) / denom

        est_freq = float((peak_bin + d) * bin_res)
        if est_freq < (self.min_freq - 2.0) or est_freq > (self.max_freq + 2.0):
            return HILEstimateResult(
                seq_id=seq_id,
                status=STATUS_OUT_OF_BOUNDS,
                estimated_freq=0.0,
                max_magnitude=max_mag,
                peak_bin=peak_bin,
                dsp_latency_us=380
            )

        return HILEstimateResult(
            seq_id=seq_id,
            status=STATUS_OK,
            estimated_freq=est_freq,
            max_magnitude=max_mag,
            peak_bin=peak_bin,
            dsp_latency_us=380
        )


class HILClient:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, timeout: float = 1.0, mock: bool = False):
        self.mock = mock
        self.seq_id = 0
        self.mock_sim = MockRP2350Simulator() if mock else None
        self.ser: Optional[serial.Serial] = None

        if not mock:
            if port is None:
                port = auto_detect_pico_port()
                if port is None:
                    raise RuntimeError("No RP2350 Pico detected. Specify port or use --mock.")
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            # Clear input/output buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.05)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _next_seq(self) -> int:
        self.seq_id = (self.seq_id + 1) & 0xFFFF
        return self.seq_id

    def ping(self, payload: bytes = b"PING") -> bool:
        seq = self._next_seq()
        if self.mock:
            return True

        packet = encode_packet(CMD_PING, seq, payload)
        self.ser.write(packet)
        self.ser.flush()

        msg_type, rx_seq, rx_payload = self._read_packet()
        return msg_type == RSP_PONG and rx_seq == seq

    def get_config(self) -> HILDeviceConfig:
        seq = self._next_seq()
        if self.mock:
            return HILDeviceConfig(
                sample_rate=self.mock_sim.sample_rate,
                fft_size=self.mock_sim.fft_size,
                noise_threshold=self.mock_sim.noise_threshold,
                min_freq=self.mock_sim.min_freq,
                max_freq=self.mock_sim.max_freq
            )

        packet = encode_packet(CMD_GET_CONFIG, seq)
        self.ser.write(packet)
        self.ser.flush()

        msg_type, rx_seq, payload = self._read_packet()
        if msg_type != RSP_CONFIG:
            raise RuntimeError(f"Unexpected response type 0x{msg_type:02X} for GET_CONFIG")
        return unpack_config(payload)

    def set_config(self, noise_threshold: float, min_freq: float, max_freq: float) -> bool:
        seq = self._next_seq()
        if self.mock:
            self.mock_sim.noise_threshold = noise_threshold
            self.mock_sim.min_freq = min_freq
            self.mock_sim.max_freq = max_freq
            return True

        import struct
        payload = struct.pack("<fff", noise_threshold, min_freq, max_freq)
        packet = encode_packet(CMD_SET_CONFIG, seq, payload)
        self.ser.write(packet)
        self.ser.flush()

        msg_type, rx_seq, _ = self._read_packet()
        return msg_type == RSP_ACK and rx_seq == seq

    def reset_device(self) -> bool:
        seq = self._next_seq()
        if self.mock:
            return True

        packet = encode_packet(CMD_RESET, seq)
        self.ser.write(packet)
        self.ser.flush()

        msg_type, rx_seq, _ = self._read_packet()
        return msg_type == RSP_ACK and rx_seq == seq

    def inject_block(self, samples_float: np.ndarray, format_type: str = "float32") -> HILEstimateResult:
        """
        Injects an audio block and blocks until the estimation result is returned.
        format_type: 'float32' (CMD_INJECT_FLOAT32) or 'pcm16' (CMD_INJECT_PCM16).
        """
        seq = self._next_seq()
        t_start = time.perf_counter()

        if self.mock:
            result = self.mock_sim.estimate(samples_float, seq)
            result.round_trip_ms = (time.perf_counter() - t_start) * 1000.0
            return result

        if format_type == "pcm16":
            cmd = CMD_INJECT_PCM16
            payload = to_pcm16_bytes(samples_float)
        else:
            cmd = CMD_INJECT_FLOAT32
            payload = to_float32_bytes(samples_float)

        packet = encode_packet(cmd, seq, payload)
        self.ser.write(packet)
        self.ser.flush()

        msg_type, rx_seq, rx_payload = self._read_packet()
        round_trip_ms = (time.perf_counter() - t_start) * 1000.0

        if msg_type == RSP_ERROR:
            raise RuntimeError(f"Firmware error response (code: 0x{rx_payload[2]:02X})")
        if msg_type != RSP_ESTIMATE:
            raise RuntimeError(f"Unexpected response 0x{msg_type:02X} for audio injection")

        result = unpack_estimate_result(rx_payload)
        result.round_trip_ms = round_trip_ms
        return result

    def _read_packet(self) -> Tuple[int, int, bytes]:
        """Reads and frames one complete response packet from USB serial."""
        # Find magic "HIL1"
        magic_match = b""
        start_time = time.time()
        while magic_match != b"HIL1":
            if time.time() - start_time > self.ser.timeout:
                raise TimeoutError("Timed out waiting for packet magic header")
            b = self.ser.read(1)
            if not b:
                continue
            if b == b"H":
                magic_match = b"H"
            elif magic_match == b"H" and b == b"I":
                magic_match = b"HI"
            elif magic_match == b"HI" and b == b"L":
                magic_match = b"HIL"
            elif magic_match == b"HIL" and b == b"1":
                magic_match = b"HIL1"
            else:
                magic_match = b""

        # Read remaining 5 bytes of header: msg_type (1), seq_id (2), payload_len (2)
        rest_header = self._read_exact(5)
        import struct
        msg_type, seq_id, payload_len = struct.unpack("<BHH", rest_header)

        # Read payload
        payload = self._read_exact(payload_len) if payload_len > 0 else b""

        # Read CRC16 (2 bytes)
        crc_bytes = self._read_exact(2)
        (rx_crc,) = struct.unpack("<H", crc_bytes)

        # Verify CRC
        full_packet_data = b"HIL1" + rest_header + payload
        from hil_protocol import compute_crc16
        computed_crc = compute_crc16(full_packet_data)
        if rx_crc != computed_crc:
            raise ValueError(f"CRC Mismatch: expected 0x{computed_crc:04X}, got 0x{rx_crc:04X}")

        return msg_type, seq_id, payload

    def _read_exact(self, num_bytes: int) -> bytes:
        data = bytearray()
        start = time.time()
        while len(data) < num_bytes:
            if time.time() - start > self.ser.timeout:
                raise TimeoutError(f"Timed out reading {num_bytes} bytes (received {len(data)})")
            chunk = self.ser.read(num_bytes - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)
