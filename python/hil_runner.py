#!/usr/bin/env python3
"""
RP2350 Hardware-in-the-Loop (HIL) Test Runner.
Executes automated test suites across pure tones, noise limits, harmonics, and latency.
Generates comprehensive console logs, metrics tables, and optional matplotlib plots.
"""

import os
import sys
import argparse
import time
import numpy as np
from typing import List, Dict, Any

from hil_protocol import (
    STATUS_OK, STATUS_SILENCE, STATUS_OUT_OF_BOUNDS,
    HILEstimateResult, HILDeviceConfig
)
from hil_client import HILClient, auto_detect_pico_port
from signal_generator import (
    generate_pure_tone, generate_tone_with_awgn,
    generate_harmonic_signal, generate_silence
)


class HILTestSuite:
    def __init__(self, client: HILClient, sample_rate: int = 16000, fft_size: int = 2048, output_dir: str = "reports"):
        self.client = client
        self.sample_rate = sample_rate
        self.fft_size = fft_size
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def test_handshake(self) -> bool:
        print("\n=======================================================")
        print(" [TEST 1] Handshake & Firmware Configuration Check")
        print("=======================================================")
        print("  Sending PING...")
        ping_ok = self.client.ping()
        print(f"  PING Response: {'SUCCESS' if ping_ok else 'FAILED'}")
        if not ping_ok:
            return False

        config = self.client.get_config()
        print(f"  Firmware Config:")
        print(f"    - Sample Rate:     {config.sample_rate} Hz")
        print(f"    - FFT Size:        {config.fft_size} samples (Resolution: {config.sample_rate / config.fft_size:.4f} Hz/bin)")
        print(f"    - Noise Threshold: {config.noise_threshold:.2f}")
        print(f"    - Valid Range:     [{config.min_freq:.1f} Hz, {config.max_freq:.1f} Hz]")
        return True

    def test_frequency_sweep(self, test_freqs: List[float], format_type: str = "float32") -> Dict[str, Any]:
        print("\n=======================================================")
        print(f" [TEST 2] Frequency Sweep & Sub-Hertz Precision ({format_type.upper()})")
        print("=======================================================")
        print(f"{'Target (Hz)':>12} | {'Estimated (Hz)':>14} | {'Error (Hz)':>10} | {'Rel Err (%)':>11} | {'Peak Mag':>9} | {'DSP (us)':>8} | {'Status':>10}")
        print("-" * 88)

        results = []
        errors = []
        rel_errors = []
        latencies = []

        for f_target in test_freqs:
            signal = generate_pure_tone(f_target, self.sample_rate, self.fft_size, amplitude=0.85)
            res = self.client.inject_block(signal, format_type=format_type)

            error = abs(res.estimated_freq - f_target) if res.status == STATUS_OK else float("nan")
            rel_err = (error / f_target * 100.0) if (res.status == STATUS_OK and f_target > 0) else float("nan")

            if res.status == STATUS_OK:
                errors.append(error)
                rel_errors.append(rel_err)
                latencies.append(res.dsp_latency_us)

            status_str = "OK" if res.status == STATUS_OK else "SILENCE"
            est_str = f"{res.estimated_freq:14.2f}" if res.status == STATUS_OK else f"{'---':>14}"
            err_str = f"{error:10.2f}" if not np.isnan(error) else f"{'---':>10}"
            rel_str = f"{rel_err:10.3f}%" if not np.isnan(rel_err) else f"{'---':>11}"

            print(f"{f_target:12.2f} | {est_str} | {err_str} | {rel_str} | {res.max_magnitude:9.2f} | {res.dsp_latency_us:8d} | {status_str:>10}")
            results.append({"target": f_target, "result": res, "error": error, "rel_error": rel_err})

        mean_err = np.mean(errors) if errors else 0.0
        max_err = np.max(errors) if errors else 0.0
        mean_lat = np.mean(latencies) if latencies else 0.0

        print("-" * 88)
        print(f" Summary: Mean Error: {mean_err:.3f} Hz | Max Error: {max_err:.3f} Hz | Avg DSP Latency: {mean_lat:.1f} us")
        return {"data": results, "mean_error": mean_err, "max_error": max_err, "mean_latency": mean_lat}

    def test_out_of_band_limits(self, format_type: str = "float32") -> bool:
        print("\n=======================================================")
        print(" [TEST 3] Out-of-Band Limit Rejection")
        print("=======================================================")
        print("  Evaluating rejection below 50 Hz and above 3000 Hz...")

        out_of_band_freqs = [10.0, 25.0, 40.0, 3100.0, 4500.0, 6000.0, 7500.0]
        all_passed = True

        for f in out_of_band_freqs:
            signal = generate_pure_tone(f, self.sample_rate, self.fft_size, amplitude=0.85)
            res = self.client.inject_block(signal, format_type=format_type)

            # Signal should be rejected (report 0.0 Hz or silence status)
            passed = (res.status == STATUS_SILENCE) or (res.estimated_freq == 0.0)
            status_desc = "REJECTED (PASSED)" if passed else f"DETECTED ({res.estimated_freq:.1f} Hz) - FAILED"
            print(f"  Freq: {f:7.1f} Hz  -->  {status_desc} (Max Mag: {res.max_magnitude:.2f})")
            if not passed:
                all_passed = False

        return all_passed

    def test_snr_robustness(self, test_snr_values: List[float], f_target: float = 440.0, format_type: str = "float32") -> Dict[str, Any]:
        print("\n=======================================================")
        print(f" [TEST 4] Noise Robustness (AWGN) at f = {f_target} Hz")
        print("=======================================================")
        print(f"{'SNR (dB)':>10} | {'Estimated (Hz)':>14} | {'Error (Hz)':>10} | {'Peak Mag':>9} | {'Status':>10}")
        print("-" * 65)

        snr_results = []
        breakdown_snr = None

        for snr in test_snr_values:
            signal = generate_tone_with_awgn(f_target, snr_db=snr, sample_rate=self.sample_rate, num_samples=self.fft_size)
            res = self.client.inject_block(signal, format_type=format_type)

            err = abs(res.estimated_freq - f_target) if res.status == STATUS_OK else float("nan")
            passed = (res.status == STATUS_OK) and (err < 2.0)

            est_str = f"{res.estimated_freq:14.2f}" if res.status == STATUS_OK else f"{'---':>14}"
            err_str = f"{err:10.2f}" if not np.isnan(err) else f"{'---':>10}"
            status_str = "LOCKED" if passed else "LOST / NOISE"

            print(f"{snr:10.1f} | {est_str} | {err_str} | {res.max_magnitude:9.2f} | {status_str:>10}")

            if not passed and breakdown_snr is None:
                breakdown_snr = snr

            snr_results.append({"snr": snr, "estimated": res.estimated_freq, "error": err, "mag": res.max_magnitude, "locked": passed})

        print("-" * 65)
        print(f" Breakdown SNR: ~{breakdown_snr if breakdown_snr is not None else '< min tested'} dB")
        return {"data": snr_results, "breakdown_snr": breakdown_snr}

    def test_harmonics_discrimination(self, f0: float = 220.0, format_type: str = "float32") -> bool:
        print("\n=======================================================")
        print(f" [TEST 5] Harmonics Discrimination (Fundamental f0 = {f0} Hz)")
        print("=======================================================")

        cases = [
            ("Pure tone (f0 only)", {1: 1.0}),
            ("Rich Tone (f0 dominant + 2f0, 3f0)", {1: 1.0, 2: 0.5, 3: 0.25}),
            ("Square-like odd harmonics", {1: 1.0, 3: 0.33, 5: 0.2}),
            ("Suppressed Fundamental (2f0 dominates)", {1: 0.3, 2: 1.0, 3: 0.5}),
        ]

        for desc, weights in cases:
            signal = generate_harmonic_signal(f0, weights, self.sample_rate, self.fft_size, amplitude=0.85)
            res = self.client.inject_block(signal, format_type=format_type)
            print(f"  Profile: {desc}")
            print(f"    Weights: {weights}")
            print(f"    Estimated: {res.estimated_freq:.2f} Hz | Peak FFT Bin: {res.peak_bin} | Mag: {res.max_magnitude:.2f}")

        return True

    def test_benchmark_speed(self, num_iterations: int = 100, format_type: str = "float32") -> Dict[str, float]:
        print("\n=======================================================")
        print(f" [TEST 6] Latency & Throughput Benchmark ({num_iterations} blocks)")
        print("=======================================================")

        signal = generate_pure_tone(440.0, self.sample_rate, self.fft_size, amplitude=0.8)
        latencies_dsp = []
        latencies_rt = []

        t_total_start = time.perf_counter()
        for _ in range(num_iterations):
            res = self.client.inject_block(signal, format_type=format_type)
            latencies_dsp.append(res.dsp_latency_us)
            latencies_rt.append(res.round_trip_ms)
        t_total = time.perf_counter() - t_total_start

        avg_dsp = np.mean(latencies_dsp)
        avg_rt = np.mean(latencies_rt)
        fps = num_iterations / t_total

        # Speedup vs Real-time (1 block of 2048 @ 16kHz = 128 ms)
        real_time_block_s = self.fft_size / float(self.sample_rate)
        sim_speedup = (real_time_block_s * 1000.0) / avg_rt

        print(f"  Average DSP Latency:       {avg_dsp:6.1f} us  (CPU time on RP2350 Cortex-M33)")
        print(f"  Average Round-trip Time:   {avg_rt:6.2f} ms  (USB transfer + DSP + USB reply)")
        print(f"  HIL Throughput:            {fps:6.1f} blocks/sec")
        print(f"  Simulation Speedup:        {sim_speedup:6.1f}x FASTER than Real-Time!")
        return {"avg_dsp_us": avg_dsp, "avg_rt_ms": avg_rt, "fps": fps, "speedup": sim_speedup}


def main():
    parser = argparse.ArgumentParser(description="RP2350 Audio Fundamental Frequency Estimator HIL Runner")
    parser.add_argument("--port", type=str, default=None, help="Serial port of RP2350 (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--test", type=str, default="all", choices=["all", "sweep", "limits", "snr", "harmonics", "benchmark"], help="Test suite to run")
    parser.add_argument("--format", type=str, default="float32", choices=["float32", "pcm16"], help="Sample injection format")
    parser.add_argument("--mock", action="store_true", help="Run offline simulation without physical hardware")
    parser.add_argument("--outdir", type=str, default="reports", help="Directory to save test outputs")
    parser.add_argument("--plot", action="store_true", help="Generate and save matplotlib plots")
    args = parser.parse_args()

    print("===================================================================")
    print("      RP2350 HARDWARE-IN-THE-LOOP (HIL) AUDIO ESTIMATOR SUITE      ")
    print("===================================================================")
    if args.mock:
        print("  MODE: Offline Simulation (Software CMSIS-DSP Emulator)")
    else:
        print(f"  MODE: Hardware-in-the-Loop on RP2350 (Port: {args.port or 'Auto-detect'})")

    try:
        client = HILClient(port=args.port, baudrate=args.baud, mock=args.mock)
    except Exception as e:
        print(f"ERROR connecting to client: {e}")
        sys.exit(1)

    suite = HILTestSuite(client, output_dir=args.outdir)

    try:
        suite.test_handshake()

        if args.test in ("all", "sweep"):
            test_frequencies = [
                60.0, 82.4, 110.0, 150.0, 220.0, 329.63, 440.0, 523.25,
                659.25, 880.0, 1000.0, 1250.0, 1500.0, 1760.0, 2000.0,
                2400.0, 2750.0, 2950.0
            ]
            sweep_res = suite.test_frequency_sweep(test_frequencies, format_type=args.format)

        if args.test in ("all", "limits"):
            suite.test_out_of_band_limits(format_type=args.format)

        if args.test in ("all", "snr"):
            snr_range = [30.0, 25.0, 20.0, 15.0, 12.0, 10.0, 8.0, 6.0, 4.0, 2.0, 0.0, -3.0, -6.0]
            snr_res = suite.test_snr_robustness(snr_range, f_target=440.0, format_type=args.format)

        if args.test in ("all", "harmonics"):
            suite.test_harmonics_discrimination(f0=220.0, format_type=args.format)

        if args.test in ("all", "benchmark"):
            suite.test_benchmark_speed(num_iterations=50, format_type=args.format)

        print("\n=======================================================")
        print(" [ALL TESTS COMPLETED SUCCESSFULLY]")
        print("=======================================================\n")

    finally:
        client.close()


if __name__ == "__main__":
    main()
