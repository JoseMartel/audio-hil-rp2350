"""
Synthetic Audio Signal Generator for RP2350 HIL Testing.
Generates pure tones, multi-harmonic signals, AWGN noise, and transients.
"""

import numpy as np
from typing import Dict, Optional


def generate_pure_tone(
    freq_hz: float,
    sample_rate: int = 16000,
    num_samples: int = 2048,
    amplitude: float = 0.8,
    phase_rad: float = 0.0
) -> np.ndarray:
    """Generates a pure continuous sinusoidal tone."""
    t = np.arange(num_samples) / float(sample_rate)
    signal = amplitude * np.sin(2.0 * np.pi * freq_hz * t + phase_rad)
    return signal.astype(np.float32)


def generate_tone_with_awgn(
    freq_hz: float,
    snr_db: float,
    sample_rate: int = 16000,
    num_samples: int = 2048,
    amplitude: float = 0.8,
    seed: Optional[int] = None
) -> np.ndarray:
    """
    Generates a pure tone embedded in Additive White Gaussian Noise (AWGN)
    with a precise Signal-to-Noise Ratio (SNR) in dB.
    """
    if seed is not None:
        np.random.seed(seed)

    clean = generate_pure_tone(freq_hz, sample_rate, num_samples, amplitude)
    
    # Calculate signal power
    p_signal = np.mean(clean ** 2)
    # Target noise power based on SNR: SNR_dB = 10 * log10(P_signal / P_noise)
    p_noise = p_signal / (10.0 ** (snr_db / 10.0))
    noise_std = np.sqrt(p_noise)
    
    noise = np.random.normal(0.0, noise_std, num_samples).astype(np.float32)
    noisy_signal = clean + noise
    
    # Clip to avoid saturation
    noisy_signal = np.clip(noisy_signal, -1.0, 1.0)
    return noisy_signal.astype(np.float32)


def generate_harmonic_signal(
    f0_hz: float,
    harmonic_weights: Dict[int, float],
    sample_rate: int = 16000,
    num_samples: int = 2048,
    amplitude: float = 0.8
) -> np.ndarray:
    """
    Generates a complex periodic signal with fundamental frequency f0
    and harmonic components.
    Example: harmonic_weights = {1: 1.0, 2: 0.5, 3: 0.25}
    """
    t = np.arange(num_samples) / float(sample_rate)
    composite = np.zeros(num_samples, dtype=np.float32)

    total_weight = sum(harmonic_weights.values())
    for harmonic, weight in harmonic_weights.items():
        freq = f0_hz * harmonic
        if freq < sample_rate / 2.0:  # Below Nyquist
            composite += (weight / total_weight) * np.sin(2.0 * np.pi * freq * t)

    composite *= amplitude
    return composite.astype(np.float32)


def generate_frequency_step(
    f1_hz: float,
    f2_hz: float,
    step_sample: int = 1024,
    sample_rate: int = 16000,
    num_samples: int = 2048,
    amplitude: float = 0.8
) -> np.ndarray:
    """Generates an abrupt frequency change in the middle of the analysis window."""
    t1 = np.arange(0, step_sample) / float(sample_rate)
    t2 = np.arange(step_sample, num_samples) / float(sample_rate)

    part1 = amplitude * np.sin(2.0 * np.pi * f1_hz * t1)
    phase_offset = 2.0 * np.pi * f1_hz * (step_sample / float(sample_rate))
    part2 = amplitude * np.sin(2.0 * np.pi * f2_hz * (t2 - t1[-1]) + phase_offset)

    return np.concatenate([part1, part2]).astype(np.float32)


def generate_silence(num_samples: int = 2048) -> np.ndarray:
    """Generates a buffer of pure digital silence."""
    return np.zeros(num_samples, dtype=np.float32)


def to_pcm16_bytes(signal_float: np.ndarray) -> bytes:
    """Converts float [-1.0, 1.0] samples to 16-bit signed integer little-endian bytes."""
    clipped = np.clip(signal_float, -1.0, 1.0)
    pcm16 = (clipped * 32767.0).astype(np.int16)
    return pcm16.tobytes()


def to_float32_bytes(signal_float: np.ndarray) -> bytes:
    """Returns raw float32 IEEE-754 bytes."""
    return signal_float.astype(np.float32).tobytes()
