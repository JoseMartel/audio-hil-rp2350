# RP2350 Audio Hardware-in-the-Loop (HIL) Frequency Estimator

An automated, high-precision **Hardware-in-the-Loop (HIL)** validation testbed for audio digital signal processing (DSP) on the **Raspberry Pi Pico 2 (RP2350 ARM Cortex-M33)**.

Instead of relying on physical acoustic speakers and I2S microphones (which introduce room reverberations, ambient noise, and inconsistent test conditions), this architecture injects synthetic audio directly into the RP2350 firmware via **Native USB CDC (Virtual COM Port)**. The onboard CMSIS-DSP pipeline executes in real hardware, measuring DSP execution latency and returning frequency estimates to the host test runner.

---

## Key Features

- **Pristine Deterministic Testing:** Eliminates analog acoustics and transducers; tests DSP algorithms with mathematically generated signals (pure tones, multi-harmonic signals, AWGN noise, transients).
- **Sub-Hertz Fundamental Estimation:** Combines Hann windowing, 2048-point Real FFT (`arm_rfft_fast_f32`), and parabolic sub-bin peak interpolation to achieve **< 0.25 Hz mean error** across the 50 Hz – 3000 Hz band (native bin resolution: 7.8125 Hz).
- **Native USB CDC Framing:** Custom binary protocol (`HIL1`) with 16-bit CCITT CRC checksum validation and disabled CR/LF translations for zero byte corruption.
- **Hardware-Decoupled Firmware Architecture:** Clean abstraction layers (`hil_hal`, `dsp_processor`, `frequency_estimator`) allow swapping the synthetic USB input with an actual I2S microphone driver without touching DSP kernels.
- **Automated Python Test Harness:** Includes 6 comprehensive test suites (Handshake, Sub-Hertz Sweep, Out-of-Band Limit Rejection, SNR/AWGN Robustness, Harmonics Discrimination, and Latency Benchmarks).
- **Dual-Mode Execution:** Supports both physical hardware execution on connected RP2350 boards and an offline software simulation mode (`--mock`) for CI/CD environments.

---

## System Architecture

```
+-------------------------------------------------------------------------------+
|                               HOST PC (Python)                                |
|                                                                               |
|  +------------------------+   +-------------------+   +--------------------+  |
|  |   signal_generator.py  |-->|   hil_client.py   |<--|   hil_runner.py    |  |
|  | (Tones, AWGN, Harmonics)   | (Port Auto-Detect)|   | (Automated Suites) |  |
|  +------------------------+   +---------+---------+   +--------------------+  |
|                                         |                                     |
|                                  [HIL1 Protocol]                              |
|                              (Magic, CRC16, Payloads)                         |
+-----------------------------------------|-------------------------------------+
                                          | USB CDC (ACM)
+-----------------------------------------|-------------------------------------+
|                             RP2350 (Cortex-M33)                               |
|                                         v                                     |
|                       +-----------------------------------+                   |
|                       |  hil_hal.c (USB CDC + CRC16 FSM)  |                   |
|                       +-----------------+-----------------+                   |
|                                         | float[] [-1.0, 1.0]                 |
|                       +-----------------v-----------------+                   |
|                       | frequency_estimator.c (Coordinator)                   |
|                       +-----------------+-----------------+                   |
|                                         |                                     |
|                       +-----------------v-----------------+                   |
|                       | dsp_processor.c (CMSIS-DSP Core)  |                   |
|                       |  - Hann Window                    |                   |
|                       |  - arm_rfft_fast_f32 (N = 2048)   |                   |
|                       |  - Spectral Peak Search           |                   |
|                       |  - Parabolic Interpolation        |                   |
|                       +-----------------+-----------------+                   |
|                                         | (Est Freq, Latency, Mag)            |
|                       +-----------------v-----------------+                   |
|                       |        hil_hal_send_estimate      |                   |
+-------------------------------------------------------------------------------+
```

---

## HIL1 Binary Protocol Specification

Communication occurs over USB CDC using structured binary packets. The RP2350 disables stdio CR/LF translations (`stdio_set_translate_crlf(&stdio_usb, false)`) to prevent `0x0A` (`\n`) from mutating into `0x0D 0x0A`.

### Packet Frame Layout

| Field | Type | Size | Description |
| :--- | :--- | :--- | :--- |
| `magic` | `uint8_t[4]` | 4 bytes | Synchronization header: ASCII `"HIL1"` (`0x48, 0x49, 0x4C, 0x31`) |
| `msg_type` | `uint8_t` | 1 byte | Command or response message identifier |
| `seq_id` | `uint16_t` | 2 bytes | Monotonic sequence number (Little-Endian) |
| `payload_len`| `uint16_t` | 2 bytes | Length of the subsequent payload $N$ in bytes (Little-Endian) |
| `payload` | `uint8_t[N]` | $N$ bytes | Variable command/response data |
| `crc16` | `uint16_t` | 2 bytes | CRC-16-CCITT (poly `0x1021`, init `0xFFFF`) over Header + Payload |

### Message Identifiers

| ID | Name | Direction | Description |
| :---: | :--- | :---: | :--- |
| `0x01` | `HIL_CMD_PING` | Host $\to$ Dev | Ping packet for link validation (echoed back) |
| `0x02` | `HIL_CMD_GET_CONFIG` | Host $\to$ Dev | Request current sampling rate, FFT size, and thresholds |
| `0x03` | `HIL_CMD_SET_CONFIG` | Host $\to$ Dev | Dynamically adjust noise floor threshold and valid frequency bounds |
| `0x10` | `HIL_CMD_INJECT_PCM16` | Host $\to$ Dev | Inject 2048 signed 16-bit integer samples (4096 bytes) |
| `0x11` | `HIL_CMD_INJECT_FLOAT32`| Host $\to$ Dev | Inject 2048 IEEE-754 32-bit float samples (8192 bytes) |
| `0x20` | `HIL_CMD_RESET` | Host $\to$ Dev | Reset parser state machine and internal buffers |
| `0x80` | `HIL_RSP_ACK` | Dev $\to$ Host | Command acknowledgment |
| `0x81` | `HIL_RSP_PONG` | Dev $\to$ Host | Response to `HIL_CMD_PING` |
| `0x82` | `HIL_RSP_CONFIG` | Dev $\to$ Host | Configuration structure payload |
| `0x90` | `HIL_RSP_ESTIMATE` | Dev $\to$ Host | Frequency estimate, peak magnitude, bin index, and DSP latency |
| `0xFF` | `HIL_RSP_ERROR` | Dev $\to$ Host | Error indicator (CRC mismatch, length error, buffer overflow) |

---

## Directory Structure

```
audio-hil-rp2350/
├── CMakeLists.txt                 # CMake configuration for RP2350 with CMSIS-DSP
├── pico_sdk_import.cmake          # Pico SDK import script
├── .gitignore                     # Git ignore rules for build artifacts
├── main.c                         # Firmware entry point and main event loop
├── components/
│   ├── include/
│   │   ├── protocol.h             # HIL1 packet structs, command IDs, inline CRC16
│   │   ├── dsp_processor.h        # CMSIS-DSP frequency estimator API
│   │   ├── hil_hal.h              # USB CDC streaming abstraction API
│   │   └── frequency_estimator.h  # HIL execution coordinator
│   └── src/
│       ├── dsp_processor.c        # RFFT, Hann window, peak finding & parabolic interpolation
│       ├── hil_hal.c              # 8-state FSM byte stream parser & USB CDC HAL
│       └── frequency_estimator.c  # Timing profiler and response dispatcher
└── python/
    ├── requirements.txt           # Python dependencies (pyserial, numpy, scipy, matplotlib)
    ├── hil_protocol.py            # Protocol packet encoder, decoder & CRC calculator
    ├── signal_generator.py        # Synthetic tone, AWGN, and harmonic signal generator
    ├── hil_client.py              # Serial communication client & mock software simulator
    └── hil_runner.py              # Automated test runner with 6 test batteries
```

---

## Hardware & Software Requirements

### Hardware
- **Raspberry Pi Pico 2** (RP2350 ARM Cortex-M33).
- **USB Cable** (Micro-USB or USB-C depending on the board) connected to the host PC.
- *No breadboard, microphones, or audio wiring required.*

### Host Software
- **Pico SDK v2.0+** and **ARM GCC Toolchain** (`arm-none-eabi-gcc`).
- **CMSIS-DSP Library** (v1.16+).
- **Python 3.8+** with dependencies:
  ```bash
  pip install -r python/requirements.txt
  ```

---

## Building and Flashing

### 1. Build Firmware
Export the required environment variables pointing to your local SDK paths:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
export CMSIS_DSP_PATH=/path/to/CMSIS-DSP

mkdir -p build && cd build
cmake ..
make -j$(nproc)
```
This generates `audio_hil.uf2`, `audio_hil.elf`, and `audio_hil.bin`.

### 2. Flash to Pico 2
1. Hold down the **`BOOTSEL`** button on your Pico 2 while plugging it into the host PC via USB.
2. The board mounts as a mass-storage drive (e.g., `/media/$USER/RP2350/`).
3. Copy the UF2 binary:
   ```bash
   cp build/audio_hil.uf2 /media/$USER/RP2350/
   ```
4. The board automatically reboots into the HIL firmware and exposes a CDC serial interface (typically `/dev/ttyACM0` on Linux).

---

## Running the Automated Test Suites

### 1. Complete Test Suite on Hardware
Run all 6 test suites against the connected RP2350:
```bash
python3 python/hil_runner.py --test all
```
*Note: The runner auto-detects Raspberry Pi Pico VID `0x2E8A`. You can also specify the port manually with `--port /dev/ttyACM0`.*

### 2. Available Test Suites
You can run individual test batteries via the `--test` flag:

- **Sub-Hertz Frequency Sweep:**
  ```bash
  python3 python/hil_runner.py --test sweep --format float32
  ```
- **Out-of-Band Limit Rejection (< 50 Hz, > 3000 Hz):**
  ```bash
  python3 python/hil_runner.py --test limits
  ```
- **Noise Robustness (AWGN SNR Sweep):**
  ```bash
  python3 python/hil_runner.py --test snr
  ```
- **Harmonic Discrimination:**
  ```bash
  python3 python/hil_runner.py --test harmonics
  ```
- **Throughput & Latency Benchmark:**
  ```bash
  python3 python/hil_runner.py --test benchmark
  ```

### 3. Offline Software Simulation (No Hardware Needed)
For headless CI/CD pipelines or environments without physical hardware:
```bash
python3 python/hil_runner.py --mock --test all
```

---

## Empirical Hardware Benchmark Results

The following metrics were obtained directly from a physical **Raspberry Pi Pico 2 (RP2350 Cortex-M33 @ 150 MHz)** running `audio_hil`:

### Sub-Hertz Sweep (Selected Results)

| Target Freq | Estimated Freq | Absolute Error | Relative Error | Peak Mag | DSP Latency | Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **60.00 Hz** | 60.41 Hz | 0.41 Hz | 0.685% | 407.05 | 1913 µs | OK |
| **110.00 Hz** | 109.85 Hz | 0.15 Hz | 0.139% | 433.20 | 1866 µs | OK |
| **440.00 Hz** | 439.59 Hz | 0.41 Hz | 0.093% | 407.04 | 1853 µs | OK |
| **523.25 Hz** | 523.30 Hz | **0.05 Hz** | **0.009%** | 434.83 | 1846 µs | OK |
| **1000.00 Hz** | 1000.00 Hz | **0.00 Hz** | **0.000%** | 434.99 | 1860 µs | OK |
| **2000.00 Hz** | 2000.00 Hz | **0.00 Hz** | **0.000%** | 434.99 | 1842 µs | OK |
| **2950.00 Hz** | 2950.33 Hz | 0.33 Hz | 0.011% | 391.96 | 1857 µs | OK |

- **Mean Estimation Error:** **0.223 Hz**
- **Maximum Estimation Error:** **0.411 Hz**

### Out-of-Band Limit Rejection
- Signals injected below $50\text{ Hz}$ ($10\text{ Hz}, 25\text{ Hz}, 40\text{ Hz}$) and above $3000\text{ Hz}$ ($3100\text{ Hz}, 4500\text{ Hz}, 6000\text{ Hz}, 7500\text{ Hz}$) are cleanly identified as out-of-bounds/silence and rejected ($0.0\text{ Hz}$).

### Noise Robustness (AWGN)
- Frequency lock maintained at $440.0\text{ Hz}$ with sub-Hertz accuracy down to **$\text{SNR} = -6\text{ dB}$**.

### Real-Time Benchmark
- **Core DSP Calculation Time:** $\approx 1.85\text{ ms}$ (Hann window + CMSIS-DSP RFFT + peak interpolation on RP2350).
- **Round-Trip Latency:** $\approx 48.1\text{ ms}$ (USB full-speed transfer of 8192-byte blocks + DSP + USB response).
- **HIL Throughput:** **20.8 blocks/sec**.
- **Simulation Speed:** **2.7x FASTER than Real-Time audio playback** (each 2048-sample block represents 128 ms of audio).
