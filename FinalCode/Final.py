#!/usr/bin/env python3
"""
Industrial-grade PPG reader on Raspberry Pi Zero 2 W
- Reads from MCP3008 ADC via SPI
- Band-pass filters raw data (0.5–5 Hz)
- Envelope-based, noise-robust peak detection
- Computes BPM in sliding window with minimum peak count guard
- Robust logging and clean shutdown
"""

import spidev
import time
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
import logging
import signal
import sys
from collections import deque

# === CONFIGURATION ===
SPI_BUS       = 0           # SPI bus (0 or 1)
SPI_DEVICE    = 0           # SPI device (CS0 or CS1)
PPG_CHANNEL   = 0           # MCP3008 channel where PPG is wired
V_REF         = 3.3         # ADC reference voltage
ADC_RES       = 1023.0      # 10-bit MCP3008 → 0–1023

SAMPLING_RATE = 100         # Hz
WINDOW_S      = 10          # seconds per BPM update
BUFFER_SIZE   = SAMPLING_RATE * WINDOW_S

# Band-pass filter design
LOWCUT        = 0.5         # Hz
HIGHCUT       = 5.0         # Hz
FILTER_ORDER  = 3

LOG_FILE      = 'ppg_bpm.log'

# === SETUP LOGGING ===
logging.basicConfig(
    level    = logging.INFO,
    format   = '%(asctime)s [%(levelname)s] %(message)s',
    handlers = [
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.getLogger('PPG')

# === HELPER FUNCTIONS ===
def make_filter(samplerate, lowcut, highcut, order=3):
    nyq = 0.5 * samplerate
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def read_adc(spi, channel):
    """
    Read raw ADC value (0–1023) from MCP3008 channel.
    """
    cmd = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]


def voltage_from_adc(adc_val):
    return (adc_val / ADC_RES) * V_REF

# === MAIN READER CLASS ===
class PPGReader:
    def __init__(self):
        # SPI setup
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1350000

        # filter coefficients
        self.b, self.a = make_filter(SAMPLING_RATE, LOWCUT, HIGHCUT, FILTER_ORDER)

        # circular buffer for raw and timestamps
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)

        log.info("PPGReader initialized. Sampling at %d Hz, window=%ds.", SAMPLING_RATE, WINDOW_S)

    def _shutdown(self, signum, frame):
        log.info("Shutdown signal received, cleaning up...")
        self.running = False

    def sample_loop(self):
        """
        Main loop: read, filter, and compute BPM.
        """
        next_time = time.time()
        while self.running:
            now = time.time()
            if now < next_time:
                time.sleep(next_time - now)
            ts = time.time()

            try:
                raw = read_adc(self.spi, PPG_CHANNEL)
            except Exception as e:
                log.error("SPI read error: %s", e)
                continue

            self.raw_buf.append(raw)
            self.ts_buf.append(ts)

            # if buffer is full, apply filter & compute BPM
            if len(self.raw_buf) == BUFFER_SIZE:
                raw_arr = np.array(self.raw_buf, dtype=float)
                filt_arr = filtfilt(self.b, self.a, raw_arr)

                bpm = self.compute_bpm(np.array(self.ts_buf), filt_arr)
                if bpm is not None:
                    log.info("Current BPM: %.1f", bpm)
                else:
                    log.warning("Unable to detect BPM (not enough valid peaks)")

            next_time += 1.0 / SAMPLING_RATE

        self.cleanup()

    def compute_bpm(self, timestamps, signal_data):
        """
        Envelope-based peak detection:
        - Compute signal envelope
        - Dynamic height threshold
        - Minimum peak count guard
        """
        # envelope around mean
        env = np.abs(signal_data - np.mean(signal_data))

        # dynamic height: mean + 0.5 * std
        height = np.mean(env) + 0.5 * np.std(env)
        min_distance = int(0.5 * SAMPLING_RATE)

        peaks, _ = find_peaks(
            env,
            distance=min_distance,
            height=height
        )
        log.debug("Envelope peaks detected: %d", len(peaks))

        # require at least 8 peaks in window to be valid
        if len(peaks) < 8:
            return None

        # compute intervals and BPM
        peak_ts = timestamps[peaks]
        intervals = np.diff(peak_ts)
        return 60.0 / np.mean(intervals)

    def cleanup(self):
        """
        Graceful cleanup on exit.
        """
        try:
            self.spi.close()
        except Exception:
            pass
        log.info("SPI closed. Exiting.")

# === ENTRY POINT ===
if __name__ == '__main__':
    reader = PPGReader()
    reader.sample_loop()
