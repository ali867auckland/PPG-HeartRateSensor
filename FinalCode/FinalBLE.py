#!/usr/bin/env python3
"""
Industrial-grade PPG reader on Raspberry Pi Zero 2 W
- Reads raw PPG from MCP3008 via SPI
- Band-pass filters (0.5–5 Hz) and envelope-based peak detection
- Computes sliding-window BPM with outlier guard
- Robust logging, clean shutdown on Ctrl+C
- Publishes BPM to an MQTT broker (Mosquitto)
"""

import spidev
import time
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
import logging
import signal
from collections import deque

# === MQTT (Mosquitto) setup ===
import paho.mqtt.client as mqtt
MQTT_BROKER = "BROKER_IP_OR_HOSTNAME"   # ← replace with your broker’s IP (e.g. "192.168.1.42")
MQTT_PORT   = 1883
MQTT_TOPIC  = "home/ppg/bpm"

# === ADC / Sampling settings ===
SPI_BUS       = 0
SPI_DEVICE    = 0
PPG_CHANNEL   = 0
ADC_RES       = 1023.0
V_REF         = 3.3
SAMPLING_RATE = 100               # Hz
WINDOW_S      = 10                # seconds
BUFFER_SIZE   = SAMPLING_RATE * WINDOW_S

def read_adc(spi, channel):
    """Read raw ADC value from MCP3008."""
    # MCP3008 protocol: start bit, single-ended, channel
    cmd = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 3) << 8) | resp[2]

def voltage_from_adc(raw):
    """Convert raw ADC to voltage."""
    return (raw / ADC_RES) * V_REF

class PPGReader:
    def __init__(self):
        # --- SPI init ---
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1_350_000

        # --- Band-pass filter design ---
        nyq = 0.5 * SAMPLING_RATE
        low = 0.5 / nyq
        high = 5.0 / nyq
        self.b, self.a = butter(2, [low, high], btype='band')

        # --- Logging ---
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s [%(levelname)s] %(message)s")
        self.logger = logging.getLogger("PPGReader")

        # --- MQTT client ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()
        self.logger.info("MQTT connected to %s:%d, topic=%s",
                         MQTT_BROKER, MQTT_PORT, MQTT_TOPIC)

        # --- Buffers for sliding window ---
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        # --- Shutdown handler ---
        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)
        self.logger.info("PPGReader initialized (%.1f Hz window=%ds)",
                         SAMPLING_RATE, WINDOW_S)

    def _shutdown(self, signum, frame):
        self.logger.info("SIGINT received, shutting down…")
        self.running = False

    def sample_loop(self):
        next_time = time.time()
        while self.running:
            now = time.time()
            if now < next_time:
                time.sleep(next_time - now)
            ts = time.time()

            try:
                # --- Read & buffer raw PPG ---
                raw = read_adc(self.spi, PPG_CHANNEL)
                voltage = voltage_from_adc(raw)
                self.raw_buf.append(voltage)
                self.ts_buf.append(ts)

                # --- Filter signal ---
                raw_arr  = np.array(self.raw_buf, dtype=float)
                filt_arr = filtfilt(self.b, self.a, raw_arr)

                # --- Compute BPM ---
                bpm = self.compute_bpm(np.array(self.ts_buf), filt_arr)
                if bpm is not None:
                    self.logger.info("Current BPM: %.1f", bpm)  # :contentReference[oaicite:0]{index=0}
                    # --- Publish to MQTT ---
                    self.mqtt_client.publish(MQTT_TOPIC, f"{bpm:.1f}")
                else:
                    self.logger.warning("No valid BPM detected")

            except Exception as e:
                self.logger.error("Error in sample_loop: %s", e)

            next_time += 1.0 / SAMPLING_RATE

        self.cleanup()

    def compute_bpm(self, timestamps, signal_data):
        """
        Envelope-based peak detection:
        1. Compute signal envelope (abs + smoothing)
        2. Find peaks with min height & distance based on expected HR
        3. Compute intervals → BPM; require ≥2 peaks
        """
        # (Your existing implementation goes here…)
        # Example placeholder:
        peaks, _ = find_peaks(signal_data, distance=SAMPLING_RATE*0.5)
        if len(peaks) < 2:
            return None
        intervals = np.diff(timestamps[peaks])
        bpm = 60.0 / np.mean(intervals)
        return bpm

    def cleanup(self):
        try:
            self.spi.close()
        except Exception:
            pass
        self.logger.info("SPI closed, exiting.")

if __name__ == '__main__':
    reader = PPGReader()
    reader.sample_loop()
