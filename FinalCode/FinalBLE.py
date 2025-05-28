#!/usr/bin/env python3
"""
Industrial-grade PPG reader on Raspberry Pi Zero 2 W
- Reads from MCP3008 over SPI
- Band-pass filters, detects peaks, computes sliding-window BPM
- Publishes every result to MQTT:
    • a float string "72.4" if valid
    • the string "invalid" if not enough peaks
"""

import spidev, time, logging, signal, sys
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
from collections import deque

import paho.mqtt.client as mqtt

# ─── MQTT ────────────────────────────────────────────────────────────
MQTT_BROKER = "BROKER_IP_OR_HOSTNAME"   # e.g. "192.168.1.42"
MQTT_PORT   = 1883
MQTT_TOPIC  = "home/ppg/bpm"

# ─── SAMPLING & FILTER ───────────────────────────────────────────────
SPI_BUS      = 0
SPI_DEVICE   = 0
PPG_CHANNEL  = 0
ADC_RES      = 1023.0
V_REF        = 3.3
SAMPLING_RATE= 100  # Hz
WINDOW_S     = 10   # seconds
BUFFER_SIZE  = SAMPLING_RATE * WINDOW_S

def read_adc(spi, ch):
    cmd  = [1, (8+ch)<<4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1]&3)<<8) | resp[2]

def voltage(raw):
    return (raw/ADC_RES)*V_REF

def make_filter(sr, low, high, order=3):
    nyq = 0.5*sr
    b, a = butter(order, [low/nyq, high/nyq], btype='band')
    return b, a

# ─── LOGGER ───────────────────────────────────────────────────────────
logging.basicConfig(
    level    = logging.INFO,
    format   = "%(asctime)s [%(levelname)s] %(message)s",
    handlers = [logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger("PPG")

class PPGReader:
    def __init__(self):
        # SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1_350_000

        # Filter
        self.b, self.a = make_filter(SAMPLING_RATE, 0.5, 5.0, order=3)

        # Buffers
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        # MQTT
        self.mqtt = mqtt.Client()
        self.mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt.loop_start()
        log.info("MQTT connected to %s:%d, topic=%s",
                 MQTT_BROKER, MQTT_PORT, MQTT_TOPIC)

        # Shutdown
        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)
        log.info("PPGReader init: %d Hz, %ds window", SAMPLING_RATE, WINDOW_S)

    def _shutdown(self, sig, frame):
        log.info("Shutdown requested")
        self.running = False

    def sample_loop(self):
        next_time = time.time()
        while self.running:
            now = time.time()
            if now < next_time:
                time.sleep(next_time-now)
            ts = time.time()

            raw = read_adc(self.spi, PPG_CHANNEL)
            self.raw_buf.append(voltage(raw))
            self.ts_buf.append(ts)

            if len(self.raw_buf)==BUFFER_SIZE:
                sig = np.array(self.raw_buf)
                filt = filtfilt(self.b, self.a, sig)
                bpm = self.compute_bpm(np.array(self.ts_buf), filt)

                if bpm is not None:
                    msg = f"{bpm:.1f}"
                    log.info("Current BPM: %s", msg)
                else:
                    msg = "invalid"
                    log.warning("Unable to detect BPM (not enough peaks)")

                # publish *every* cycle
                try:
                    self.mqtt.publish(MQTT_TOPIC, msg)
                except Exception as e:
                    log.error("MQTT publish error: %s", e)

            next_time += 1.0/SAMPLING_RATE

        self.cleanup()

    def compute_bpm(self, timestamps, signal_data):
        env = np.abs(signal_data - np.mean(signal_data))
        thresh = np.mean(env) + 0.5*np.std(env)
        min_d   = int(0.5*SAMPLING_RATE)
        peaks, _ = find_peaks(env, distance=min_d, height=thresh)
        if len(peaks)<8:
            return None
        intervals = np.diff(timestamps[peaks])
        return 60.0/np.mean(intervals)

    def cleanup(self):
        try: self.spi.close()
        except: pass
        log.info("SPI closed, exiting")

if __name__=='__main__':
    PPGReader().sample_loop()
