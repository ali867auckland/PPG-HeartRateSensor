#!/usr/bin/env python3
"""
Unified PPG + Polar H10 comparison with noise‐robust beat detection
- PPG via MCP3008 → 0.5–5 Hz band-pass filter → envelope detection → peak detection → sliding-window BPM
- Polar H10 over BLE → real-time BPM notifications
- Thread-safe sharing, CSV + console logging
- Clean shutdown on Ctrl-C
"""

import spidev
import time
import logging
import signal
import sys
import threading
import asyncio
import csv
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
from collections import deque
from bleak import BleakClient

# === CONFIGURATION ===
SPI_BUS      = 0
SPI_DEVICE   = 0
PPG_CHANNEL  = 0
V_REF        = 3.3
ADC_RES      = 1023.0

FS           = 100          # sampling rate (Hz)
WINDOW_S     = 10           # seconds of data per BPM update
BUFFER_SIZE  = FS * WINDOW_S

LOWCUT       = 0.5          # filter band (Hz)
HIGHCUT      = 5.0
FILTER_ORDER = 3

POLAR_MAC    = "XX:XX:XX:XX:XX:XX"  # replace with your strap’s MAC
HR_UUID      = "00002a37-0000-1000-8000-00805f9b34fb"

LOG_FILE     = "ppg_polar.log"
CSV_FILE     = f"bpm_compare_{time.strftime('%Y%m%d_%H%M%S')}.csv"

# === THREAD-SAFE GLOBALS ===
data_lock   = threading.Lock()
ppg_bpm     = None
polar_bpm   = None

# === LOGGING SETUP ===
logging.basicConfig(
    level    = logging.INFO,
    format   = "%(asctime)s [%(levelname)s] %(message)s",
    handlers = [
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.getLogger("PPG_Polar")

# === UTILITIES ===
def make_filter(fs, low, high, order):
    nyq = 0.5 * fs
    return butter(order, [low/nyq, high/nyq], btype="band")

def read_adc(spi, channel):
    cmd  = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

def parse_ble_hr(data: bytearray) -> int:
    flag = data[0]
    if flag & 0x01:
        return int.from_bytes(data[1:3], byteorder="little")
    return data[1]

# === MAIN CLASS ===
class PPGPolar:
    def __init__(self):
        # SPI + filter design
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1_350_000
        self.b, self.a = make_filter(FS, LOWCUT, HIGHCUT, FILTER_ORDER)

        # sliding buffers for raw & timestamps
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        # CSV logging
        self.csv_f = open(CSV_FILE, "w", newline="")
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(["Timestamp","PPG_BPM","Polar_BPM","Difference"])

        # control flag
        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)

        log.info("PPGPolar initialized: FS=%dHz, window=%ds", FS, WINDOW_S)

    def _shutdown(self, *args):
        log.info("Shutdown requested")
        self.running = False

    def compute_bpm(self, timestamps: np.ndarray, signal_data: np.ndarray):
        """
        Envelope-based peak detection with dynamic height threshold
        and a minimum peak count guard.
        """
        # remove DC and get envelope
        env = np.abs(signal_data - np.mean(signal_data))

        # dynamic height: mean + 0.5 * std
        height = np.mean(env) + 0.5 * np.std(env)
        min_dist = int(0.5 * FS)

        peaks, _ = find_peaks(env, distance=min_dist, height=height)
        log.debug("Envelope peaks detected: %d", len(peaks))

        # require at least 8 peaks in WINDOW_S to trust
        if len(peaks) < 8:
            return None

        peak_ts = timestamps[peaks]
        intervals = np.diff(peak_ts)
        return 60.0 / np.mean(intervals)

    def _polar_task(self):
        """Async BLE loop in a separate thread."""
        async def runner():
            global polar_bpm
            try:
                async with BleakClient(POLAR_MAC) as client:
                    log.info("Connected to Polar H10 (%s)", POLAR_MAC)
                    await client.start_notify(HR_UUID, self._on_polar)
                    while self.running:
                        await asyncio.sleep(1.0)
            except Exception as e:
                log.error("Polar BLE error: %s", e)
        asyncio.run(runner())

    def _on_polar(self, _, data: bytearray):
        hr = parse_ble_hr(data)
        with data_lock:
            global polar_bpm
            polar_bpm = hr
        log.debug("Polar BPM: %d", hr)

    def start(self):
        # start Polar BLE thread
        threading.Thread(target=self._polar_task, daemon=True).start()
        # start PPG sampling loop
        self.sample_loop()

    def sample_loop(self):
        global ppg_bpm, polar_bpm
        next_t = time.time()

        while self.running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            ts = time.time()

            # read PPG and buffer
            try:
                raw = read_adc(self.spi, PPG_CHANNEL)
            except Exception as e:
                log.error("SPI read error: %s", e)
                continue

            self.raw_buf.append(raw)
            self.ts_buf.append(ts)

            # when window full, filter & compute
            if len(self.raw_buf) == BUFFER_SIZE:
                raw_arr  = np.array(self.raw_buf, dtype=float)
                filt_arr = filtfilt(self.b, self.a, raw_arr)

                bpm = self.compute_bpm(np.array(self.ts_buf), filt_arr)

                with data_lock:
                    ppg_bpm = bpm
                    pol    = polar_bpm

                diff = abs(bpm - pol) if (bpm is not None and pol is not None) else None

                # write CSV
                tstr = time.strftime("%Y-%m-%d %H:%M:%S")
                self.csv_w.writerow([
                    tstr,
                    f"{bpm:.1f}" if bpm else "",
                    f"{pol}"     if pol else "",
                    f"{diff:.1f}" if diff else ""
                ])
                self.csv_f.flush()

                # console log
                if bpm is not None:
                    log.info(f"PPG BPM={bpm:.1f} | Polar BPM={pol or '–'} | Δ={diff or '–'}")
                else:
                    log.warning("PPG: no valid peaks (sensor off-finger or too noisy)")

            next_t += 1.0 / FS

        self.cleanup()

    def cleanup(self):
        try: self.spi.close()
        except: pass
        try: self.csv_f.close()
        except: pass
        log.info("Cleaned up and exiting.")

if __name__ == "__main__":
    PPGPolar().start()
