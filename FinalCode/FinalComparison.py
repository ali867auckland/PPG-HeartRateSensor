#!/usr/bin/env python3
"""
Industrial-grade PPG + Polar H10 comparison with live plotting
- PPG via MCP3008 → band-pass filter → sliding-window BPM
- Polar H10 over BLE → real-time BPM notifications
- Thread-safe deques for live matplotlib animation
- CSV + file logging
- Clean shutdown on Ctrl-C
"""

import spidev
import time
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
import logging
import signal
import sys
import threading
import asyncio
from bleak import BleakClient
import csv
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === CONFIGURATION ===
SPI_BUS       = 0
SPI_DEVICE    = 0
PPG_CHANNEL   = 0
V_REF         = 3.3
ADC_RES       = 1023.0
SAMPLING_RATE = 100      # Hz
WINDOW_S      = 10       # seconds per BPM update
BUFFER_SIZE   = SAMPLING_RATE * WINDOW_S

LOWCUT        = 0.5
HIGHCUT       = 5.0
FILTER_ORDER  = 3

POLAR_MAC     = "XX:XX:XX:XX:XX:XX"
HR_UUID       = "00002a37-0000-1000-8000-00805f9b34fb"

LOG_FILE      = 'ppg_polar_plot.log'
CSV_FILE      = f"bpm_compare_{time.strftime('%Y%m%d_%H%M%S')}.csv"

# Thread‐safe globals
data_lock   = threading.Lock()
ppg_bpm     = None
polar_bpm   = None

# Deques for plotting
max_len     = 100
ppg_vals    = deque([0]*max_len, maxlen=max_len)
polar_vals  = deque([0]*max_len, maxlen=max_len)
diff_vals   = deque([0]*max_len, maxlen=max_len)

# === LOGGING SETUP ===
logging.basicConfig(
    level    = logging.INFO,
    format   = '%(asctime)s [%(levelname)s] %(message)s',
    handlers = [
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.getLogger('PPG_Plot')

def make_filter(sr, low, high, order):
    nyq = 0.5 * sr
    return butter(order, [low/nyq, high/nyq], btype='band')

def read_adc(spi, channel):
    cmd  = [1, (8+channel)<<4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1]&0x03)<<8) | resp[2]

def parse_ble_hr(data: bytearray) -> int:
    flag = data[0]
    if flag & 0x01:
        return int.from_bytes(data[1:3], 'little')
    return data[1]

class PPGPolar:
    def __init__(self):
        # SPI + filter
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1_350_000
        self.b, self.a = make_filter(SAMPLING_RATE, LOWCUT, HIGHCUT, FILTER_ORDER)

        # sliding buffers
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        # CSV
        self.csv_f  = open(CSV_FILE, 'w', newline='')
        self.csv_w  = csv.writer(self.csv_f)
        self.csv_w.writerow(['Timestamp','PPG_BPM','Polar_BPM','Difference'])

        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)
        log.info("Initialized PPGPolar reader")

    def _shutdown(self, *args):
        log.info("Shutdown requested")
        self.running = False

    def compute_bpm(self, ts_arr, sig_arr):
        peaks, _ = find_peaks(sig_arr, distance=int(0.5*SAMPLING_RATE), prominence=0.1)
        if len(peaks) < 2:
            return None
        intervals = np.diff(ts_arr[peaks])
        return 60.0/np.mean(intervals)

    def _ble_task(self):
        """BLE in executor to avoid blocking asyncio loop."""
        async def run():
            global polar_bpm
            try:
                async with BleakClient(POLAR_MAC) as client:
                    log.info("Polar H10 connected")
                    await client.start_notify(HR_UUID, self._on_polar)
                    while self.running:
                        await asyncio.sleep(1)
            except Exception as e:
                log.error("BLE error: %s", e)
        asyncio.run(run())

    def _on_polar(self, handle, data):
        hr = parse_ble_hr(data)
        with data_lock:
            global polar_bpm
            polar_bpm = hr
        log.debug("Polar BPM: %d", hr)

    def start(self):
        # start BLE thread
        threading.Thread(target=self._ble_task, daemon=True).start()
        # start PPG sampling thread
        threading.Thread(target=self._ppg_loop, daemon=True).start()

    def _ppg_loop(self):
        """Continuously sample PPG, filter & compute BPM."""
        global ppg_bpm
        next_t = time.time()
        while self.running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t-now)
            ts = time.time()
            try:
                raw = read_adc(self.spi, PPG_CHANNEL)
            except Exception as e:
                log.error("SPI error: %s", e)
                continue
            self.raw_buf.append(raw)
            self.ts_buf.append(ts)

            if len(self.raw_buf) == BUFFER_SIZE:
                raw_arr  = np.array(self.raw_buf, float)
                filt_arr = filtfilt(self.b, self.a, raw_arr)
                bpm = self.compute_bpm(np.array(self.ts_buf), filt_arr)
                with data_lock:
                    ppg_bpm = bpm
                # log CSV
                with data_lock:
                    pol = polar_bpm
                diff = abs(bpm-pol) if (bpm and pol) else None
                self.csv_w.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    f"{bpm:.1f}" if bpm else '',
                    f"{pol}" if pol else '',
                    f"{diff:.1f}" if diff else ''
                ])
                self.csv_f.flush()
                # console log
                if bpm:
                    log.info(f"PPG={bpm:.1f} Polar={pol or '–'} Δ={diff or '–'}")
                else:
                    log.warning("No PPG peaks")
            next_t += 1.0/SAMPLING_RATE

        self.cleanup()

    def cleanup(self):
        try: self.spi.close()
        except: pass
        try: self.csv_f.close()
        except: pass
        log.info("Cleaned up")

def update_plot(frame):
    with data_lock:
        ppg_vals.append(ppg_bpm or 0)
        polar_vals.append(polar_bpm or 0)
        diff_vals.append(abs((ppg_bpm or 0) - (polar_bpm or 0)))
    line1.set_data(range(max_len), list(ppg_vals))
    line2.set_data(range(max_len), list(polar_vals))
    line3.set_data(range(max_len), list(diff_vals))
    ax.relim(); ax.autoscale_view()
    return line1, line2, line3

if __name__ == '__main__':
    reader = PPGPolar()
    reader.start()

    # === Set up live plot in main thread ===
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], label='PPG BPM')
    line2, = ax.plot([], [], label='Polar BPM')
    line3, = ax.plot([], [], label='Difference')
    ax.set_xlabel('Samples'); ax.set_ylabel('BPM')
    ax.set_title('Live PPG vs Polar H10')
    ax.legend()

    ani = animation.FuncAnimation(fig, update_plot, interval=200)
    plt.tight_layout()
    plt.show()

    # when user closes plot window, trigger shutdown
    reader.running = False
