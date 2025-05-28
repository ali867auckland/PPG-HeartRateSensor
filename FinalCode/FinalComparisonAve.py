#!/usr/bin/env python3
"""
Unified PPG + Polar H10 comparison with decoupled sampling and CSV summary
- Dedicated sampler thread reads MCP3008 at 100 Hz into a queue
- Processor thread filters 10 s windows, detects peaks, computes sliding-window BPM
- Polar H10 BLE client in separate thread
- Live plotting of PPG BPM, Polar BPM, and difference
- CSV + console logging; post-run summary of mean difference vs. difference of means stored in CSV
- Clean, low-jitter sampling unaffected by downstream I/O or plotting
"""

import spidev, time, logging, signal, sys, threading, asyncio, csv, queue
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
from collections import deque
from bleak import BleakClient
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === CONFIGURATION ===
SPI_BUS      = 0
SPI_DEVICE   = 0
PPG_CHANNEL  = 0
FS           = 100           # Hz
WINDOW_S     = 10            # seconds per BPM update
BUFFER_SIZE  = FS * WINDOW_S
LOWCUT       = 0.5           # Hz
HIGHCUT      = 5.0           # Hz
FILTER_ORDER = 3

POLAR_MAC    = "XX:XX:XX:XX:XX:XX"
HR_UUID      = "00002a37-0000-1000-8000-00805f9b34fb"
LOG_FILE     = "ppg_polar_plot.log"
CSV_FILE     = f"bpm_compare_{time.strftime('%Y%m%d_%H%M%S')}.csv"

# === GLOBAL STATE ===
data_lock   = threading.Lock()
ppg_bpm     = None
polar_bpm   = None

# === LIVE PLOT BUFFERS ===
max_len     = 100
ppg_vals    = deque([0]*max_len, maxlen=max_len)
polar_vals  = deque([0]*max_len, maxlen=max_len)
diff_vals   = deque([0]*max_len, maxlen=max_len)

# === LOGGING SETUP ===
logging.basicConfig(
    level    = logging.INFO,
    format   = "%(asctime)s [%(levelname)s] %(message)s",
    handlers = [logging.FileHandler(LOG_FILE), logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger("PPG_Plot")

# === HELPERS ===
def make_filter(fs, low, high, order):
    nyq = 0.5 * fs
    return butter(order, [low/nyq, high/nyq], btype='band')

def read_adc(spi, channel):
    cmd  = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

def parse_ble_hr(data: bytearray) -> int:
    flag = data[0]
    if flag & 0x01:
        return int.from_bytes(data[1:3], byteorder='little')
    return data[1]

# === MAIN CLASS ===
class PPGPolarPlot:
    def __init__(self):
        # SPI setup for sampler
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 1_350_000
        # filter design for processor
        self.b, self.a = make_filter(FS, LOWCUT, HIGHCUT, FILTER_ORDER)

        # sliding buffers for processor
        self.raw_buf = deque(maxlen=BUFFER_SIZE)
        self.ts_buf  = deque(maxlen=BUFFER_SIZE)

        # queue to decouple sampling
        self.sample_q = queue.Queue()

        # history for summary
        self.diff_history  = []
        self.ppg_history   = []
        self.polar_history = []

        # CSV logging
        self.csv_f = open(CSV_FILE, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['Timestamp','PPG_BPM','Polar_BPM','Difference'])

        # control
        self.running = True
        signal.signal(signal.SIGINT, self._shutdown)
        log.info(f"Initialized PPGPolarPlot (FS={FS}Hz, window={WINDOW_S}s)")

    def _shutdown(self, signum, frame):
        log.info("Shutdown requested")
        self.running = False

    def sampler_thread(self):
        """Reads raw ADC at fixed rate and enqueues samples."""
        next_t = time.time()
        while self.running:
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            ts = time.time()
            try:
                raw = read_adc(self.spi, PPG_CHANNEL)
            except Exception as e:
                log.error(f"SPI error: {e}")
            else:
                self.sample_q.put((ts, raw))
            next_t += 1.0 / FS

    def processor_thread(self):
        """Consumes samples, runs filter + BPM compute once per full window."""
        global ppg_bpm
        while self.running or not self.sample_q.empty():
            try:
                ts, raw = self.sample_q.get(timeout=0.5)
            except queue.Empty:
                continue
            self.raw_buf.append(raw)
            self.ts_buf.append(ts)

            if len(self.raw_buf) == BUFFER_SIZE:
                raw_arr  = np.array(self.raw_buf, float)
                filt_arr = filtfilt(self.b, self.a, raw_arr)
                bpm = self.compute_bpm(np.array(self.ts_buf), filt_arr)
                with data_lock:
                    ppg_bpm = bpm
                    pol    = polar_bpm
                diff = abs(bpm - pol) if (bpm is not None and pol is not None) else None

                # record history
                if diff is not None:
                    self.diff_history.append(diff)
                    self.ppg_history.append(ppg_bpm)
                    self.polar_history.append(polar_bpm)

                # log CSV + console
                tstr = time.strftime('%Y-%m-%d %H:%M:%S')
                self.csv_w.writerow([
                    tstr,
                    f"{bpm:.1f}" if bpm else "",
                    f"{pol}"     if pol else "",
                    f"{diff:.1f}" if diff else ""
                ])
                self.csv_f.flush()
                if bpm is not None:
                    log.info(f"PPG={bpm:.1f} Polar={pol or '-'} Δ={diff or '-'}")
                else:
                    log.warning("PPG: no valid peaks detected")
        self.cleanup()

    def compute_bpm(self, timestamps, signal_data):
        """Envelope-based peak detection identical to standalone 10 s reader."""
        env      = np.abs(signal_data - np.mean(signal_data))
        height   = np.mean(env) + 0.5 * np.std(env)
        min_dist = int(0.5 * FS)
        peaks, _ = find_peaks(env, distance=min_dist, height=height)
        log.debug(f"Envelope peaks: {len(peaks)}")
        if len(peaks) < 8:
            return None
        intervals = np.diff(timestamps[peaks])
        return 60.0 / np.mean(intervals)

    def _polar_task(self):
        async def runner():
            global polar_bpm
            try:
                async with BleakClient(POLAR_MAC) as client:
                    log.info(f"Polar H10 connected ({POLAR_MAC})")
                    await client.start_notify(HR_UUID, self._on_polar)
                    while self.running:
                        await asyncio.sleep(1.0)
            except Exception as e:
                log.error(f"Polar BLE error: {e}")
        asyncio.run(runner())

    def _on_polar(self, handle, data):
        hr = parse_ble_hr(data)
        with data_lock:
            global polar_bpm
            polar_bpm = hr
        log.debug(f"Polar BPM: {hr}")

    def cleanup(self):
        # summary
        if self.diff_history:
            avg_diff     = sum(self.diff_history) / len(self.diff_history)
            avg_ppg      = sum(self.ppg_history)   / len(self.ppg_history)
            avg_pol      = sum(self.polar_history) / len(self.polar_history)
            diff_of_avgs = abs(avg_ppg - avg_pol)
            log.info(f"Average of instantaneous differences: {avg_diff:.2f} BPM")
            log.info(f"Difference of overall averages:   {diff_of_avgs:.2f} BPM")
            # write summary into CSV
            self.csv_w.writerow([])
            self.csv_w.writerow(['Average of instantaneous differences', f"{avg_diff:.2f}"])
            self.csv_w.writerow(['Difference of overall averages',    f"{diff_of_avgs:.2f}"])

        try: self.spi.close()
        except: pass
        try: self.csv_f.close()
        except: pass
        log.info("Cleaned up and exiting")

    def start(self):
        threading.Thread(target=self._polar_task, daemon=True).start()
        threading.Thread(target=self.sampler_thread, daemon=True).start()
        threading.Thread(target=self.processor_thread, daemon=True).start()

# === PLOTTING ===
def update_plot(frame):
    with data_lock:
        ppg_vals.append(ppg_bpm or 0)
        polar_vals.append(polar_bpm or 0)
        diff_vals.append(abs((ppg_bpm or 0) - (polar_bpm or 0)))
    line_ppg.set_data(range(max_len), list(ppg_vals))
    line_pol.set_data(range(max_len), list(polar_vals))
    line_diff.set_data(range(max_len), list(diff_vals))
    ax.relim(); ax.autoscale_view()
    return line_ppg, line_pol, line_diff

if __name__ == '__main__':
    reader = PPGPolarPlot()
    reader.start()

    fig, ax = plt.subplots()
    line_ppg, = ax.plot([], [], label='PPG BPM')
    line_pol, = ax.plot([], [], label='Polar BPM')
    line_diff,= ax.plot([], [], label='Difference')
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('BPM')
    ax.set_title('Live PPG vs. Polar H10 BPM')
    ax.legend()

    ani = animation.FuncAnimation(fig, update_plot, interval=200)
    plt.tight_layout()
    plt.show()

    # stop threads on window close
    reader.running = False
