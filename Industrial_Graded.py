import spidev
import time
import numpy as np
from scipy.signal import butter, lfilter
import csv
import os
from datetime import datetime

# =========================
# CONFIGURATION
# =========================
CHANNEL = 0
SAMPLE_RATE = 50  # Hz
BUFFER_SIZE = 250  # 5 seconds of data
BPM_SMOOTHING = 10
LOG_FILE = "ppg_log.csv"
THRESH_SENSITIVITY = 0.6
BANDPASS_LOW = 0.5
BANDPASS_HIGH = 4.0

# =========================
# SETUP
# =========================
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

signal_buffer = []
IBIs = []
last_peak_time = time.time()
prev_filtered = 0
threshold = 512
bpm = 0

# CSV Logging Setup
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Raw", "Filtered", "BPM"])

# =========================
# FILTERS
# =========================
def butter_bandpass(lowcut, highcut, fs, order=2):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    return butter(order, [low, high], btype='band')

def apply_filter(data, lowcut=0.5, highcut=4.0, fs=50):
    b, a = butter_bandpass(lowcut, highcut, fs)
    return lfilter(b, a, data)

# =========================
# DATA ACQUISITION
# =========================
def read_channel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

# =========================
# BPM CALCULATION
# =========================
def detect_peak(signal, threshold, prev_signal):
    return signal > threshold and prev_signal <= threshold

def calculate_threshold(peaks, troughs):
    if not peaks or not troughs:
        return 512  # fallback
    return min(peaks[-1], 1023) - THRESH_SENSITIVITY * (min(peaks[-1], 1023) - max(troughs[-1], 0))

# =========================
# MAIN LOOP
# =========================
print("Starting industrial-grade PPG reader. Press Ctrl+C to stop.")
try:
    peaks = []
    troughs = []
    while True:
        timestamp = datetime.now().isoformat()
        raw = read_channel(CHANNEL)

        signal_buffer.append(raw)
        if len(signal_buffer) < BUFFER_SIZE:
            time.sleep(1 / SAMPLE_RATE)
            continue
        elif len(signal_buffer) > BUFFER_SIZE:
            signal_buffer.pop(0)

        filtered = apply_filter(signal_buffer, BANDPASS_LOW, BANDPASS_HIGH, SAMPLE_RATE)[-1]

        now = time.time()
        if detect_peak(filtered, threshold, prev_filtered):
            ibi = (now - last_peak_time) * 1000  # ms
            if 300 < ibi < 2000:  # Valid range for human BPM
                IBIs.append(ibi)
                if len(IBIs) > BPM_SMOOTHING:
                    IBIs.pop(0)
                bpm = int(60000 / np.mean(IBIs))
                peaks.append(filtered)
        elif filtered < threshold:
            troughs.append(filtered)

        prev_filtered = filtered
        threshold = calculate_threshold(peaks[-BPM_SMOOTHING:], troughs[-BPM_SMOOTHING:])

        print(f"{timestamp} | Raw: {raw} | Filtered: {int(filtered)} | BPM: {bpm}")
        with open(LOG_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, raw, int(filtered), bpm])

        time.sleep(1 / SAMPLE_RATE)

except KeyboardInterrupt:
    print("\nStopped by user. Cleaning up...")
    spi.close()
