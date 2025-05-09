
import spidev
import time
import numpy as np
from scipy.signal import butter, lfilter

# === Enhancements Applied ===
# 1. Bandpass Filter (0.5 - 5 Hz) to clean PPG signal
# 2. Adaptive Thresholding with rolling amplitude average and BPM validation
# 3. Outlier Rejection: Only accept BPM between 30 and 220
# =============================================================

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel=0):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((r[1] & 3) << 8) | r[2]

def butter_bandpass(lowcut, highcut, fs, order=2):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_filter(data, b, a):
    return lfilter(b, a, data)

# === Pulse Detection Variables ===
Signal = 0
signal_buffer = []
filter_window_size = 50

sampleCounter = 0
lastBeatTime = 0
P = 512
T = 512
thresh = 525
amp = 100
IBI = 600
Pulse = False
BPM = 0
rate = [600] * 10
amp_history = [100] * 5
firstBeat = True
secondBeat = False

fs = 100  # Hz
b, a = butter_bandpass(0.5, 5, fs)

start_time = time.time()

try:
    print("Pulse Reader with Filter, Adaptive Thresholding, and Outlier Handling")
    while True:
        currentTime = time.time()
        elapsed_ms = int((currentTime - start_time) * 1000)
        start_time = currentTime
        sampleCounter += elapsed_ms

        raw_signal = read_adc(0)
        signal_buffer.append(raw_signal)

        if len(signal_buffer) > filter_window_size:
            signal_buffer.pop(0)

        if len(signal_buffer) >= filter_window_size:
            filtered_signal = apply_filter(signal_buffer, b, a)
            Signal = int(filtered_signal[-1])
        else:
            Signal = raw_signal

        N = sampleCounter - lastBeatTime

        # Validate and update trough
        if Signal < thresh and N > (IBI / 5) * 3 and 30 <= BPM <= 220:
            if Signal < T:
                T = Signal

        # Validate and update peak
        if Signal > thresh and 30 <= BPM <= 220:
            if Signal > P:
                P = Signal

        # Look for heartbeat
        if N > 250:
            if Signal > thresh and not Pulse and N > (IBI / 5) * 3:
                Pulse = True
                IBI = sampleCounter - lastBeatTime
                lastBeatTime = sampleCounter

                if firstBeat:
                    firstBeat = False
                    secondBeat = True
                    continue

                if secondBeat:
                    secondBeat = False
                    rate = [IBI] * 10

                BPM_candidate = 60000 // IBI
                if 30 <= BPM_candidate <= 220:
                    rate.pop(0)
                    rate.append(IBI)
                    BPM = 60000 // (sum(rate) // len(rate))

                    # Update amp and adaptive threshold using rolling average
                    amp = P - T
                    if amp < 10: amp = 10
                    amp_history.pop(0)
                    amp_history.append(amp)
                    avg_amp = sum(amp_history) / len(amp_history)
                    thresh = int(T + 0.6 * avg_amp)

                    print(f"BPM: {BPM}")
                else:
                    print(f"Rejected outlier IBI: {IBI} ms (BPM={BPM_candidate})")

        # Reset pulse flag
        if Signal < thresh and Pulse:
            Pulse = False
            P = thresh
            T = thresh

        if N > 2500:
            thresh = 525
            P = 512
            T = 512
            lastBeatTime = sampleCounter
            firstBeat = True
            secondBeat = False
            BPM = 0
            IBI = 600
            Pulse = False
            amp = 100
            amp_history = [100] * 5
            rate = [600] * 10
            print("No beat detected, reset")

        time.sleep(1.0 / fs)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    spi.close()
