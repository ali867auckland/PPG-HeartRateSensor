import spidev
import time
import numpy as np
from scipy.signal import butter, lfilter

# === MCP3008 Setup ===
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel=0):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((r[1] & 3) << 8) | r[2]

# === Bandpass Filter Setup (0.5–5 Hz for heart rate) ===
def butter_bandpass(lowcut, highcut, fs, order=2):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_filter(data, b, a):
    return lfilter(b, a, data)

fs = 100  # Hz sampling rate
b, a = butter_bandpass(0.5, 5, fs)
signal_buffer = []
filter_window_size = 50

# === Pulse Detection Variables ===
Signal = 0
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
firstBeat = True
secondBeat = False

start_time = time.time()

try:
    print("💓 Starting PulseSensor Reader with Bandpass Filtering (Ctrl+C to stop)")
    while True:
        currentTime = time.time()
        elapsed_ms = int((currentTime - start_time) * 1000)
        start_time = currentTime
        sampleCounter += elapsed_ms

        raw_signal = read_adc(0)
        signal_buffer.append(raw_signal)

        # Keep buffer at correct size
        if len(signal_buffer) > filter_window_size:
            signal_buffer.pop(0)

        # Apply filter if buffer is full
        if len(signal_buffer) >= filter_window_size:
            filtered = apply_filter(signal_buffer, b, a)
            Signal = int(filtered[-1])
        else:
            Signal = raw_signal

        N = sampleCounter - lastBeatTime

        # Detect trough
        if Signal < thresh and N > (IBI / 5) * 3:
            if Signal < T:
                T = Signal

        # Detect peak
        if Signal > thresh and Signal > P:
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

                rate.pop(0)
                rate.append(IBI)
                runningTotal = sum(rate)
                BPM = 60000 // (runningTotal // len(rate))
                print(f"💓 BPM: {BPM}")

        # Reset pulse detection
        if Signal < thresh and Pulse:
            Pulse = False
            amp = P - T
            thresh = amp // 2 + T
            P = thresh
            T = thresh

        # Reset system after too long without beat
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
            rate = [600] * 10
            print("⏳ No beat detected, reset")

        time.sleep(1.0 / fs)

except KeyboardInterrupt:
    print("\n🛑 Stopped.")
finally:
    spi.close()
