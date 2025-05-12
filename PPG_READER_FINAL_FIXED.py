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
IBI = 600
Pulse = False
BPM = 0
rate = [600] * 10
ibi_history = []
firstBeat = True
secondBeat = False
alpha = 0.2  # smoothing factor for BPM
bpm_smoothed = 0

def compute_hrv(ibi_series):
    if len(ibi_series) < 2:
        return 0, 0
    ibi_diffs = np.diff(ibi_series)
    rmssd = np.sqrt(np.mean(ibi_diffs ** 2))
    sdnn = np.std(ibi_series)
    return round(sdnn, 2), round(rmssd, 2)

try:
    print("💓 Starting PulseSensor Reader with Filtering, SQI, HRV (Ctrl+C to stop)")
    start_time = time.time()

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
            filtered = apply_filter(signal_buffer, b, a)
            Signal = int(filtered[-1])
        else:
            Signal = raw_signal

        # Initialize peak/trough with signal
        if firstBeat and len(signal_buffer) == filter_window_size:
            P = Signal
            T = Signal

        N = sampleCounter - lastBeatTime

        # Update trough
        if N > (IBI / 5) * 3 and Signal < T:
            T = Signal

        # Update peak
        if Signal > P:
            P = Signal

        dynamic_thresh = T + (P - T) * 0.3  # 30% dynamic threshold

        # Beat detection
        if N > 250:
            if Signal > dynamic_thresh and not Pulse and N > (IBI / 5) * 3:
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

                # Exponential smoothing
                bpm_smoothed = alpha * BPM + (1 - alpha) * bpm_smoothed if bpm_smoothed else BPM
                ibi_history.append(IBI)
                if len(ibi_history) > 20:
                    ibi_history.pop(0)
                sdnn, rmssd = compute_hrv(np.array(ibi_history))

                # Signal Quality Index
                amp = P - T
                sqi = min(max(amp / 150, 0), 1)  # scaled for 0–150 amplitude
                if sqi < 0.1:
                    print(f"⚠️  Very low SQI: {sqi:.2f} — skipping BPM update")
                else:
                    print(f"💓 BPM: {bpm_smoothed:.1f} | SQI: {sqi:.2f} | SDNN: {sdnn} | RMSSD: {rmssd}")

        # Reset pulse state
        if Signal < dynamic_thresh and Pulse:
            Pulse = False
            P = Signal
            T = Signal

        # Handle long gaps without beats
        if N > 2500:
            print("⏳ No beat detected recently (2.5s+) — maintaining previous BPM.")
            lastBeatTime = sampleCounter
            Pulse = False
            P = Signal
            T = Signal

        time.sleep(1.0 / fs)

except KeyboardInterrupt:
    print("\n🛑 Stopped.")
finally:
    spi.close()
