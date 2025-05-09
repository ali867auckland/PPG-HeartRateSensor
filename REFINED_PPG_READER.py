import spidev
import time

# === MCP3008 Setup ===
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1350000

# === Pulse Sensor Algorithm Variables ===
Signal = 0
sampleCounter = 0
lastBeatTime = 0
P = 512
T = 512
thresh = 550
amp = 100
IBI = 600
BPM = 0
Pulse = False
firstBeat = True
secondBeat = False
threshSetting = 550

# === Timing ===
lastTime = time.time()

# === Read from MCP3008 ===
def read_adc(channel=0):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((r[1] & 3) << 8) | r[2]

try:
    print("📈 Starting Pulse Detection (Ctrl+C to stop)")
    while True:
        currentTime = time.time()
        sampleIntervalMs = int((currentTime - lastTime) * 1000)
        lastTime = currentTime
        sampleCounter += sampleIntervalMs

        Signal = read_adc(0)
        N = sampleCounter - lastBeatTime

        # Track trough
        if Signal < thresh and N > (IBI / 5) * 3:
            if Signal < T:
                T = Signal

        # Track peak
        if Signal > thresh and Signal > P:
            P = Signal

        # Look for beat
        if N > 250:
            if (Signal > thresh) and (not Pulse) and (N > (IBI / 5) * 3):
                Pulse = True
                IBI = sampleCounter - lastBeatTime
                lastBeatTime = sampleCounter

                if secondBeat:
                    secondBeat = False

                if firstBeat:
                    firstBeat = False
                    secondBeat = True
                    continue  # Ignore the first IBI

                BPM = int(60000 / IBI)
                print(f"💓 BPM: {BPM}")

        # Reset pulse detection after beat
        if Signal < thresh and Pulse:
            Pulse = False
            amp = P - T
            thresh = amp // 2 + T
            P = thresh
            T = thresh

        # Reset if no beat detected for too long
        if N > 2500:
            thresh = threshSetting
            P = 512
            T = 512
            lastBeatTime = sampleCounter
            firstBeat = True
            secondBeat = False
            BPM = 0
            IBI = 600
            Pulse = False
            amp = 0

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Stopped.")
finally:
    spi.close()
