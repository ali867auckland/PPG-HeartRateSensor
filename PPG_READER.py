import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  
spi.max_speed_hz = 1350000

threshold = 520
prev_above = False
last_beat_time = 0
bpm = 0

def read_adc(channel=0):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((r[1] & 3) << 8) | r[2]

def detect_bpm():
    global prev_above, last_beat_time, bpm
    value = read_adc(0)
    now = int(time.time() * 1000)
    above = value > threshold

    # Rising edge detected
    if above and not prev_above and (now - last_beat_time > 300):
        interval = now - last_beat_time
        bpm = int(60000 / interval)
        last_beat_time = now
        prev_above = above
        return bpm

    prev_above = above
    return None

try:
    print("Reading PulseSensor")
    while True:
        result = detect_bpm()
        if result:
            print("BPM:", result)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    spi.close()
