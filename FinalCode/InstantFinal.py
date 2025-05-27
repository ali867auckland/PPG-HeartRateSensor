# ppg_instant_smoothed.py
#!/usr/bin/env python3
"""
Smoothed Instantaneous PPG BPM reader
- Reads from MCP3008 ADC via SPI
- Detects beats via threshold crossing with lockout
- Computes instantaneous BPM and applies exponential smoothing
- Logs smoothed BPM to console
- Clean shutdown on Ctrl-C
"""

import spidev
import time
import signal
import logging
import sys
from threading import Event

# === CONFIGURATION ===
SPI_BUS         = 0    # SPI bus
SPI_DEVICE      = 0    # SPI device
PPG_CHANNEL     = 0    # ADC channel
SAMPLING_RATE   = 100  # Hz
THRESHOLD       = 520  # ADC value threshold for beats
MIN_INTERVAL_MS = 300  # Minimum ms between beats

# Exponential smoothing factor (0 < ALPHA <= 1)
ALPHA = 0.3

# === LOGGING SETUP ===
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger('PPGInstant')

# === CONTROL FLAG ===
stopped = Event()

def shutdown(signum, frame):
    log.info('Shutdown signal received')
    stopped.set()

signal.signal(signal.SIGINT, shutdown)

# === ADC READ ===
def read_adc(spi, channel):
    cmd = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

# === MAIN LOOP ===
def main():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 1_350_000

    prev_above = False
    last_beat_time = 0
    smoothed_bpm = None
    next_sample = time.time()

    log.info('Starting smoothed instantaneous PPG reader')
    while not stopped.is_set():
        now = time.time()
        if now < next_sample:
            time.sleep(next_sample - now)
        raw = read_adc(spi, PPG_CHANNEL)
        above = raw > THRESHOLD
        current_ms = int(time.time() * 1000)

        # Detect rising edge with lockout
        if above and not prev_above and (current_ms - last_beat_time) > MIN_INTERVAL_MS:
            if last_beat_time != 0:
                interval = current_ms - last_beat_time
                inst_bpm = 60000.0 / interval
                # Exponential smoothing
                if smoothed_bpm is None:
                    smoothed_bpm = inst_bpm
                else:
                    smoothed_bpm = ALPHA * inst_bpm + (1 - ALPHA) * smoothed_bpm
                log.info('Smoothed PPG BPM: %.1f (inst %.1f)', smoothed_bpm, inst_bpm)
            last_beat_time = current_ms

        prev_above = above
        next_sample += 1.0 / SAMPLING_RATE

    spi.close()
    log.info('Exited')

if __name__ == '__main__':
    main()
