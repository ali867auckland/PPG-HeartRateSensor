#!/usr/bin/env python3
"""
Instantaneous PPG BPM reader
- Reads from MCP3008 ADC via SPI
- Detects beats via threshold crossing
- Computes BPM as 60,000 / (ms since last beat)
- Logs instantaneous BPM to console
- Clean shutdown on Ctrl-C
"""

import spidev
import time
import signal
import logging
import sys
from threading import Event

# === CONFIGURATION ===
SPI_BUS         = 0
SPI_DEVICE      = 0
PPG_CHANNEL     = 0
SAMPLING_RATE   = 100          # Hz
THRESHOLD       = 520          # ADC counts for beat detection
MIN_INTERVAL_MS = 300          # ms lockout between beats

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
    cmd  = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

# === MAIN ===
def main():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 1_350_000

    prev_above      = False
    last_beat_time  = 0
    next_sample_time = time.time()

    log.info('Starting instantaneous PPG BPM reader')
    while not stopped.is_set():
        now = time.time()
        if now < next_sample_time:
            time.sleep(next_sample_time - now)
        raw = read_adc(spi, PPG_CHANNEL)
        above = (raw > THRESHOLD)
        current_ms = int(time.time() * 1000)

        # rising edge and lockout
        if above and not prev_above and (current_ms - last_beat_time) > MIN_INTERVAL_MS:
            if last_beat_time != 0:
                interval = current_ms - last_beat_time
                bpm = 60000.0 / interval
                log.info('PPG BPM: %.1f', bpm)
            last_beat_time = current_ms
        prev_above = above
        next_sample_time += 1.0 / SAMPLING_RATE

    spi.close()
    log.info('Exited')

if __name__ == '__main__':
    main()
