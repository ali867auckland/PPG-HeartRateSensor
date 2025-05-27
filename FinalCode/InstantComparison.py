# ppg_vs_polar_instant_smoothed.py
#!/usr/bin/env python3
"""
Smoothed Instantaneous PPG vs. Polar H10 comparison
- PPG instantaneous BPM via threshold crossing + exponential smoothing
- Polar H10 instant BPM via BLE notifications
- Logs smoothed PPG BPM, Polar BPM, and their difference to console and CSV
- Clean shutdown on Ctrl-C
"""

import spidev
import time
import signal
import logging
import sys
import csv
import asyncio
from threading import Event, Thread, Lock
from bleak import BleakClient

# === CONFIGURATION ===
SPI_BUS         = 0
SPI_DEVICE      = 0
PPG_CHANNEL     = 0
SAMPLING_RATE   = 100  # Hz
THRESHOLD       = 520  # ADC threshold
MIN_INTERVAL_MS = 300  # ms lockout

ALPHA = 0.3  # smoothing factor

POLAR_MAC       = "XX:XX:XX:XX:XX:XX"  # Polar H10 MAC
HR_UUID         = "00002a37-0000-1000-8000-00805f9b34fb"
CSV_FILE        = f"bpm_compare_instant_smoothed_{time.strftime('%Y%m%d_%H%M%S')}.csv"

# === LOGGING ===
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger('PPGPolarInstantSmoothed')

# === SHARED STATE ===
ppg_bpm = None
polar_bpm = None
lock = Lock()
stopped = Event()

# === SIGNAL HANDLER ===
def shutdown(signum, frame):
    log.info('Shutdown signal received')
    stopped.set()

signal.signal(signal.SIGINT, shutdown)

# === ADC READ ===
def read_adc(spi, channel):
    cmd = [1, (8 + channel) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

# === PPG THREAD ===
def ppg_thread():
    global ppg_bpm
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 1_350_000

    prev_above = False
    last_time = 0
    smoothed = None
    next_time = time.time()

    while not stopped.is_set():
        now = time.time()
        if now < next_time:
            time.sleep(next_time - now)
        raw = read_adc(spi, PPG_CHANNEL)
        above = raw > THRESHOLD
        current_ms = int(time.time() * 1000)

        if above and not prev_above and (current_ms - last_time) > MIN_INTERVAL_MS:
            if last_time != 0:
                interval = current_ms - last_time
                inst = 60000.0 / interval
                # smooth
                if smoothed is None:
                    smoothed = inst
                else:
                    smoothed = ALPHA * inst + (1 - ALPHA) * smoothed
                with lock:
                    ppg_bpm = smoothed
            last_time = current_ms
        prev_above = above
        next_time += 1.0 / SAMPLING_RATE

    spi.close()

# === BLE CALLBACK ===
def polar_callback(handle, data):
    global polar_bpm
    flag = data[0]
    if flag & 0x01:
        val = int.from_bytes(data[1:3], byteorder='little')
    else:
        val = data[1]
    with lock:
        polar_bpm = val
    log.debug('Polar BPM: %d', val)

# === BLE THREAD ===
async def polar_task():
    try:
        async with BleakClient(POLAR_MAC) as client:
            log.info(f'Connected to Polar H10 ({POLAR_MAC})')
            await client.start_notify(HR_UUID, polar_callback)
            while not stopped.is_set():
                await asyncio.sleep(1.0)
    except Exception as e:
        log.error('Polar BLE error: %s', e)


def run_polar():
    asyncio.run(polar_task())

# === MAIN ===
def main():
    Thread(target=ppg_thread, daemon=True).start()
    Thread(target=run_polar, daemon=True).start()

    # CSV logging
    csv_f = open(CSV_FILE, 'w', newline='')
    w = csv.writer(csv_f)
    w.writerow(['Timestamp','PPG_BPM','Polar_BPM','Difference'])

    log.info('Started smoothed PPG vs Polar comparison')
    while not stopped.is_set():
        time.sleep(1.0)
        with lock:
            p = ppg_bpm
            h = polar_bpm
        if p is not None or h is not None:
            diff = abs(p - h) if (p is not None and h is not None) else None
            tstr = time.strftime('%Y-%m-%d %H:%M:%S')
            w.writerow([
                tstr,
                f"{p:.1f}" if p else "",
                f"{h}"    if h else "",
                f"{diff:.1f}" if diff else ""
            ])
            csv_f.flush()
            log.info(f"PPG={p or '-'} Polar={h or '-'} Δ={diff or '-'}")

    csv_f.close()
    log.info('Exited')

if __name__ == '__main__':
    main()
