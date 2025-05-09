#!/usr/bin/env python3

import time
import threading
import spidev
import csv
import datetime
import asyncio
import socket
from bleak import BleakClient
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ==== Configuration ====
POLAR_MAC = "XX:XX:XX:XX:XX:XX"  # <-- Replace with your Polar H10 MAC address
HR_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
threshold = 520

# ==== Global Shared Variables ====
ppg_bpm = 0
polar_bpm = 0
last_ppg_timestamp = 0
last_polar_timestamp = 0
start_time = time.time()
diff_values_logged = []

prev_above = False
last_beat_time = 0
data_lock = threading.Lock()

# ==== SPI Setup for MCP3008 ====
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel=0):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((r[1] & 3) << 8) | r[2]

def detect_ppg_bpm():
    global prev_above, last_beat_time, ppg_bpm, last_ppg_timestamp
    value = read_adc(0)
    now = int(time.time() * 1000)
    above = value > threshold
    if above and not prev_above and (now - last_beat_time > 300):
        interval = now - last_beat_time
        last_beat_time = now
        with data_lock:
            ppg_bpm = int(60000 / interval)
            last_ppg_timestamp = time.time()
        prev_above = above
    prev_above = above

def ppg_reader_thread():
    while True:
        detect_ppg_bpm()
        time.sleep(0.01)

def parse_ble_hr(data):
    flag = data[0]
    if flag & 0x01:
        return int.from_bytes(data[1:3], byteorder='little')
    return data[1]

def handle_polar_notification(handle, data):
    global polar_bpm, last_polar_timestamp
    bpm = parse_ble_hr(data)
    with data_lock:
        polar_bpm = bpm
        last_polar_timestamp = time.time()
    print(f"[Polar H10 BLE] BPM: {bpm}")

async def polar_ble_reader():
    async with BleakClient(POLAR_MAC) as client:
        print(f"Connected to Polar H10 via BLE ({POLAR_MAC})")
        await client.start_notify(HR_UUID, handle_polar_notification)
        while True:
            await asyncio.sleep(1)

def run_ble():
    asyncio.run(polar_ble_reader())

# ==== TCP Socket Setup ====
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('', 5001))
sock.listen(1)
print("Waiting for connection from client...")
client_sock, addr = sock.accept()
print(f"Connected to {addr}")

# ==== CSV Setup ====
csv_filename = f"bpm_data_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['PPG Timestamp', 'Polar Timestamp', 'PPG BPM', 'Polar BPM', 'Difference', 'Filtered'])

# ==== Plot Setup ====
max_len = 100
ppg_vals = deque([0]*max_len, maxlen=max_len)
polar_vals = deque([0]*max_len, maxlen=max_len)
diff_vals = deque([0]*max_len, maxlen=max_len)

def update_graph(frame):
    global diff_values_logged
    with data_lock:
        ppg_vals.append(ppg_bpm)
        polar_vals.append(polar_bpm)
        if abs(last_ppg_timestamp - last_polar_timestamp) < 1.0:
            diff = abs(ppg_bpm - polar_bpm)
            diff_vals.append(diff)
            diff_values_logged.append(diff)
        else:
            diff = 0
            diff_vals.append(0)

    ppg_time_str = datetime.datetime.fromtimestamp(last_ppg_timestamp).strftime('%Y-%m-%d %H:%M:%S')
    polar_time_str = datetime.datetime.fromtimestamp(last_polar_timestamp).strftime('%Y-%m-%d %H:%M:%S')
    filtered_flag = diff if diff < 20 else ''
    csv_writer.writerow([ppg_time_str, polar_time_str, ppg_bpm, polar_bpm, diff, filtered_flag])

    try:
        msg = f"{ppg_time_str},{polar_time_str},{ppg_bpm},{polar_bpm},{diff},{filtered_flag}\n"
        client_sock.sendall(msg.encode())
    except Exception as e:
        print("TCP send error:", e)

    csv_file.flush()

    line1.set_data(range(len(ppg_vals)), list(ppg_vals))
    line2.set_data(range(len(polar_vals)), list(polar_vals))
    line3.set_data(range(len(diff_vals)), list(diff_vals))
    ax.relim()
    ax.autoscale_view()
    return line1, line2, line3

# ==== Start Threads and Main Loop ====
threading.Thread(target=ppg_reader_thread, daemon=True).start()
threading.Thread(target=run_ble, daemon=True).start()

fig, ax = plt.subplots()
line1, = ax.plot([], [], label="PPG BPM")
line2, = ax.plot([], [], label="Polar BPM")
line3, = ax.plot([], [], label="Difference")
ax.set_title("Real-Time BPM Comparison")
ax.set_xlabel("Time")
ax.set_ylabel("BPM")
ax.legend()
ani = animation.FuncAnimation(fig, update_graph, interval=500)
plt.tight_layout()

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    duration = time.time() - start_time
    minutes, seconds = divmod(int(duration), 60)
    avg_diff_all = sum(diff_values_logged) / len(diff_values_logged) if diff_values_logged else 0
    filtered_diffs = [d for d in diff_values_logged if d < 20]
    avg_diff_filtered = sum(filtered_diffs) / len(filtered_diffs) if filtered_diffs else 0
    discarded = len(diff_values_logged) - len(filtered_diffs)

    summary_text = (
        f"Data collection duration: {minutes} min {seconds} sec\n"
        f"Samples recorded: {len(diff_values_logged)}\n"
        f"Average BPM difference (all): {avg_diff_all:.2f}\n"
        f"Average BPM difference (<20 BPM only): {avg_diff_filtered:.2f}\n"
        f"Outliers discarded: {discarded}\n"
    )
    print("\n--- Summary ---")
    print(summary_text)
    with open("summary.txt", "w") as f:
        f.write(summary_text)
    csv_file.close()
    spi.close()
    client_sock.close()
    sock.close()
