#!/usr/bin/env python3
"""
Windows MQTT subscriber that logs incoming BPM messages to a CSV file.
- Uses MQTT v5 + callback API v2 (no deprecation warning)
- Subscribes to "home/ppg/bpm"
- Prepends each reading with an ISO timestamp
- Appends to bpm_log.csv (creates header if missing)
- Echoes each reading to the console
"""

import paho.mqtt.client as mqtt
import datetime
import os

# ─── CONFIGURATION ──────────────────────────────────────────────────────────────
PI_IP       = "172.23.149.49"      # Replace with your Pi’s IP
BROKER_PORT = 1883
TOPIC       = "home/ppg/bpm"
LOG_FILE    = "bpm_log.csv"       # CSV file to store timestamp,bpm

# ─── CALLBACKS (MQTT v5 + API v2) ───────────────────────────────────────────────

def on_connect(client, userdata, flags, reasonCode, properties=None):
    if reasonCode == mqtt.MQTT_ERR_SUCCESS:
        print(f"✅ Connected to broker at {PI_IP}:{BROKER_PORT}")
        client.subscribe(TOPIC)
    else:
        print("❌ Connection failed, reason code:", reasonCode)

def on_message(client, userdata, msg):
    # Build timestamped CSV line
    timestamp = datetime.datetime.now().isoformat()
    bpm_value = msg.payload.decode().strip()
    line      = f"{timestamp},{bpm_value}\n"

    # Ensure CSV header exists
    if not os.path.isfile(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            f.write("timestamp,bpm\n")

    # Append the new reading
    with open(LOG_FILE, "a", newline="") as f:
        f.write(line)

    # Echo to console
    print(f"{timestamp} — {msg.topic}: {bpm_value}")

# ─── CLIENT SETUP ────────────────────────────────────────────────────────────────

client = mqtt.Client(
    client_id="WinSubWithLog",
    protocol=mqtt.MQTTv5,
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)
client.on_connect = on_connect
client.on_message = on_message

# ─── MAIN ENTRY POINT ───────────────────────────────────────────────────────────

def main():
    client.connect(PI_IP, BROKER_PORT, keepalive=60)
    client.loop_forever()

if __name__ == "__main__":
    main()
