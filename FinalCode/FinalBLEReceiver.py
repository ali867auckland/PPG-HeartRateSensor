import paho.mqtt.client as mqtt

# ─── CALLBACKS (MQTT v5) ────────────────────────────────────────────────────────

def on_connect(client, userdata, flags, reasonCode, properties):
    if reasonCode == mqtt.MQTT_ERR_SUCCESS:
        print("✅ Connected (MQTTv5 + APIv2) to broker")
        client.subscribe("home/ppg/bpm")
    else:
        print("❌ Connection failed, reason code:", reasonCode)

def on_message(client, userdata, msg):
    print(f"{msg.topic}: {msg.payload.decode()}")

# ─── CLIENT SETUP ────────────────────────────────────────────────────────────────

client = mqtt.Client(
    client_id="WindowsSubscriberV5",
    protocol=mqtt.MQTTv5,
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)
client.on_connect = on_connect
client.on_message = on_message

# ─── CONNECT & LOOP ─────────────────────────────────────────────────────────────

PI_IP       = "172.23.149.49"   # ← just the raw IP
BROKER_PORT = 1883

client.connect(PI_IP, BROKER_PORT, keepalive=60)
client.loop_forever()
