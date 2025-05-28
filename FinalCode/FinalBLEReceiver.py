import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    print("BPM:", msg.payload.decode())

client = mqtt.Client()
client.on_message = on_message
client.connect("<172.23.149.49>", 1883, 60)
client.subscribe("home/ppg/bpm")
client.loop_forever()
