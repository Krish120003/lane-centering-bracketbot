#!/usr/bin/env python3

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import paho.mqtt.client as mqtt
from sshkeyboard import listen_keyboard, stop_listening

# ------------------------------------------------------------------------------------
# Constants & Setup
# ------------------------------------------------------------------------------------
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

# Create MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(MQTT_BROKER_ADDRESS)
client.loop_start()

def press(key):
    if key.lower() == 'w':  # Forward
        client.publish(MQTT_TOPIC, "forward")
    elif key.lower() == 's':  # Backward
        client.publish(MQTT_TOPIC, "back") 
    elif key.lower() == 'a':  # Left turn
        client.publish(MQTT_TOPIC, "left")
    elif key.lower() == 'd':  # Right turn
        client.publish(MQTT_TOPIC, "right")
    elif key.lower() == 'q':  # Quit
        stop_listening()

def release(key):
    # Stop motors when key is released
    client.publish(MQTT_TOPIC, "stop")

try:
    print("WASD to control, Q to quit")
    listen_keyboard(
        on_press=press,
        on_release=release,

    )

except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    client.publish(MQTT_TOPIC, "stop")
    client.loop_stop()
    client.disconnect()
    print("Shutdown complete.")
