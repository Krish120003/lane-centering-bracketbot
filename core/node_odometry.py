#!/usr/bin/env python3

import json
import time
import math
import sys
import os
import paho.mqtt.client as mqtt

# Adds the lib directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from lib.odrive_uart import ODriveUART

# ------------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------------
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC_ODOMETRY = "robot/odometry"
MQTT_TOPIC_RESET_ODOMETRY = "robot/reset_odometry"  # New topic

# ODrive UART port
ODRIVE_UART_PORT = '/dev/ttyAMA1'  # Adjust as necessary

# Robot parameters
WHEEL_RADIUS = 0.0825   # meters (adjust based on your robot's wheel radius)
WHEEL_BASE = 0.420      # meters (track width is 400mm)

# ------------------------------------------------------------------------------------
# Initialize ODrive
# ------------------------------------------------------------------------------------
try:
    # Load motor directions from JSON file
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
        motor_dirs = json.load(f)
        left_dir = motor_dirs['left']
        right_dir = motor_dirs['right']
except Exception as e:
    raise Exception("Error reading motor_dir.json") from e

# Initialize the motor controller
motor_controller = ODriveUART(
    port=ODRIVE_UART_PORT,
    left_axis=0,
    right_axis=1,
    dir_left=left_dir,
    dir_right=right_dir
)

# ------------------------------------------------------------------------------------
# Odometry Node
# ------------------------------------------------------------------------------------
def on_reset_odometry(client, userdata, msg):
    """
    Callback to reset odometry when a reset message is received.
    """
    global x, y, theta
    payload = json.loads(msg.payload)
    if payload.get('reset', False):
        x = 0.0
        y = 0.0
        theta = 0.0
        print("[node_odometry.py] Odometry reset to zero.")

        # Reset encoder readings as well
        global prev_left_turns, prev_right_turns
        prev_left_turns = motor_controller.get_position_turns_left()
        prev_right_turns = motor_controller.get_position_turns_right()
        print("[node_odometry.py] Encoder counts reset.")

def on_message(client, userdata, msg):
    """
    General message handler.
    """
    if msg.topic == MQTT_TOPIC_RESET_ODOMETRY:
        on_reset_odometry(client, userdata, msg)

def main():
    global x, y, theta
    global prev_left_turns, prev_right_turns

    # Initialize MQTT client
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER_ADDRESS)
    client.loop_start()

    # Subscribe to odometry reset topic
    client.subscribe(MQTT_TOPIC_RESET_ODOMETRY)
    print("[node_odometry.py] Subscribed to reset odometry topic.")

    # Initialize robot state
    x = 0.0    # in meters
    y = 0.0    # in meters
    theta = 0.0  # in radians

    # Get initial encoder readings
    prev_left_turns = motor_controller.get_position_turns_left()
    prev_right_turns = motor_controller.get_position_turns_right()

    rate = 50  # Compute odometry at 50 Hz
    publish_rate = 5  # Publish odometry at 5 Hz
    dt = 1.0 / rate
    publish_interval = 1.0 / publish_rate  # Time between publishes in seconds
    last_publish_time = time.time()

    print("[node_odometry.py] Starting odometry node.")

    try:
        while True:
            current_time = time.time()

            # Get current encoder turns
            try:
                curr_left_turns = motor_controller.get_position_turns_left()
                curr_right_turns = motor_controller.get_position_turns_right()
            except Exception as e:
                print(f"[node_odometry.py] Error getting encoder turns: {e}")
                continue

            # Compute difference in turns
            delta_left_turns = curr_left_turns - prev_left_turns
            delta_right_turns = curr_right_turns - prev_right_turns

            # Update previous turns
            prev_left_turns = curr_left_turns
            prev_right_turns = curr_right_turns

            # Convert turns to distances
            delta_left_dist = delta_left_turns * 2.0 * math.pi * WHEEL_RADIUS
            delta_right_dist = delta_right_turns * 2.0 * math.pi * WHEEL_RADIUS

            # Compute average distance
            delta_dist = (delta_right_dist + delta_left_dist) / 2.0

            # Compute change in orientation
            delta_theta = (delta_right_dist - delta_left_dist) / WHEEL_BASE

            # Compute new pose
            x += delta_dist * math.cos(theta + delta_theta / 2.0)
            y += delta_dist * math.sin(theta + delta_theta / 2.0)
            theta += delta_theta

            # Normalize theta to (-pi, pi]
            theta = (theta + math.pi) % (2 * math.pi) - math.pi

            # Publish odometry data at 5 Hz
            if current_time - last_publish_time >= publish_interval:
                odom_msg = {
                    'x': x,
                    'y': y,
                    'theta': theta
                }
                client.publish(MQTT_TOPIC_ODOMETRY, json.dumps(odom_msg))
                print(f"Published odometry data: {odom_msg}")
                last_publish_time = current_time  # Reset last publish time

            # Sleep for the remainder of the loop
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[node_odometry.py] Interrupted by user.")
    finally:
        client.loop_stop()
        client.disconnect()
        print("[node_odometry.py] Shutdown complete.")

if __name__ == "__main__":
    main()
