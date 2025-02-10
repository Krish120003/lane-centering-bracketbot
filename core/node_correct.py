#!/usr/bin/env python3

"""
node_correct.py - Approximate lane-centering robot steering and drive control
                  for Bracket Bot using OpenCV. 

This script uses OpenCV to detect red object-markers that define lane bounding
boxes for a path and safely navigate the robot along the path. 

We used red plastic party cups as markers to define edges of a "lane" path. The
countour thresholding is hand-tuned to detect these cups and discard noise or large
red objects in the environment.

Authors:
    - Krish (https://github.com/Krish120003)
    - Rachelle De Man (https://github.com/demanr)
    - Jackie Yi (https://github.com/Luigimash)
"""


import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
import time
import threading
import random

# Constants
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

# Control parameters
BASE_LINEAR_VELOCITY = 0.24  # Base forward speed
ANGULAR_CAP = 0.9  # Maximum angular velocity in rad/s
KP_ANGULAR = 0.005  # Proportional gain for angular control

# def preprocess_image(image):
#     """Convert image to HSV and create mask for red objects"""
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Define broader range for red color
#     lower_red1 = np.array([0, 100, 100])     # First range of red
#     upper_red1 = np.array([10, 255, 255])    # Narrower hue range
#     lower_red2 = np.array([160, 100, 100])   # Second range of red
#     upper_red2 = np.array([180, 255, 255])   # End of hue spectrum


#     # Create masks for both red ranges and combine
#     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     return cv2.bitwise_or(mask1, mask2)


def preprocess_image(image):
    """Convert image to HSV and create mask for red objects with tighter thresholds"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define tighter ranges for red color
    # First range (0-10 in hue)
    lower_red1 = np.array([0, 120, 100])  # Increased saturation minimum
    upper_red1 = np.array([8, 255, 255])  # Reduced hue range

    # Second range (160-180 in hue)
    lower_red2 = np.array([170, 120, 100])  # Increased saturation minimum
    upper_red2 = np.array([180, 255, 255])  # End of hue spectrum

    # Create masks for both red ranges and combine
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Optional: Add morphological operations to reduce noise
    combined_mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((3, 3), np.uint8)
    combined_mask = cv2.erode(combined_mask, kernel, iterations=1)
    combined_mask = cv2.dilate(combined_mask, kernel, iterations=1)

    return combined_mask


def detect_red_targets(image):
    """Detect red targets and return the average position of the three closest to center"""
    height, width = image.shape[:2]
    mask = preprocess_image(image)
    center_x = width // 2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Store all valid contours with their distances
    valid_targets = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 50:  # Minimum area threshold
            continue

        if area > 800:  # Maximum area threshold
            continue

        x, y, w, h = cv2.boundingRect(contour)
        target_center_x = x + w // 2
        distance = abs(target_center_x - center_x)

        valid_targets.append(
            {
                "contour": contour,
                "center_x": target_center_x,
                "distance": distance,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
            }
        )

    if not valid_targets:
        return None, None, mask

    # Sort targets by distance from center and get top 3
    valid_targets.sort(key=lambda x: x["distance"])
    closest_targets = valid_targets

    # Calculate average center position
    avg_center_x = int(
        sum(t["center_x"] for t in closest_targets) / len(closest_targets)
    )

    # Draw visualization
    for target in closest_targets:
        # Draw rectangle around target
        cv2.rectangle(
            image,
            (target["x"], target["y"]),
            (target["x"] + target["w"], target["y"] + target["h"]),
            (0, 255, 255),
            2,
        )
        # Draw center point
        cv2.circle(
            image,
            (target["center_x"], target["y"] + target["h"] // 2),
            3,
            (0, 255, 0),
            -1,
        )

    # Draw average center line
    cv2.line(image, (avg_center_x, 0), (avg_center_x, height), (0, 255, 0), 2)
    # Draw target center line
    cv2.line(image, (center_x, 0), (center_x, height), (255, 0, 0), 1)

    return avg_center_x, center_x, mask


def calculate_velocities(avg_center_x, target_x):
    """Calculate velocities based on average position of targets"""
    if avg_center_x is None:
        return BASE_LINEAR_VELOCITY, 0

    # Calculate angular velocity based on horizontal error
    error_x = target_x - avg_center_x
    angular_velocity = KP_ANGULAR * error_x
    angular_velocity = np.clip(angular_velocity, -ANGULAR_CAP, ANGULAR_CAP)

    return BASE_LINEAR_VELOCITY, angular_velocity


def main():
    # Set up MQTT client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER_ADDRESS)
    client.loop_start()

    try:
        while True:
            cap = cv2.VideoCapture(0)

            ret, frame = cap.read()
            if not ret:
                break

            # Crop top portion of the frame
            height = frame.shape[0]
            frame_cropped = frame[: int(0.7 * height), :]

            frame_cropped = cv2.resize(frame_cropped, None, fx=0.75, fy=0.75)

            frame_blurred = cv2.GaussianBlur(frame_cropped, (5, 5), 0)

            # Detect red targets
            avg_center_x, target_x, mask = detect_red_targets(frame_cropped)

            # Calculate control velocities
            linear_vel, angular_vel = calculate_velocities(avg_center_x, target_x)

            # Create and publish command
            command = {
                "linear_velocity": float(linear_vel),
                "angular_velocity": float(angular_vel),
            }
            client.publish(MQTT_TOPIC, json.dumps(command))

            # Display debug information
            status = "TRACKING" if avg_center_x is not None else "SEARCHING"
            cv2.putText(
                frame_cropped,
                f"{status} - Linear: {linear_vel:.3f} Angular: {angular_vel:.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Show frames
            cv2.imshow("Camera View", frame_cropped)
            cv2.imshow("Red Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            # Set current time
            last_frame_time = time.time()

            # Function to check for timeout and send stop signal
            def check_timeout():
                while True:
                    if time.time() - last_frame_time > 2:
                        stop_command = {"linear_velocity": 0.0, "angular_velocity": 0.0}
                        client.publish(MQTT_TOPIC, json.dumps(stop_command))
                        print("STOPPPPPP")
                    time.sleep(0.1)

            # Start timeout checker thread
            timeout_thread = threading.Thread(target=check_timeout, daemon=True)
            timeout_thread.start()

            cap.release()
    finally:
        cv2.destroyAllWindows()
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
