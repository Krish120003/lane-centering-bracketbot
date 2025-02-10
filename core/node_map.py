#!/usr/bin/env python3
from __future__ import annotations

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import math
import json
import numpy as np
from typing import List, Dict
import base64

from RPi import GPIO
import smbus2
import paho.mqtt.client as mqtt

# VL53L5CX libraries
from lib.vl53l5cx_lib.vl53l5cx import VL53L5CX
from lib.vl53l5cx_lib.api import (
    VL53L5CX_RESOLUTION_4X4,
    VL53L5CX_RESOLUTION_8X8
)

# -----------------------------------------------------------------------------
# MQTT Setup
# -----------------------------------------------------------------------------
MQTT_BROKER = "localhost"    # Change if your broker is on a different machine
MQTT_PORT = 1883
MQTT_TOPIC = "robot/tof_map"  # Publish the map data here

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # Update to use VERSION2 callbacks
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
client.loop_start()

# -----------------------------------------------------------------------------
# Sensor Data Cache
# -----------------------------------------------------------------------------
sensor_data_cache = {
    0: None,  # Most recent valid data from sensor 0
    1: None,  # Most recent valid data from sensor 1
    2: None,  # Most recent valid data from sensor 2
}

# -----------------------------------------------------------------------------
# ToF Sensor Setup
# -----------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
sensor_pins = [17, 22, 27]  # LPn pins for the three sensors
for pin in sensor_pins:
    GPIO.setup(pin, GPIO.OUT)

def scan_i2c_bus(bus_number=1):
    bus = smbus2.SMBus(bus_number)
    devices = []
    for address in range(128):
        try:
            bus.write_byte(address, 0)
            devices.append(address)
        except IOError:
            pass
    return devices

def set_sensor_address(sensor, pin, new_address):
    # Pull down all LPn pins
    for p in sensor_pins:
        GPIO.output(p, GPIO.LOW)
    # Power up the current sensor
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    # Set new I2C address
    sensor.set_i2c_address(new_address)
    # Pull up all pins again
    for p in sensor_pins:
        GPIO.output(p, GPIO.HIGH)

print("Scanning I2C bus...")
existing_addresses = scan_i2c_bus()
print(f"Found devices at: {[hex(a) for a in existing_addresses]}")

# Example addresses for the three sensors
addresses = [0x52, 0x54, 0x56]
sensors = []

for i, pin in enumerate(sensor_pins):
    if addresses[i] not in existing_addresses:
        sensor = VL53L5CX()
        set_sensor_address(sensor, pin, addresses[i])
    else:
        sensor = VL53L5CX(i2c_address=addresses[i])
    sensors.append(sensor)

# Verify that all sensors are alive
for sensor in sensors:
    if not sensor.is_alive():
        raise IOError(f"VL53L5CX at address {hex(sensor.i2c_address)} is not alive.")

print("Initialising sensors...")
USE_8X8_MODE = True  # Change to False for 4x4

# Initialize sensors
for sensor in sensors:
    sensor.init()
    if USE_8X8_MODE:
        sensor.set_resolution(VL53L5CX_RESOLUTION_8X8)
    else:
        sensor.set_resolution(VL53L5CX_RESOLUTION_4X4)
    sensor.start_ranging()

print("Sensors initialized.")

# -----------------------------------------------------------------------------
# Helper Functions for 3D Points
# -----------------------------------------------------------------------------
# Field of View
FOV_DEG = 60
OFFSET_8X8 = 3.75
OFFSET_4X4 = 7.5

if USE_8X8_MODE:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_8X8, FOV_DEG/2 - OFFSET_8X8, 8)
    NUM_ZONES = 64
else:
    rows_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)
    cols_deg = np.linspace(-FOV_DEG/2 + OFFSET_4X4, FOV_DEG/2 - OFFSET_4X4, 4)
    NUM_ZONES = 16

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0

def get_3d_points(distances_mm: list[int], sensor_index: int) -> np.ndarray:
    """
    Convert the distance readings into (x, y, z) points in world coordinates.
    Applies a fixed offset towards the center of the robot to correct Z-axis discrepancies.
    """
    sensor_height_m = 0.75
    OFFSET_TOWARDS_CENTER = -0.5  # Adjust this value as needed (in meters)

    points = []
    grid_size = int(math.sqrt(NUM_ZONES))  # 8 or 4
    for i, dist_mm in enumerate(distances_mm):
        dist_m = dist_mm * 0.001
        row_idx = i // grid_size
        col_idx = i % grid_size

        # Angles for this zone
        vert_rad  = deg2rad(rows_deg[row_idx])
        horiz_rad = deg2rad(cols_deg[col_idx])

        # Spherical → Cartesian
        x = dist_m * math.cos(vert_rad) * math.cos(horiz_rad)
        y = dist_m * math.cos(vert_rad) * math.sin(horiz_rad)
        z = dist_m * math.sin(vert_rad)

        # Apply fixed offset towards the center of the robot
        x += OFFSET_TOWARDS_CENTER * math.cos(deg2rad(sensor_index * 60))
        y += OFFSET_TOWARDS_CENTER * math.sin(deg2rad(sensor_index * 60))

        # Shift for sensor height
        z += sensor_height_m

        points.append([x, y, z])
        
    points_np = np.array(points)

    if points_np.size > 0:
        # Determine the sensor orientation
        if sensor_index == 0:
            z_angle = -60.0   # Left sensor
        elif sensor_index == 2:
            z_angle = 60.0    # Right sensor
        else:
            z_angle = 0.0     # Forward sensor

        # Tilt angle for sensors
        tilt_angle = -30.0

        # Rotation matrices
        rot_y = np.array([
            [ math.cos(deg2rad(tilt_angle)), 0, math.sin(deg2rad(tilt_angle))],
            [ 0,                             1, 0                            ],
            [-math.sin(deg2rad(tilt_angle)), 0, math.cos(deg2rad(tilt_angle))],
        ])

        rot_z = np.array([
            [math.cos(deg2rad(z_angle)), -math.sin(deg2rad(z_angle)), 0],
            [math.sin(deg2rad(z_angle)),  math.cos(deg2rad(z_angle)), 0],
            [0,                          0,                           1],
        ])

        # Apply rotations
        points_np = points_np @ rot_y @ rot_z

    return points_np

# -----------------------------------------------------------------------------
# Occupancy Grid Parameters
# -----------------------------------------------------------------------------
GRID_MIN_X = -2.0  # meters
GRID_MAX_X = 2.0
GRID_MIN_Y = -2.0
GRID_MAX_Y = 2.0
GRID_RESOLUTION = 0.05  # 5cm per cell
OBSTACLE_HEIGHT_THRESHOLD = 0.1  # meters above ground
ROBOT_RADIUS = 0.2  # 200mm radius

def create_empty_grid() -> np.ndarray:
    """Create an empty occupancy grid."""
    grid_size_x = int((GRID_MAX_X - GRID_MIN_X) / GRID_RESOLUTION)
    grid_size_y = int((GRID_MAX_Y - GRID_MIN_Y) / GRID_RESOLUTION)
    return np.ones((grid_size_y, grid_size_x), dtype=np.uint8)  # 1 is free space

def world_to_grid(x: float, y: float) -> tuple[int, int]:
    """Convert world coordinates to grid coordinates."""
    grid_x = int((x - GRID_MIN_X) / GRID_RESOLUTION)
    grid_y = int((y - GRID_MIN_Y) / GRID_RESOLUTION)
    return grid_x, grid_y

def update_occupancy_grid(sensor_data: List[Dict]) -> np.ndarray:
    """Create occupancy grid from sensor data with robot size consideration."""
    grid = create_empty_grid()
    
    # First pass: Mark direct obstacle detections
    for sensor in sensor_data:
        for point in sensor["valid_points"]:
            x, y, z = point
            # Skip points outside our grid bounds
            if (GRID_MIN_X <= x <= GRID_MAX_X and 
                GRID_MIN_Y <= y <= GRID_MAX_Y):
                # If point is above our height threshold, mark as occupied
                if z > OBSTACLE_HEIGHT_THRESHOLD:
                    grid_x, grid_y = world_to_grid(x, y)
                    try:
                        grid[grid_y, grid_x] = 0  # 0 is occupied space
                    except IndexError:
                        continue
    
    # Second pass: Dilate obstacles by robot radius
    dilated_grid = grid.copy()
    robot_cells = int(ROBOT_RADIUS / GRID_RESOLUTION)  # Number of cells for robot radius
    
    # Find all obstacle cells
    obstacle_ys, obstacle_xs = np.where(grid == 0)
    
    # For each obstacle cell, mark surrounding cells within robot radius as occupied
    for obs_y, obs_x in zip(obstacle_ys, obstacle_xs):
        # Calculate bounds for the square region to check
        y_min = max(0, obs_y - robot_cells)
        y_max = min(grid.shape[0], obs_y + robot_cells + 1)
        x_min = max(0, obs_x - robot_cells)
        x_max = min(grid.shape[1], obs_x + robot_cells + 1)
        
        # Check each cell in the square region
        for y in range(y_min, y_max):
            for x in range(x_min, x_max):
                # Calculate distance to obstacle cell
                dist = np.sqrt((y - obs_y)**2 + (x - obs_x)**2) * GRID_RESOLUTION
                # If within robot radius, mark as occupied
                if dist <= ROBOT_RADIUS:
                    dilated_grid[y, x] = 0

    return dilated_grid

# -----------------------------------------------------------------------------
# Main Loop
# -----------------------------------------------------------------------------
print("Starting ToF read + MQTT publish loop...")
try:
    while True:
        all_sensor_data = []

        for s_idx, sensor in enumerate(sensors):
            try:
                if sensor.check_data_ready():
                    data = sensor.get_ranging_data()

                    # Ensure we have enough data before slicing
                    if len(data.distance_mm) >= NUM_ZONES and len(data.target_status) >= NUM_ZONES:
                        distances_mm = data.distance_mm[:NUM_ZONES]
                        target_status = data.target_status[:NUM_ZONES]

                        # Convert to 3D points in world coordinates
                        points_3d = get_3d_points(distances_mm, s_idx)

                        # Build a data structure for this sensor
                        # We'll also separate valid vs invalid points if you want
                        valid_points = []
                        invalid_points = []
                        for i, (dist_mm, status) in enumerate(zip(distances_mm, target_status)):
                            # Status code 5 typically means "valid" measurement on VL53L5CX
                            if status == 5 and dist_mm != 0:
                                valid_points.append(points_3d[i].tolist())
                            else:
                                invalid_points.append(points_3d[i].tolist())

                        sensor_data = {
                            "sensor_address": hex(sensor.i2c_address),
                            "sensor_index": s_idx,
                            "valid_points": valid_points,
                            "invalid_points": invalid_points,
                        }
                        all_sensor_data.append(sensor_data)
                    else:
                        print(f"Warning: Sensor {s_idx} returned incomplete data")
                        continue

            except IndexError as e:
                print(f"Error reading sensor {s_idx}: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error with sensor {s_idx}: {e}")
                continue

        # Publish all sensor data to MQTT as one JSON structure
        if all_sensor_data:
            # Update cache with new sensor data
            for sensor_data in all_sensor_data:
                s_idx = sensor_data["sensor_index"]
                if sensor_data["valid_points"]:  # Only cache if we have valid points
                    sensor_data_cache[s_idx] = sensor_data

            # Combine all cached sensor data
            combined_sensor_data = [
                data for data in sensor_data_cache.values() 
                if data is not None
            ]

            # Create occupancy grid from combined data
            occupancy_grid = update_occupancy_grid(combined_sensor_data)
            
            # Convert numpy array to list for JSON serialization
            grid_list = occupancy_grid.tolist()
            
            # Add grid to payload
            payload = json.dumps({
                "sensors": combined_sensor_data,  # Send all cached sensor data
                "occupancy_grid": {
                    "data": grid_list,
                    "height": len(grid_list),      # Add height
                    "width": len(grid_list[0]),    # Add width
                    "resolution": GRID_RESOLUTION,
                    "min_x": GRID_MIN_X,
                    "max_x": GRID_MAX_X,
                    "min_y": GRID_MIN_Y,
                    "max_y": GRID_MAX_Y
                }
            })
            client.publish(MQTT_TOPIC, payload)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Clean up
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()
    print("GPIO cleaned up, MQTT disconnected. Exiting.")
