# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


import json
import time
import numpy as np
import math


from lib.odrive_uart import ODriveUART, reset_odrive
from lib.imu import FilteredMPU6050

def test_motor_direction():
    # Initialize IMU
    imu = FilteredMPU6050()
    imu.calibrate()
    
    # Reset ODrive before initializing motors
    reset_odrive()
    time.sleep(3)  # Wait for ODrive to reset
    
    motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=1, dir_right=1)
    directions = {'left': 1, 'right': 1}

    angle_threshold = 5.0  # degrees
    max_spin_duration = 5.0  # seconds

    for name in ['left', 'right']:
        time.sleep(1)
        print(f"\nTesting {name} motor...")

        # Start motor in velocity mode and clear errors
        if name == 'left':
            motor_controller.start_left()
            motor_controller.enable_velocity_mode_left()
            if motor_controller.check_errors_left():
                print("Clearing left motor errors...")
                motor_controller.clear_errors_left()
        else:
            motor_controller.start_right()
            motor_controller.enable_velocity_mode_right()
            if motor_controller.check_errors_right():
                print("Clearing right motor errors...")
                motor_controller.clear_errors_right()

        # Spin motor (e.g., at 30 RPM)
        if name == 'left':
            motor_controller.set_speed_rpm_left(30)
        else:
            motor_controller.set_speed_rpm_right(30)

        # Get initial yaw
        _, _, initial_yaw = imu.get_orientation()
        start_time = time.time()

        # Wait until we exceed angle_threshold or hit max_spin_duration
        while True:
            time.sleep(0.01)
            _, _, current_yaw = imu.get_orientation()
            angle_diff = current_yaw - initial_yaw

            if abs(angle_diff) >= angle_threshold:
                break
            if (time.time() - start_time) > max_spin_duration:
                print("Reached max spin duration without hitting angle threshold.")
                break

        # Stop the motor
        if name == 'left':
            motor_controller.stop_left()
        else:
            motor_controller.stop_right()

        print(f"Yaw difference before stopping: {angle_diff:.2f} deg")

        # Determine final direction based on sign of yaw difference
        if abs(angle_diff) < angle_threshold:
            print("Angle change too small; defaulting to forward (+1).")
            directions[name] = 1
        else:
            if name == 'left':
                directions['left'] = -1 if angle_diff > 0 else 1
            else:
                directions['right'] = 1 if angle_diff > 0 else -1

        time.sleep(0.5)

    # Save direction results
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'w') as f:
        json.dump(directions, f)

    print("\nDirection test complete!")
    print(f"Left direction: {directions['left']}, Right direction: {directions['right']}")
    print(f"Results saved to ~/quickstart/lib/motor_dir.json: {directions}")


if __name__ == '__main__':
    try:
        test_motor_direction()
    except Exception as e:
        print(f"Error occurred: {e}")
