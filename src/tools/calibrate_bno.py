#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration script for the BNO086 (BNO08x family) IMU.
Triggers the sensor's self-calibration routine, prints live status,
and saves the calibration profile to the sensor's internal flash memory.
"""

import time
import sys

import adafruit_bno08x

from src.sensors import bno086
from src.sensors import i2c_bus

def run_calibration():
    print("Initializing BNO086...")
    if not bno086.initialize():
        print("Error: Could not initialize BNO086 sensor.")
        sys.exit(1)

    sensor = bno086.sensor
    print("BNO086 initialized successfully.")

    # IMPORTANT: `sensor.calibration_status` returns the *magnetometer* accuracy
    # (adafruit_bno08x updates it only when a BNO_REPORT_MAGNETOMETER report is
    # processed). initialize() only enables the rotation vector, so without the
    # magnetometer feature enabled the status is stuck at 0 forever no matter how
    # much you move the robot. Enable magnetometer (+ game rotation vector, which
    # is what the self-calibration actually tunes) so status updates flow.
    with i2c_bus.LOCK:
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
    time.sleep(0.5)

    print("\n--- Starting Calibration Mode ---")
    sensor.begin_calibration()
    print("Self-calibration triggered.")
    
    print("\nCalibration instructions:")
    print("1. GYROSCOPE: Place the robot on a flat, vibration-free surface and keep it completely still for 5 seconds.")
    print("2. ACCELEROMETER: Slowly rotate the robot so that it rests on each of its 6 sides (wheels down, wheels up, left side, right side, front nose, back tail) for a few seconds each.")
    print("3. MAGNETOMETER: Move/rotate the robot in a figure-8 motion in the air.")
    print("\nCalibration status levels: 0=Unreliable, 1=Low, 2=Medium, 3=High")
    print("Press Ctrl+C at any time to save the calibration data and exit.")
    
    try:
        last_status = -1
        while True:
            # Poll the calibration status
            status = sensor.calibration_status
            if status != last_status:
                print(f"[{time.strftime('%H:%M:%S')}] Calibration Status changed to: {status}")
                last_status = status
                
            if status == 3:
                print("\nCalibration has reached HIGH accuracy!")
                break
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
        
    print("\nSaving calibration data to sensor flash memory...")
    try:
        sensor.save_calibration_data()
        print("Calibration data saved successfully! This will persist across power cycles.")
    except Exception as e:
        print(f"Error saving calibration data: {e}")

if __name__ == "__main__":
    run_calibration()
