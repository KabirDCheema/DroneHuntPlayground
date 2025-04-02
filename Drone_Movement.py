#!/usr/bin/env python3
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to Pixhawk via serial port
connection_string = '/dev/serial0,921600'
vehicle = connect(connection_string, wait_ready=True, baud=921600)

def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= target_altitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_position_and_velocity(position_x, position_y, position_z, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms
        0, 0,    # target system, target component
        8,       # coordinate frame
        0b0000111111000000,  # position and velocity control
        position_x, position_y, position_z, # position
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, # acceleration (not used)
        0, 0)    # yaw, yaw_rate
    vehicle.send_mavlink(msg)

def send_ned_position(position_x, position_y, position_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms
        0, 0,    # target system, target component
        8,       # coordinate frame
        0b0000111111000111,  # position and velocity control
        position_x, position_y, position_z, # position
        0, 0, 0,
        0, 0, 0, # acceleration (not used)
        0, 0)    # yaw, yaw_rate
    vehicle.send_mavlink(msg)

# Main execution

