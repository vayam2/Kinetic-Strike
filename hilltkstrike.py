import time
import math
import requests
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from pymavlink import mavutil

# Google Elevation API Key (replace with your actual key)
API_KEY = 'YOUR_GOOGLE_ELEVATION_API_KEY'

# Connect to the vehicle
vehicle = connect('127.0.0.1:6969', wait_ready=True)

# Constants
SAFE_FLIGHT_ALTITUDE = 50  # Fly at a safe altitude (50 meters)
TARGET_DESCENT_THRESHOLD = 10  # Start descent when within 10 meters of target

def get_elevation(lat, lon):
    """Retrieve terrain elevation data from Google Elevation API."""
    url = f"https://maps.googleapis.com/maps/api/elevation/json?locations={lat},{lon}&key={API_KEY}"
    response = requests.get(url)
    elevation_data = response.json()

    if elevation_data['status'] == 'OK':
        elevation = elevation_data['results'][0]['elevation']
        return elevation
    else:
        print(f"Error retrieving elevation data: {elevation_data['status']}")
        return None

def arm_and_takeoff(target_altitude):
    """Arms the drone and takes off to a specified altitude."""
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        if altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def set_velocity_body(vx, vy, vz, yaw_rate=0.1):
    """Sets the drone's velocity in the body frame."""
    print(f"Setting velocity - vx: {vx}, vy: {vy}, vz: {vz}, yaw_rate: {yaw_rate}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame
        0b0000111111000111,  # Mask for velocity and yaw_rate
        0, 0, 0,             # x, y, z (position is not used)
        vx, vy, vz,          # Velocity (vx, vy, vz)
        0, 0, 0,             # Acceleration (ax, ay, az) - not used
        0, yaw_rate          # Yaw and yaw_rate
    )
    vehicle.send_mavlink(msg)

def get_distance_meters(target_location):
    """Calculates the distance in meters to the target location."""
    dlat = target_location.lat - vehicle.location.global_relative_frame.lat
    dlon = target_location.lon - vehicle.location.global_relative_frame.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

def adjust_altitude(target_location, current_altitude, target_elevation):
    """Adjusts the drone's altitude based on terrain elevation."""
    safe_target_altitude = SAFE_FLIGHT_ALTITUDE + target_elevation
    if current_altitude < safe_target_altitude:
        print(f"Ascending to safe altitude of {safe_target_altitude} meters")
        set_velocity_body(0, 0, 1)  # Ascend with a positive vertical speed (vz)
    elif current_altitude > safe_target_altitude + TARGET_DESCENT_THRESHOLD:
        print(f"Descending to target elevation of {safe_target_altitude} meters")
        set_velocity_body(0, 0, -1)  # Descend with a negative vertical speed (vz)
    else:
        print("Maintaining altitude")

def adjust_course_to_target(target_location, current_speed, target_elevation):
    """Adjusts the course and altitude to approach the target."""
    distance = get_distance_meters(target_location)
    
    if distance > 0:
        vx = current_speed  # Set forward speed
        vy = 0  # No sideways movement
        current_altitude = vehicle.location.global_relative_frame.alt
        adjust_altitude(target_location, current_altitude, target_elevation)
    else:
        vx = 0
        vy = 0
    
    # Apply velocity
    set_velocity_body(vx, vy, 0)

def trigger_final_mechanism():
    """Trigger the final strike mechanism (e.g., simulated or actual impact)."""
    print("Triggering final strike mechanism")

try:
    # Takeoff to a safe altitude first
    arm_and_takeoff(SAFE_FLIGHT_ALTITUDE)

    # Set the target location (latitude, longitude, and ground altitude are unknown)
    target_location = LocationGlobalRelative(-35.36230912, 149.16396965)  # Example coordinates
    
    # Retrieve target elevation using Google Elevation API
    target_elevation = get_elevation(target_location.lat, target_location.lon)
    if target_elevation is None:
        print("Unable to retrieve target elevation. Aborting.")
        exit()

    print(f"Target elevation is {target_elevation} meters")

    max_speed = 15  # Max speed of the UAV
    current_speed = 5  # Start with a lower speed

    while True:
        distance_to_target = get_distance_meters(target_location)
        if distance_to_target < 1.5:  # Within 1.5 meters of target
            trigger_final_mechanism()
            break
        
        if current_speed < max_speed:
            current_speed += 1  # Increment speed
        
        adjust_course_to_target(target_location, current_speed, target_elevation)
        time.sleep(0.1)

finally:
    print("Closing vehicle connection")
    vehicle.close()
