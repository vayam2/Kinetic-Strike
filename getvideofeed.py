import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
import dronekit
from pymavlink import mavutil 
# Connect to the vehicle
vehicle = connect('127.0.0.1:6969', wait_ready=True)

def arm_and_takeoff(target_altitude):
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
    print(f"Setting velocity - vx: {vx}, vy: {vy}, vz: {vz}, yaw_rate: {yaw_rate}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                # time_boot_ms
        0,                # target_system (default)
        0,                # target_component (default)
        dronekit.mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only velocity and yaw_rate enabled)
        0,                # x (not used)
        0,                # y (not used)
        0,                # z (not used)
        vx,               # vx
        vy,               # vy
        vz,               # vz
        0,                # ax (not used)
        0,                # ay (not used)
        0,                # az (not used)
        0,                # yaw (not used)
        yaw_rate          # yaw_rate
    )
    vehicle.send_mavlink(msg)
    time.sleep(1)  # Allow time for the velocity to be applied


def get_distance_meters(target_location):
    dlat = target_location.lat - vehicle.location.global_relative_frame.lat
    dlon = target_location.lon - vehicle.location.global_relative_frame.lon
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

def calculate_yaw_to_target(target_location):
    current_location = vehicle.location.global_relative_frame
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    yaw = math.atan2(dlon, dlat) * (180 / math.pi)  # Convert from radians to degrees
    return yaw

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.
    roll, pitch, and yaw are in radians.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    q = np.empty(4)
    q[0] = cy * cp * cr + sy * sp * sr  # w
    q[1] = cy * cp * sr - sy * sp * cr  # x
    q[2] = sy * cp * sr + cy * sp * cr  # y
    q[3] = cy * sp * cr - sy * cp * sr  # z
    
    return q

def get_current_yaw():
    """
    Get the current yaw angle of the UAV.
    """
    # Fetch the current heading from the vehicle's global relative frame
    # Assuming the heading is provided in degrees
    return vehicle.heading

def condition_yaw(degree, relative=True):
    """
    Rotate the UAV to a specific yaw angle.
    """
    if relative:
        is_relative = 1  # yaw relative to current position
    else:
        is_relative = 0  # yaw is an absolute angle
    print("degree", degree)
    if degree >=0 :
        msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            degree,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative/absolute
            0, 0, 0)  # param 5-7 not used
    else: 
        degree =  degree *-1
        print("CCW" , degree)
        msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            degree,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            -1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative/absolute
            0, 0, 0)  # param 5-7 not used
    # Send the command to the vehicle
    vehicle.send_mavlink(msg)

def align_heading_to_target(target_location):
    print("Aligning heading to target")
    current_yaw = get_current_yaw()
    target_yaw = calculate_yaw_to_target(target_location)
    
    # Compute the relative yaw adjustment
    relative_yaw = target_yaw - current_yaw
    if relative_yaw > 180:
        relative_yaw -= 360
    elif relative_yaw < -180:
        relative_yaw += 360
    
    print(f"Current yaw: {current_yaw}, Target yaw: {target_yaw}, Relative yaw: {relative_yaw}")
    condition_yaw(relative_yaw)


def adjust_course_to_target(target_location, current_speed):
    current_location = vehicle.location.global_relative_frame
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    distance = get_distance_meters(target_location)
    
    if distance > 0:
        vx = 15
        t = distance/ 15
        vz = vehicle.location.global_relative_frame.alt/t
        print(vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <0.1:
            vehicle.armed = False
        vy = 0
    else:
        vx = 0
        vy = 0
    #print(f"Calculated velocities - vx: {vx}, vy: {vy}, vz: {vz}")
    set_velocity_body(vx, vy, vz)

def trigger_final_mechanism():
    print("Triggering final strike mechanism")

try:
    arm_and_takeoff(10)  # Takeoff to 20 meters
    target_location = LocationGlobalRelative(  -35.36241187, 149.16389342 , 10)  # Example target at ground level
    
    align_heading_to_target(target_location)# Align the heading towards the target
    time.sleep(10)
    target_location = LocationGlobalRelative(  -35.36241187, 149.16389342 , 10)  # Example target at ground level
    
    align_heading_to_target(target_location)# Align the heading towards the target
    time.sleep(10)
    vehicle.mode = VehicleMode("LAND")
except:
    print("vehicle not yaw")
