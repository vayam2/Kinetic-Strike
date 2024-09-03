import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
import dronekit
# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)

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

def set_yaw(yaw):
    print(f"Setting yaw to: {yaw}")
    yaw_rad = math.radians(yaw)  # Convert yaw from degrees to radians
    q = euler_to_quaternion(0, 0, yaw_rad)  # Convert yaw to quaternion

    # Send MAVLink message for attitude target
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,                # time_boot_ms
        0,                # target system
        0,                # target component
        0b00000100,       # type_mask (only yaw is enabled)
        q.tolist(),        # Quaternion [w, x, y, z]
        0,                # body_roll_rate
        0,                # body_pitch_rate
        0,                # body_yaw_rate
        0                 # thrust
    )
    vehicle.send_mavlink(msg)
    time.sleep(1)  # Give some time for the yaw to update



def align_heading_to_target(target_location):
    print("Aligning heading to target")
    yaw = calculate_yaw_to_target(target_location)
    set_yaw(yaw)

def adjust_course_to_target(target_location, current_speed):
    current_location = vehicle.location.global_relative_frame
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    distance = get_distance_meters(target_location)
    
    if distance > 0:
        vx = 15
        t = distance/ 15
        vz = vehicle.location.global_relative_frame.alt/t
        vy = 0
    else:
        vx = 0
        vy = 0
    print(f"Calculated velocities - vx: {vx}, vy: {vy}, vz: {vz}")
    set_velocity_body(vx, vy, vz)

def trigger_final_mechanism():
    print("Triggering final strike mechanism")
    # Example placeholder code:
    # strike_servo.angle = 180
    # OR
    # GPIO.output(trigger_pin, GPIO.HIGH)

try:
    arm_and_takeoff(10)  # Takeoff to 20 meters
    target_location = LocationGlobalRelative(-35.363261, 149.165230, 0)  # Example target at ground level
    
    align_heading_to_target(target_location)  # Align the heading towards the target
    time.sleep(5)
    max_speed = 15
    acceleration = 1
    current_speed = 5
    
    while True:
        distance_to_target = get_distance_meters(target_location)
        if distance_to_target < 1.5:
            trigger_final_mechanism()
            break
        
        if current_speed < max_speed:
            current_speed += acceleration * 0.5
        
        adjust_course_to_target(target_location, current_speed)
        time.sleep(0.1)

finally:
    print("Closing vehicle connection")
    vehicle.close()
