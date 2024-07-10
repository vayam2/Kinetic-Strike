from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

vehicle = connect("127.0.0.1:6969", wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def send_velocity(vx, vy, vz):
    """
    Send velocity command to the vehicle.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def create_file_with_content(filename):
    try:
        with open(filename, 'w') as file:
            takeoff_alti = 50
            arm_and_takeoff(takeoff_alti)
            vel_z = 15

            while vel_z > 5:
                send_velocity(0, 0, vel_z)
                time.sleep(1)
                file.write(vehicle.velocity)
                time.sleep(1)
                send_velocity(0, 0, -vel_z)
                time.sleep(1)
                file.write(vehicle.velocity)
                time.sleep(1)
                vel_z -= 1

        print(f"File '{filename}' has been created successfully with the given content.")
    except IOError:
        print(f"Error: Could not create file '{filename}'.")

filename = 'velocityZLog.txt'
create_file_with_content(filename)
