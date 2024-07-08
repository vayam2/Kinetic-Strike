#!/bin/usr/env python3

# Imports

import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command, Gimbal
from pymavlink import mavutil

###################################################################################################
###################################################################################################

class KineticStrike:
    def __init__(self):
        # Detection Parameters
        '''
        replace the aruco marker detection parameters with personalised detection parameter.
        this will lead in personalised code structure and more abstraction.
        '''
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # UAV parameters
        self.vehicle = connect("127.0.0.1:14550", wait_ready = True)    # the connection string must be replaced accoring to how we want to connect the UAV. 127.0.0.1:14550 is default.
        self.takeoff_altitude = 15    # unit metres

        # Testing Initial functions
        self.target_detected = False
        self.rotate_direction = False
        self.strike_velocity_magnitude = 15    # unit - m/s
        self.arm_and_takeoff(self.takeoff_altitude)    # takeoff command sent to the UAV
        self.gimbalSearch(self.rotate_direction)

    def captureVideo(self):
        '''
        takes camera feed as input frame by frame and sends the frame to be processed to the main function
        input - none
        return type - none
        '''

        vid = cv2.VideoCapture(0)

        while vid.isOpened():
            ret, frame = vid.read()

            if ret:
                self.strike()
                cv2.imshow("Doomsday Cam", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_velocity(self, vx, vy, vz):
        """
        Send velocity command to the vehicle.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
    

###################################################################################################
###################################################################################################

# Driver Code
if __name__ == '__main__':
    try:
        pass
    except KeyboardInterrupt:
        print("Code Killed Manually")