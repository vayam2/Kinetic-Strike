# Imports

import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Gimbal
import time
import math
from pymavlink import mavutil

####################################################################################################
####################################################################################################

class KineticMotion:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)     
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        '''
        The above aruco marker detector shall be replaced with our personalised detection/tracking module.
        This is done in order to create personalised targets. Right now ArUco markers will be used for testing.
        '''
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)      # The following connection string '127.0.0.1:14550' should be replaced with the appropriate connection string of the UAV
        self.takeoff_altitude = 10      # Choose takeoff altitude according to your needs
        self.arm_and_takeoff(self.takeoff_altitude)
        self.strike_vel_magnitude = 15      # unit - m/s | set this velocity according to your system's maximum velocity
        self.send_velocity(1, 0, 0)
        print("sending initial velocity")

    def captureVideo(self):
        vid = cv2.VideoCapture(0)

        while vid.isOpened():
            ret, frame = vid.read()

            if ret:
                self.strike(frame)
                cv2.imshow("Doomsday POV", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        vid.release()
        cv2.destroyAllWindows()

    def strike(self, frame):
        height, width, _ = frame.shape
        print('####################')
        print(height, width)
        print('####################')
        # frame center
        frame_center_x = int(width / 2)
        frame_center_y = int(height / 2)
        frame_center = (frame_center_x, frame_center_y)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_points = self.detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:
            for id in range(len(ids)):
                marker_id = ids[id]

                if marker_id == 4:
                    marker_corners = corners[id][0]

                    # marker center
                    marker_center_x = int(np.mean(marker_corners[:, 0]))
                    marker_center_y = int(np.mean(marker_corners[:, 1]))
                    marker_center = (marker_center_x, marker_center_y)

                    # Displacement
                    displacement_x = marker_center_x - frame_center_x
                    displacement_y = marker_center_y - frame_center_y
                    displacement = (displacement_x, displacement_y)
                    print("Pixel Displacement : ", displacement)

                    # P controller coefficients
                    p_x = 0.0025
                    p_y = 0.003
                    p_yaw = 0.005
                    p_pitch = 0.004

                    # Flags for x and y coordinates
                    flag_x = 0
                    flag_y = 0
                    flag_yaw = 0
                    flag_pitch = 0

                    # Calculating Displacement
                    displacement_x = marker_center_x - frame_center_x
                    displacement_y = marker_center_y - frame_center_y
                    displacement = (displacement_x, displacement_y)
                    print("Pixel Displacement : ", displacement)

                    # unit vectors
                    displacement_unit_x = displacement_x / (math.sqrt(displacement_x**2 + displacement_y**2))
                    displacement_unit_y = displacement_y / (math.sqrt(displacement_x**2 + displacement_y**2))

                    # calculating gimbal angles and velocity along respective coordinates
                    if displacement_x != 0:
                        flag_x = 1
                        yaw = int(p_x * displacement_x)
                    else:
                        flag_x = 0
                        yaw = 0

                    if displacement_y != 0:
                        flag_y = 1
                        pitch = int(p_y * displacement_y * -1)
                    else:
                        flag_y = 0
                        pitch = 0

                    vel_x = self.strike_vel_magnitude * displacement_unit_x
                    vel_y = self.strike_vel_magnitude * displacement_unit_y

                    # sending respective commands
                    self.vehicle.gimbal.rotate(pitch, 0, yaw)
                    self.send_velocity(vel_x, vel_y, 0)

                    cv2.circle(frame, marker_center, 5, (0, 255, 0), -1) # Draw center point
                    cv2.putText(frame, str(marker_id), marker_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) # Display marker ID near the center point

        else:
            pass

        cv2.imshow("Doomsday POV", frame)
        cv2.waitKey(1)

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

####################################################################################################
####################################################################################################

# Driver Code

if __name__ == "__main__":
    try:
        gimbal_tracker = KineticMotion()
        gimbal_tracker.captureVideo()
    
    except KeyboardInterrupt:
        print("Code Over Manually")