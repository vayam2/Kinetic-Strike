#!/usr/bin/env python3

# Imports

import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

############################################################################################################################################
############################################################################################################################################

class Striker:
    def __init__(self, camera_matrix, distortion_array, rotation_vector, translation_vector):
        # Camera Parameters
        '''
        these camera parameters are required to convert the image 2D coordinates to 3D world coordinates.
        formula used for this is theoretical and will be in testing.
        '''
        self.camera_matrix = camera_matrix
        self.distortion_array = distortion_array
        self.rotation_vector = rotation_vector
        self.translation_vector = translation_vector

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
        '''
        
        vid = cv2.VideoCapture(0)

        while vid.isOpened():
            ret, frame = vid.read()
            if ret:
                self.strike(frame)
                cv2.imshow("Strike Cam", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def strike(self, frame):
        height, width, _ = frame.shape

        # Frame center coordinates
        frame_center_x = int(height / 2)
        frame_center_y = int(width / 2)
        frame_center = (frame_center_x, frame_center_y)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)    # converting RGB frame to GrayScale frame

        corners, ids, rejected_pts = self.detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:
            for id in range(len(ids)):
                marker_id = ids[id]

                if marker_id == 4:
                    self.target_detected = True
                    marker_corner = corners[id][0]

                    # marker center
                    marker_center_x = int(np.mean(marker_corner[:, 0]))
                    marker_center_y = int(np.mean(marker_corner[:, 1]))
                    marker_center = (marker_center_x, marker_center_y)

                    # calculating gimbal angles and NED velocity
                    (pitch_angle, roll_angle, yaw_angle) = self.gimbalAngleCalculation(frame_center, marker_center)
                    (vel_x, vel_y, vel_z) = self.calculateUAVVelocity(marker_center)
                    
                    # sending gimbal and velocity commands
                    self.vehicle.gimbal.rotate(pitch_angle, roll_angle, yaw_angle)
                    self.send_velocity(vel_x, vel_y, vel_z)

        else:
            self.target_detected = False
            self.gimbalSearch(self.rotate_direction)

    def calculateUAVVelocity(self, marker_center):
        '''
        converts image coordinates of the target to world coordinates.
        use these world coordinates to calculate velocity
        
        input --> marker_center : (marke_center_x, marker_center_y)
        
        return type --> velocity along each axis : (vel_x, vel_y, vel_z)
        '''

        img_coor = np.array([marker_center], dtype=np.float64)

        # Undistort the image point (optional)
        img_coor = cv2.undistortPoints(img_coor, self.camera_matrix, self.distortion_array)

        # Extract undistorted coordinates
        img_x_ud = img_coor[0][0][0]
        img_y_ud = img_coor[0][0][1]

        # Normalize image coordinates
        img_x_norm = (img_x_ud - self.camera_matrix[0][2]) / self.camera_matrix[0][0]
        img_y_norm = (img_y_ud - self.camera_matrix[1][2]) / self.camera_matrix[1][1]

        # Convert to homogeneous coordinates
        image_point_norm = np.array([[img_x_norm, img_y_norm, 1.0]], dtype=np.float64).T

        # Compute inverse rotation matrix
        r_vec, _ = cv2.Rodrigues(self.rotation_vector)
        r_vec_inv = np.linalg.inv(r_vec)

        # Compute 3D coordinates in the world frame
        t_vec = np.array(self.translation_vector).reshape((3, 1))
        world_point = np.dot(r_vec_inv, image_point_norm) - np.dot(r_vec_inv, t_vec)

        world_coor_3D = world_point.flatten()

        disp_x = world_coor_3D[0]
        disp_y = world_coor_3D[1]
        disp_z = world_coor_3D[2]

        disp_unit_x = disp_x / math.sqrt(disp_x**2 + disp_y**2 + disp_z**2)
        disp_unit_y = disp_y / math.sqrt(disp_x**2 + disp_y**2 + disp_z**2)
        disp_unit_z = disp_z / math.sqrt(disp_x**2 + disp_y**2 + disp_z**2)

        vel_x = disp_unit_x * self.strike_velocity_magnitude
        vel_y = disp_unit_y * self.strike_velocity_magnitude
        vel_z = disp_unit_z * self.strike_velocity_magnitude

        return (vel_x, vel_y, vel_z)

    def gimbalAngleCalculation(self, frame_center, marker_center):
        '''
        calculate gimbal angles to be provided
        NOTE : the gimbal is 2D. therefore roll is fixed. can be changed accordingly
        
        input --> frame_center : (frame_center_x, frame_center_y)
              --> marker_center : (marker_center_x, marker_center_y)
        
        return type --> (pitch, roll, yaw)
        '''
        
        # pixel displacement
        displacement_x = marker_center[0] - frame_center[0]
        displacement_y = marker_center[1] - frame_center[1]
        displacement = (displacement_x, displacement_y)
        print("Displacement : ", displacement)

        # P controller coefficients
        p_pitch = 0.03
        p_yaw = 0.028

        pitch_angle = 0
        yaw_angle = 0
        if displacement_x != 0:
            yaw_angle = int(displacement_x * p_yaw * 1)
        if displacement_y != 0:
            pitch_angle = int(displacement_y * p_pitch * 1)
        
        return (pitch_angle, 0, yaw_angle)

    def gimbalSearch(self, rotate_direction):
        '''
        keeps gimbal at an pitch angle of 45 degrees and rotates aklternatively in CW and CCW direction, searching for the target.
        
        input - rotate_direction : bool
                if rotate_direction == false --> rotate in CW direction till extreme
                else if rotate_direction == true --> rotate in CCW direction till extreme
                
        return type --> void
        '''
        if self.vehicle.gimbal.yaw >= 155:
            rotate_direction = False
        elif self.vehicle.gimbal.yaw <= -155:
            rotate_direction = True

        pitch_angle = -45

        if rotate_direction:
            yaw_angle = 160
        else:
            yaw_angle = -160

        self.vehicle.gimbal.rotate(pitch_angle, 0, yaw_angle)

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

############################################################################################################################################
############################################################################################################################################

# Driver Code
if __name__ == "__main__":
    try:
        camera_matrix = []
        distortion_array = []
        rotation_vector = []
        translation_vector = []

        kamikaze = Striker(camera_matrix, distortion_array, rotation_vector, translation_vector)
        kamikaze.captureVideo()

    except KeyboardInterrupt:
        print("Code Shutdown Manually")