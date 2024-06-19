import cv2
import numpy as np 
import time
from dronekit import connect, Gimbal
from pymavlink import mavutil

class GimbalController:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)  #127.0.0.1:14550       

    def captureVideo(self):
        vid = cv2.VideoCapture(0)

        while vid.isOpened():
            ret, frame = vid.read()

            if ret:
                self.gimbalTrack(frame)
                cv2.imshow("Gimbal Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        vid.release()
        cv2.destroyAllWindows()

    def gimbalTrack(self, frame):
        height, width, _ = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)

        corners, ids, rejected_pts = self.detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i]

                if marker_id == 4:
                    marker_corners = corners[i][0]
                    
                    # marker center
                    marker_center_x = int(np.mean(marker_corners[:, 0]))
                    marker_center_y = int(np.mean(marker_corners[:, 1]))
                    marker_center = (marker_center_x, marker_center_y)

                    # frame center
                    frame_center_x = int(height / 2)
                    frame_center_y = int(width / 2)

                    # displacements along both axes
                    displacement_x = marker_center_x - frame_center_x
                    displacement_y = marker_center_y - frame_center_y
                    displacement = (displacement_x, displacement_y)
                    print("Pixel Displacement : ", displacement)

                    
                    # displacement corrections
                    flag_x = 0
                    flag_y = 0

                    p_x = 0.5
                    p_y = 0.3
                    
                    # yaw
                    if displacement_x != 0:
                        flag_x = 1
                        yaw = int(p_x * displacement_x * 1)
                    else:
                        flag_x = 0
                        yaw = 0

                    # pitch
                    if displacement_y != 0:
                        flag_y = 1
                        pitch = int(p_y * displacement_y * -1)
                    else:
                        pitch = 0
                        flag_y = 0

                    # sending commands
                    self.vehicle.gimbal.rotate(pitch, 0, yaw)

                    if flag_x == 0 and flag_y == 0:
                        print("Gimbal Aligned!")
                    
                    cv2.circle(frame, marker_center, 5, (0, 255, 0), -1) # Draw center point
                    cv2.putText(frame, str(marker_id), marker_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) # Display marker ID near the center point

        else:
            pass

        cv2.imshow("Gimbal Tracking", frame)
        cv2.waitKey(1)

# Driver Code
if __name__ == "__main__":
    try:
        gimbal_tracker = GimbalController()
        gimbal_tracker.captureVideo()
    
    except KeyboardInterrupt:
        print("Code Over Manually")
