import numpy as np
import cv2

def calibrate_camera(video_file, grid_size, square_size):
    # Criteria for termination of the iterative process of corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (6,5,0)
    objp = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2) * square_size

    # Arrays to store object points and image points from all the images
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    # Read the video file
    cap = cv2.VideoCapture(video_file)

    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, grid_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            frame = cv2.drawChessboardCorners(frame, grid_size, corners2, ret)

        # Display the frame
        cv2.imshow('Video Calibration', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release everything if job is finished
    cap.release()
    cv2.destroyAllWindows()

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Print and store calibration results
    print("Camera Matrix (Intrinsic parameters):\n", mtx)
    print("\nDistortion Coefficients:\n", dist)
    print("\nRvecs:\n", rvecs)
    print("\nTvecs:\n", tvecs)

    # Save calibration results to a file (you can modify this as needed)
    np.savez('calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# Driver Code
if __name__ == '__name__':
    video_file = 'input_video.avi'  # write address of the input video file
    grid_size = (9, 7)  # Size of the calibration pattern (inner corners of chessboard)
    square_size = 2.35  # Size of each square in your defined unit(cm)
    calibrate_camera(video_file, grid_size, square_size)