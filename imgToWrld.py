import numpy as np
import cv2

def image_to_world_coordinates(u, v, mtx, dist, rvec, tvec):
    # Convert to numpy arrays
    uv_point = np.array([[u, v]], dtype=np.float64)

    # Undistort the image point (optional)
    undistorted_uv_point = cv2.undistortPoints(uv_point, mtx, dist)

    # Extract undistorted coordinates
    u_ud = undistorted_uv_point[0][0][0]
    v_ud = undistorted_uv_point[0][0][1]

    # Normalize image coordinates
    x_norm = (u_ud - mtx[0][2]) / mtx[0][0]
    y_norm = (v_ud - mtx[1][2]) / mtx[1][1]

    # Convert to homogeneous coordinates
    image_point_norm = np.array([[x_norm, y_norm, 1.0]], dtype=np.float64).T

    # Compute inverse rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    R_inv = np.linalg.inv(R)

    # Compute 3D coordinates in the world frame
    tvec = np.array(tvec).reshape((3, 1))
    world_point = np.dot(R_inv, image_point_norm) - np.dot(R_inv, tvec)

    return world_point.flatten()


# Driver Code:
if __name__ == '__main__':
    # Assuming 'u' and 'v' are pixel coordinates in the image
    u = 320
    v = 240

    # Example intrinsic and extrinsic parameters (replace with your own values)
    mtx = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float64)  # Camera matrix
    dist = np.array([0.1, -0.2, 0, 0, 0], dtype=np.float64)  # Distortion coefficients
    rvec = np.array([[0.1, 0.2, 0.3]], dtype=np.float64)  # Rotation vector
    tvec = np.array([[1, 2, 3]], dtype=np.float64)  # Translation vector

    world_coords = image_to_world_coordinates(u, v, mtx, dist, rvec, tvec)
    print("3D World Coordinates:", world_coords)