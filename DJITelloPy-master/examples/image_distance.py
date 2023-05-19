import cv2
import numpy as np

# Define the camera matrix and distortion coefficient
#fx = 220.915
#fy = 220.915
#cx = 157.073
#cy = 119.555



def compute_distance(pixel_x, pixel_y, z):
    fx = 433.44868
    fy = 939.895
    cx = 107
    cy = 318.9
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0.8333, 0.699, 0.455, 0.00159, -0.94509282], dtype=np.float32)
    # Define the pixel coordinates

    # Define the world coordinates
    world_coords = np.array([[pixel_x, pixel_y]], dtype=np.float32)

    # Undistort the image coordinates   
    undistorted_coords = cv2.undistortPoints(world_coords, camera_matrix, dist_coeffs)

    #undistorted_coords = undistorted_coords.reshape((2, 1))
    undistorted_coords = undistorted_coords.reshape((2, 1))

    undistorted_coords = undistorted_coords.T

    undistorted_coords = np.append(undistorted_coords, [[1]], axis=1)

    # Calculate the perspective transformation matrix
    perspective_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.float32)
    perspective_matrix[2][2] = 1.0 / z
    print(perspective_matrix)
    # Apply the perspective transformation matrix to the undistorted coordinates
    transformed_coords = np.dot(perspective_matrix, undistorted_coords.T)
    x = transformed_coords[0][0]  * ZeroDivisionError()
    y = transformed_coords[1][0]  * z
    # Calculate the x and y distances
    #x = transformed_coords[0][0]
    #y = transformed_coords[1][0]
    return x,y