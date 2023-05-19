from djitellopy import Tello
import time
import math
import cv2
import numpy as np
from path_computation import find_path
import cv2.aruco as aruco

# Takes photo using bottom camera, saves as image parameter name
def take_photo(tello, image):

        tello.streamon()
        tello.set_video_direction(1)
        

        dist_z = tello.get_distance_tof()

        # take a picture with the bottom camera
        frame = tello.get_frame_read().frame

        dist_z2 = tello.get_distance_tof()

        # save the picture to a file
        cv2.imwrite(image, frame)


     
        # return average distance away from floor

        return (dist_z + dist_z2) / 2


def land(tello, camera_matrix, dist_coeffs):
        '''
        cam = cv2.VideoCapture(0)

        #defining the camera capture resolution and fps
        camera_width = 720
        camera_height = 580
        camera_frame_rate = 30

        cam.set(2,camera_width)
        cam.set(4,camera_height)
        cam.set(5,camera_frame_rate)
        '''
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        marker_size = 80 
        dist_z = tello.get_height()
        dist_x = 100
        dist_y = 100
        parameters = cv2.aruco.DetectorParameters()

        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 500
        parameters.adaptiveThreshConstant = 100
        parameters.polygonalApproxAccuracyRate = 0.1
        parameters.minMarkerPerimeterRate = 0.001
        parameters.maxMarkerPerimeterRate = 4
        parameters.errorCorrectionRate = 0.1
        parameters.minMarkerDistanceRate = 0.05
        parameters.minOtsuStdDev =  1
        parameters.perspectiveRemovePixelPerCell = 1
        condition_met = False

        parameters.minMarkerPerimeterRate = 0.01   # Decrease the minimum perimeter rate
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE   # Skip corner refinement
        parameters.polygonalApproxAccuracyRate = 0.01   # Decrease the polygonal approximation accuracy rate


        #parameters.perspectiveRemovePixelPerCell = 0.1
        # move drone until over center of rover and at specified height
        #tello.go_xyz_speed(0, 0, -30, 10)

        while dist_z > 50 or (dist_y > 3 or dist_y < -3) or (dist_x > 3 or dist_x < -3) :
            frame = tello.get_frame_read().frame
            #img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #gray_frame = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_frame = cv2.GaussianBlur(gray_frame, (3, 3), 0)
            gray_frame = cv2.adaptiveThreshold(gray_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 71, 0.5)

          #  _, gray_frame = cv2.threshold(img_blurred, 100, 255, cv2.THRESH_BINARY)

            #gray_frame = cv2.adaptiveThreshold(img_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            # Detect the ArUco markers in the frame
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, dist_coeffs)   
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
            print("IDs found: ", ids)
            if ids != None:
                if 0 in ids:
                    for id in range(len(ids)):
                        if id == 0:
                            rvec_marker = rvec[marker][0]
                            tvec_marker = tvec[marker][0]
                            R, _ = cv2.Rodrigues(rvec_marker)   
                            marker_pos_camera = np.array([0, 0, 0], dtype=np.float32)
                            marker_pos_world = np.linalg.inv(R) @ tvec_marker.reshape((3,1)) - np.linalg.inv(R) @ marker_pos_camera.reshape((3,1))


                            dist_z = tello.get_height()
                            dist_x = marker_pos_world[0][0]
                            dist_y = marker_pos_world[1][0]

                            if dist_z < 50 and (dist_y < 3 and dist_y > -3) and (dist_x < 3 or dist_x > -3):
                                condition_met = True  # Set the flag to True
                                break  

                            if dist_z > 40:
                                tello.go_xyz_speed(dist_x, dist_y, - 10, 10 )
                            else:
                                tello.go_xyz_speed(dist_x, dist_y, 0, 10)
                else:
                    tello.move_up(20)
                
            
            if condition_met:
                break 

            time.sleep(0.3)

            
        tello.land()
        


# this function creates a list of movement commands for use with the drone mover_right move_forwards commands
# to implement, still require if statements to deal with negative or positive (e.g. move right or left)
def move_drone(x, y):
        
        #movement commands have to be greater than 20, so to bring drone to exact center, if drone is within 20, it moves 20 away, to then move the exact amount to the center
        movements = []
        while x >= 4 or x <= -4 and y > 4 or y <= -4:
            if x > 0:
                if x < 20:
                    movements.append((20, 0))
                    x += 20
                    movements.append((-(x), 0))
                    x = 0
                else:
                    movements.append((-20, 0))
                    x -= 20
            if x < 0:
                if x > -20:
                    movements.append((20, 0))
                    x -= 20
                    movements.append(((x), 0))
                    x = 0
                else:
                    movements.append((-20, 0))
                    x += 20
            
            if y > 0:
                if y < 20:
                    movements.append((0, 20))
                    y += 20
                    movements.append((0, -(y)))
                    y = 0
                else:
                    movements.append((0, -20))
                    y -= 20
            if y < 0:
                if y > -20:
                    movements.append((0, 20))
                    y += 20
                    movements.append((0, (y)))
                    y = 0
                else:
                    movements.append((0, -20))
                    y += 20            
        
        return movements



# runs all required drone code
def run_drone():
    print("RUNNNING")
    fx = 433.44868
    fy = 939.895
    cx = 107
    cy = 318.9

    dist_coeffs = np.array([0.8333, 0.699, 0.455, 0.00159, -0.94509282], dtype=np.float32)



    fx = 130
    cx = 160.7
    fy = 131.9
    cy = 106

    dist_coeffs = np.array([ 0.53889391, -0.85682489, -0.01124153,  0.01029454,  0.41007014])


    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    dist_coeffs = np.array([ 0.02252689, -0.32137224, -0.00607589, -0.00284081,  0.19866972])



    fx = 141.15
    cx = 163.55
    fy = 141.5
    cy = 107.034

    dist_coeffs = np.array([ 0.32329846, -0.57971221,  0.00214554,  0.0186935 ,  0.27343211])

    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    dist_coeffs = np.array([ 0.02252689, -0.32137224, -0.00607589, -0.00284081,  0.19866972])


    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)



    # Initialise drone
    tello = Tello()
    tello.connect()

    # configure drone
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)  # 

    # takeoff
    tello.takeoff()
    image = "bottom_camera.jpg"

    #print("taking photo...")

    #height = take_photo(tello, image)

    print("photo taken")
    #height = 80
    print("HEIGHT: ", 600)
    
    #lands drone

    print("Landing...")
    land(tello, camera_matrix, dist_coeffs)
    print("landed")

    #computes path
    #parameters: image path, height of image, rover aruco id, destination aruco id, wall aruco id, camera matrix, distortion coefficients, size of grid to segment image into for path computation

    print("computing path...")
    path = find_path(image, height, 0, 1, 2, camera_matrix, dist_coeffs, compress_factor=50)
    print("path computed")


    print("PATH: ", path)

            


    #tello.disable_mission_pads()
    #tello.land()
    #tello.streamoff()
    #tello.end()





