from djitellopy import Tello
import time
import math
import cv2
import numpy as np
from path_computation import find_path
import cv2.aruco as aruco
import threading
from droneFunctions2 import landAutoAruco
from droneFunctions2 import detectAruco

# Takes photo using bottom camera, saves as image parameter name


def take_photo(tello, image):

        dist_z = tello.get_distance_tof()
        
        
        tello.move_up(280-dist_z)
        tello.move_forward(90)
        counter = 0
        while True:
            counter += 1
            dist_z = tello.get_distance_tof()
            # take a picture with the bottom camera
            frame = tello.get_frame_read()

            dist_z2 = tello.get_distance_tof()

            
            
            
            frame = frame.frame
            
            detected, tvec, ids, corners = detectAruco(frame, tello)
            
            cv2.imshow("window",frame)

            print(ids)
            if ids is not None:
                print("IDs: ", ids)         
                if 2 in ids and 3 in ids and len(ids) >= 7:
                #if 0 in ids and 2 in ids and len(ids) == 5:
                    print((dist_z + dist_z2) /2)
                    print("ALL IDs Detected")

                    break
                    
            if counter % 150 == 0:
                tello.move_down(30)
            if counter % 300 == 0:
                tello.move_up(60)
                
        tello.move_back(90)


        # save the picture to a file
        print("writing")
        cv2.imwrite(image + "_uncropped", frame)

        frame = frame[0:240,0:320]
        cv2.imwrite(image, frame)
        #path = find_path(image, height, 0, 1, 2, camera_matrix, dist_coeffs, compress_factor=50)
        # return average distance away from floor

        return (dist_z + dist_z2) / 2


def land(tello, camera_matrix, dist_coeffs):
 
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

        marker_size = 150

        dist_x = 100
        dist_y = 100
        dist_z = 100

        condition_met = False

        # move drone until over center of rover and at specified height
        print("CHECK")
        prev_align_bit = 0
        while condition_met == False:

            frame_read = tello.get_frame_read()
            frame = frame_read.frame
            #gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

            #detect aruco markers
       
            condition_met,prev_align_bit = landAutoAruco(tello, frame,prev_align_bit)
        

def keep_drone_alive(tello):
    try:
        while True:
            tello.rotate_clockwise(30)
            time.sleep(3)  # Send the command every 4 seconds to prevent landing

    except KeyboardInterrupt():
        tello.land()


# runs all required drone code
def run_drone():

    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    dist_coeffs = np.array([ 0.02252689, -0.32137224, -0.00607589, -0.00284081,  0.19866972])


       
    fx = 130
    cx = 160.7
    fy = 131.9
    cy = 106

    dist_coeffs = np.array([ 0.53889391, -0.85682489, -0.01124153,  0.01029454,  0.41007014])
    

    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    fly = True

    try:

     # Initialise drone
        tello = Tello()
        tello.connect()
        tello.streamon()
        tello.set_video_direction(1)
        

        # configure drone
        tello.enable_mission_pads()
        tello.set_mission_pad_detection_direction(2)  # 

        # takeoff

        if fly:
            tello.takeoff()
        image = "bottom_camera.jpg"


        # Function to send a 'keep-alive' command to the drone
        # Start the 'keep-alive' thread
        #keep_alive_thread = threading.Thread(target=keep_drone_alive, args=(tello,))
        #keep_alive_thread.daemon = True
        #keep_alive_thread.start()

        print("taking photo...")

        height = take_photo(tello, image)

        print("photo taken")
        height = 80
        print("HEIGHT: ", 600)
        
        #lands drone

        print("Landing...")

        if fly:
            land(tello, camera_matrix, dist_coeffs)
        print("landed")

    except (KeyboardInterrupt()):
        print("STOP")
        tello.land()

    #computes path
    #parameters: image path, height of image, rover aruco id, destination aruco id, wall aruco id, camera matrix, distortion coefficients, size of grid to segment image into for path computation

    print("computing path...")  
    path = find_path(image, height, 3, 1, 2, camera_matrix, dist_coeffs)
    print("path computed")


    return path


   # print("PATH: ", path)

            


    #tello.disable_mission_pads()
    #tello.land()
    #tello.streamoff()
    #tello.end()





