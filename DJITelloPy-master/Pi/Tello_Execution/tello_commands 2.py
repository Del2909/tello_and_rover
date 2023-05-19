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


        while True:
            dist_z = tello.get_distance_tof()
            # take a picture with the bottom camera
            frame = tello.get_frame_read()

            dist_z2 = tello.get_distance_tof()

            
            frame = frame.frame

            detected, tvec, ids = detectAruco(tello, frame)


            
            if ids is not None:
                print("IDs: ", ids)         
                #if 1 in ids and 2 in ids and len(ids) == 5:
                if 0 in ids and 2 in ids and len(ids) == 5:
                    print((dist_z + dist_z2) /2)
                    break
                    print("ALL IDs Detected")
                    
                


        # save the picture to a file
        print("writing")

        cv2.imwrite(image, frame)
        #path = find_path(image, height, 0, 1, 2, camera_matrix, dist_coeffs, compress_factor=50)
        # return average distance away from floor

        return (dist_z + dist_z2) / 2


def land(tello, camera_matrix, dist_coeffs):
 
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

        marker_size = 15

        dist_x = 100
        dist_y = 100
        dist_z = 100

        condition_met = False

        # move drone until over center of rover and at specified height
        print("CHECK")

        while condition_met == False:

            frame_read = tello.get_frame_read()
            frame = frame_read.frame
            #gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

            #detect aruco markers
            '''corners, ids , rejected = aruco.detectMarkers(gray_frame,aruco_dict, camera_matrix, dist_coeffs)              
            rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners,marker_size,camera_matrix,dist_coeffs)

            if ids is not None:
                print("IDs found: ", ids)
       
                
                if 0 in ids:
                    for x in range(len(ids)):
                        if ids[x] == 0:
                       
                            dist_x = int(tvec_list_all[x][0][0])
                            dist_y = int(tvec_list_all[x][0][1])
                            dist_z = tello.get_distance_tof()
                            

                            dist_y -= 4

                            if dist_z < 50 and (dist_y < 3 and dist_y > -3) and (dist_x < 3 or dist_x > -3):
                                condition_met = True  # Set the flag to True
                                #tello.send_rc_control(0,0,0,0)
                                break  



                            if dist_z > 40:
                                #tello.send_rc_control((dist_x/abs(dist_x)) * 30, (dist_y/abs(dist_y)) * 30, -10, 0)
                                tello.go_xyz_speed(-dist_x, -dist_y, 40, 10 )
                                print("MOVE")

                            else:
                                #tello.send_rc_control((dist_x/abs(dist_x)) * 30, (dist_y/abs(dist_y)) * 30, 0, 0)
                                print("MOVE")

                                tello.go_xyz_speed(-dist_x, -dist_y, 40, 10)
                else:
                    #tello.move_up(20)
                    print("Move up")
                
                time.sleep(1)
            
            if condition_met:
                break 

        tello.land()'''
            condition_met = landAutoAruco(tello, frame)
        

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


    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    fly = False

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
    path = find_path(image, height, 0, 1, 2, camera_matrix, dist_coeffs)
    print("path computed")


   # print("PATH: ", path)

            


    #tello.disable_mission_pads()
    #tello.land()
    #tello.streamoff()
    #tello.end()





