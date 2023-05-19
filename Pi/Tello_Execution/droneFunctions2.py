from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import cv2.aruco as aruco

#self.send_rc_control must be set to true after drone takes off and false when it lands
def update(tello, left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity, send_rc_control):
    """ Update routine. Send velocities to Tello.
    """
    if send_rc_control:
        tello.send_rc_control(left_right_velocity, for_back_velocity,
            up_down_velocity, yaw_velocity)

def landAutoAruco(tello, frame, prev_align_bit):
    #cv2.putText(frame, "detecting",(150,240),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    error_margin = 30
    #print("start Finding")
    detected,tvec,ids, corners=detectAruco(frame, tello)
    #print("output")
    yaw_velocity = 0
    
    send_rc_control = True

    z_dist=tello.get_distance_tof()
    
    speedAlignX = 10
    speedAlignY = 10
    #speedAlignZ = -10
    if detected==1:
        #detectCounter=0
        print("ids found",ids)
        #cv2.putText(frame, "aligning",(0,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        aligned=[0,0,0]
        command =[0,0,0]
        print("initiate align::::::::")

        if(abs(tvec[0])<100):
            speedAlignX =10
        
        if(abs(tvec[1])<100):
            speedAlignY = 10

        if(abs(tvec[2])<600):
            speedAlignZ = -10
        
        if tvec[0]>60:
            command[0] = -speedAlignX
            for_back_velocity = -speedAlignX  #move back
        elif tvec[0]<-60:
            command[0] = speedAlignX
            for_back_velocity = speedAlignX  #move front
        
        elif tvec[0]>error_margin:
            command[0] = -5
            for_back_velocity = -5  #move back
        elif tvec[0]<-error_margin:
            command[0] = 5
            for_back_velocity = 5  #move front
        else:
            aligned[0]= 1
            command[0] = 0
            for_back_velocity = 0
        
        
        if tvec[1]>60:
            left_right_velocity = -speedAlignY  # set left velocity
            command[1] = -speedAlignY
        elif tvec[1]<-60:
            left_right_velocity = speedAlignY # set right velocity
            command[1] = speedAlignY
        
        elif tvec[1]>error_margin:
            left_right_velocity = -5  # set left velocity
            command[1] = -5
        elif tvec[1]<-error_margin:
            left_right_velocity = 5 # set right velocity
            command[1] = 5
        else:
            aligned[1]=1
            command[1] = 0
            left_right_velocity=0
        
        if tvec[2]>1000:
            up_down_velocity = -25
        elif tvec[2]>450:
            command[2] = -10
            up_down_velocity = -10
        else:
            aligned[2]= 1
            command[2] = 0
            up_down_velocity = 0

        if (abs(tvec[0])<error_margin and abs(tvec[1])<error_margin) and (tvec[2]<500 and prev_align_bit==1):
                print("!!!!!!!!!!!!!!!!!!!!! !LLLLLLLLANANAD")
                prev_align_bit=0
                #cv2.putText(frame, "aLanding",(90,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                tello.land()
                send_rc_control = False
                return True, prev_align_bit

        if (aligned[0] & aligned[1] & aligned[2]) == 1:
            print("LLLLLLLLANANAD")
            for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity = resetDroneCommands()
            update(tello, left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity, send_rc_control)

            # time given to stabilise the drone
            time.sleep(2)
            #detected,tvec,ids=self.detectAruco()
            #cv2.putText(frame, "Land 1",(90,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            #if (detected==1):
            prev_align_bit = 1

    

        tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
        print(tvec_str)
        print("alignment::  ", aligned[0],aligned[1],aligned[2], z_dist)
        print("command::  ", command[0],command[1],command[2],tvec[2])
    
    if detected==0:
        #detectCounter+=1
        print("!!!!!!!!!!!!!!!!!!!!Reset")
        for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity = resetDroneCommands()
    
    update(tello, left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity, send_rc_control)

    return False, prev_align_bit

def resetDroneCommands():  
    return 0,0,0,0

def detectAruco(frame, tello):
    """
    For future work try to reinitialise the frame for the downward camera and front camera seperately
    """
    
    ### --- Detecting aruco marker and calculating distance
    #print("detecting")
    #marker width and height 
    marker_size = 150 #mm        #maybe in meters 

    #initialise camera calibration matrix

    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    camera_distortion = np.array([0.02252689, -0.32137224, -0.00607589, -0.00284081,0.19866972])    
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

    #define the aruco marker spec 
    #it is 4x4_250 series
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    #convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners, ids , rejected = aruco.detectMarkers(gray_frame,aruco_dict, camera_matrix, camera_distortion)

    
    """ #manual thresholding
    hsv=cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
    lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
    upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
    mask = cv2.inRange(hsv, lower, upper)

    cv2.imshow("threshh",mask)
    #detect aruco markers
    corners, ids , rejected = aruco.detectMarkers(mask,aruco_dict, camera_matrix, camera_distortion)   """

    tvec = [0,0,0]

    detected=0

    if ids is not None:
        print("IDS FOUND: ", ids)
        if 0 in ids:
            marker_size = 80
            print("Found Marker in mask")
            detected=1
            #print("ids found",ids)
            #draw box around the aruco markers
            #aruco.drawDetectedMarkers(frame,corners,ids)
            
            #get 3D pose of each and every aruco marker
            #get rotational and translational vectors
            rvec_list_all, tvec_list_all, _objPoints =aruco.estimatePoseSingleMarkers(corners,marker_size, camera_matrix,camera_distortion)
            rvec = rvec_list_all[0][0]

            index = np.where(ids == 0)[0]
            print(index)
            print(tvec_list_all)
            
            tvec = tvec_list_all[index][0][0]
            print(tvec)
            #tvec = tvec_list_all[0][0]
            tvec[0] -= 40
        
            #aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,30)
            tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
            rvec_str= "x_r=%4.0f y_r=%4.0f z_r=%4.0f"%(rvec[0],rvec[1],rvec[2])
            #print("rvec",rvec_str)
            #cv2.putText(frame,tvec_str,(10,20),cv2.FONT_HERSHEY_PLAIN,1.5,(0,0,255),2,cv2.LINE_AA)
        elif 3 in ids:
            marker_size = 150
            print("Found Marker in mask: ", ids)
            detected=1
            #print("ids found",ids)
            #draw box around the aruco markers
            #aruco.drawDetectedMarkers(frame,corners,ids)
            
            #get 3D pose of each and every aruco marker
            #get rotational and translational vectors
            rvec_list_all, tvec_list_all, _objPoints =aruco.estimatePoseSingleMarkers(corners,marker_size, camera_matrix,camera_distortion)
            rvec = rvec_list_all[0][0]

            index = np.where(ids == 3)[0]
            print(index)
            print(tvec_list_all)
            
            tvec = tvec_list_all[index][0][0]
            print(tvec)
            #tvec = tvec_list_all[0][0]
            tvec[0] -= 30
        
            #aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,30)
            tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
            rvec_str= "x_r=%4.0f y_r=%4.0f z_r=%4.0f"%(rvec[0],rvec[1],rvec[2])
        #else:
        #print("illa")
    return detected, tvec, ids, corners

