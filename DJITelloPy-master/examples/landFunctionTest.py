from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import cv2.aruco as aruco



# Speed of the drone
S = 60
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120
hsvVals=[0,0,95,179,0,255]

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
        - M: land on the mission pad
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.

    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.img=0
        self.send_rc_control = False
        self.detectCounter=0

        # create update timer
        # updates every 1000//120 seconds
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)
        
        # self defined variables
        self.toggle_autonomous = 0
        self.prev_align_bit = 0

    # function that continuosly runs to retrieve user inputs(keystrokes) and 
    def run(self):

        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()
        # set the downward camera ON
        self.tello.set_video_direction(1)

        frame_read = self.tello.get_frame_read()
        self.img=frame_read
        #print(type(frame_read))

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    print("!!!!!!!!!!!!!", event.key)
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    elif event.key == pygame.K_m:
                        print("M is pressed") 
                    else:
                        self.toggle_autonomous = 0 
                        self.keydown(event.key)
                        print('keydown',event.key)
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_m:
                        self.toggle_autonomous = 1 
                        print("M is released")
                    elif event.key == pygame.K_p:
                        self.resetDroneCommands()
                    self.keyup(event.key)
                    print('keyUP',event.key)

            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            self.frame = frame_read.frame
            #self.detectAruco()

            if self.toggle_autonomous == 1:
                #print("i guess dis is it")
                self.landAutoAruco()


            # Video feed window initialisation:
            # battery 
            text = "Battery: {}%".format(self.tello.get_battery())
            cv2.putText(self.frame, text,(0,240),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            self.frame = np.rot90(self.frame)
            self.frame = np.flipud(self.frame)

            self.frame = pygame.surfarray.make_surface(self.frame)
            self.screen.blit(self.frame, (0, 0))
            #pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key

        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key

        Resets the velocities to 0 once key is released
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            #self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            not self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello.
        """
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                self.up_down_velocity, self.yaw_velocity)
            
    def thresholding(img):
 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
 
        lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
 
        upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
 
        mask = cv2.inRange(hsv, lower, upper)
 
        return mask

    def resetDroneCommands(self):
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        #print('Iwork')

    def landAutoAruco(self):
        cv2.putText(self.frame, "detecting",(150,240),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        error_margin = 30
        #print("start Finding")
        detected,tvec,ids=self.detectAruco()
        #print("output")
        tvec[1] -=40
        z_dist=self.tello.get_height()
        
        speedAlignX = 10
        speedAlignY = 10
        speedAlignZ = -10
        if detected==1:
            self.detectCounter=0
            print("ids found",ids)
            cv2.putText(self.frame, "aligning",(0,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            aligned=[0,0,0]
            command =[0,0,0]
            print("initiate align::::::::")

            if(abs(tvec[0])<100):
                speedAlignX =10
            
            if(abs(tvec[1])<100):
                speedAlignY = 10

            if(abs(tvec[2])<600):
                speedAlignZ = -10

            if tvec[0]>error_margin:
                command[0] = -speedAlignX
                self.for_back_velocity = -speedAlignX  #move back
            elif tvec[0]<-error_margin:
                command[0] = speedAlignX
                self.for_back_velocity = speedAlignX  #move front
            else:
                aligned[0]= 1
                command[0] = 0
                self.for_back_velocity = 0

            if tvec[1]>error_margin:
                self.left_right_velocity = -speedAlignY  # set left velocity
                command[1] = -speedAlignY
            elif tvec[1]<-error_margin:
                self.left_right_velocity = speedAlignY # set right velocity
                command[1] = speedAlignY
            else:
                aligned[1]=1
                command[1] = 0
                self.left_right_velocity=0
            
            if tvec[2]>450:
                command[2] = -10
                self.up_down_velocity = -10
            else:
                aligned[2]= 1
                command[2] = 0
                self.up_down_velocity = 0

            if (abs(tvec[0])<error_margin and abs(tvec[1])<error_margin) and (tvec[2]<500 and self.prev_align_bit==1):
                    print("!!!!!!!!!!!!!!!!!!!!! !LLLLLLLLANANAD")
                    self.prev_align_bit=0
                    cv2.putText(self.frame, "aLanding",(90,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    #not self.tello.land()
                    self.send_rc_control = False

            if (aligned[0] & aligned[1] & aligned[2]) == 1:
                print("LLLLLLLLANANAD")
                self.resetDroneCommands()
                # time given to stabilise the drone
                print(tvec[0],tvec[1])
                time.sleep(10)
                #detected,tvec,ids=self.detectAruco()
                cv2.putText(self.frame, "Land 1",(90,100),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                #if (detected==1):
                self.prev_align_bit = 1


            tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
            print(tvec_str)
            print("alignment::  ", aligned[0],aligned[1],aligned[2], z_dist)
            print("command::  ", command[0],command[1],command[2],tvec[2])
            #time.sleep(0.3)
        
        if detected==0:
            self.detectCounter+=1
            #print("not Acting:") 
            #if (self.detectCounter>=1):
                #can make it go higher again
            print("!!!!!!!!!!!!!!!!!!!!Reset")
            self.resetDroneCommands()
            
        
        

            
    def detectAruco(self):
        """
        For future work try to reinitialise the frame for the downward camera and front camera seperately
        """
        
        ### --- Detecting aruco marker and calculating distance
        #print("detecting")
        #marker width and height 
        marker_size = 80 #mm        #maybe in meters 

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
        gray_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        corners, ids , rejected = aruco.detectMarkers(gray_frame,aruco_dict, camera_matrix, camera_distortion)

        
        """ #manual thresholding
        hsv=cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
        upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
        mask = cv2.inRange(hsv, lower, upper)

        cv2.imshow("threshh",mask)
        #detect aruco markers
        corners, ids , rejected = aruco.detectMarkers(mask,aruco_dict, camera_matrix, camera_distortion)   """
        z_dist=self.tello.get_height()

        tvec = [0,0,0]

        detected=0
        print("Found Marker in mask not found :",ids)
        if ids is not None:
            print("Found Marker in mask")
            detected=1
            #print("ids found",ids)
            #draw box around the aruco markers
            aruco.drawDetectedMarkers(self.frame,corners,ids)
            
            #get 3D pose of each and every aruco marker
            #get rotational and translational vectors
            rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners,marker_size,camera_matrix,camera_distortion)
            
            rvec = rvec_list_all[0][0]
            tvec = tvec_list_all[0][0]
        
            aruco.drawAxis(self.frame,camera_matrix,camera_distortion,rvec,tvec,30)
            tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
            rvec_str= "x_r=%4.0f y_r=%4.0f z_r=%4.0f"%(rvec[0],rvec[1],rvec[2])
            #print("rvec",rvec_str)
            cv2.putText(self.frame,tvec_str,(10,20),cv2.FONT_HERSHEY_PLAIN,1.5,(0,0,255),2,cv2.LINE_AA)
        
        #else:
           #print("illa")
        return detected, tvec, ids

def main():
    frontend = FrontEnd()

    # run frontend

    frontend.run()


if __name__ == '__main__':
    main()
