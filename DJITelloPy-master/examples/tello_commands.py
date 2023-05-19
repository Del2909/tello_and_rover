from djitellopy import Tello
import time
import math
import cv2

from path_computation import find_path

from image_distance import compute_distance
# create and connect



def round_to_nearest_5(x):
        return round(x / 5) * 5


def take_photo(tello, image):
        pad = tello.get_mission_pad_id()
        print(pad)

        dist_x = tello.get_mission_pad_distance_x()
        dist_y = tello.get_mission_pad_distance_y()
        dist_z = tello.get_mission_pad_distance_z()

        print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
        print("PAD DETECTED: ", pad)

        # take a picture with the bottom camera
        frame = tello.get_frame_read().frame

        # save the picture to a file
        cv2.imwrite(image, frame)

        pad = tello.get_mission_pad_id()
        dist_x2 = tello.get_mission_pad_distance_x()
        dist_y2 = tello.get_mission_pad_distance_y()
        dist_z2 = tello.get_mission_pad_distance_z()
        print("Distance` pad in X: ",  dist_x2, "Y: ", dist_y2, "Z: ", dist_z2)
        print("PAD DETECTED: ", pad)
        
        time.sleep(1)
        return (dist_z + dist_z2) / 2


def land(tello):
        pad = tello.get_mission_pad_id()
        dist_x = tello.get_mission_pad_distance_x()
        dist_y = tello.get_mission_pad_distance_y()
        dist_z = tello.get_mission_pad_distance_z()
        print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
        print("PAD DETECTED: ", pad)
        last_z = 100

        '''while dist_z > 40 or (dist_y > 3 or dist_y < -3) or (dist_x > 3 or dist_x < -3) :
                pad = tello.get_mission_pad_id()
                if pad == -1:
                    tello.move_up(20)
                else:      
                    if dist_z > 25:
                        tello.go_xyz_speed_mid(0, 0, dist_z - 10, 10, pad)
                    else:
                        tello.go_xyz_speed_mid(0, 0, dist_z, 10, pad)
                    #tello.go_xyz_speed_mid(0, dist_y, dist_z, 10, pad)
                    #tello.go_xyz_speed_mid(dist_x, dist_y, dist_z, 10, pad)
                    #if dist_z < 80:
                    #    tello.go_xyz_speed(dist_x, dist_y, dist_z, 10)        
                    #else:
                    #    tello.go_xyz_speed(dist_x, dist_y, dist_z, 10)        
                
                time.sleep(1)


                
                pad = tello.get_mission_pad_id()
                dist_x = tello.get_mission_pad_distance_x()
                dist_y = tello.get_mission_pad_distance_y()
                dist_z = tello.get_mission_pad_distance_z()
                print("Distance` pad in X: ",  dist_x, "Y: ", dist_y, "Z: ", dist_z)
                print("PAD DETECTED: ", pad)
            '''
        tello.land()



try:

    tello = Tello()
    tello.connect()


    # configure drone
    tello.enable_mission_pads()

    tello.streamon()
    tello.set_video_direction(1)
    tello.set_mission_pad_detection_direction(2)  # 

    # takeoff
    tello.takeoff()

   # image = "bottom_camera.jpg"
    image = "C:/Users\peter\Documents\Bio inspired robotics\telloside\DJITelloPy-master\examples\bottom_camera.jpg"
    height = take_photo(tello, image)

    land(tello)

    path = find_path(image, height, 0, 1, 2)


    print(path)

            


    print("STOP")
    tello.disable_mission_pads()
    tello.land()
    tello.streamoff()
    tello.end()
except KeyboardInterrupt:
    tello.land()
    print("NOT WORKING2")






def move_drone(x, y):
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