# import the necessary modules
import cv2
from djitellopy import Tello

# initialize the Tello drone
tello = Tello()

tello.connect()
tello.streamon()
tello.set_video_direction(1)

#tello.takeoff()


# take a picture with the bottom camera
frame = tello.get_frame_read().frame

# save the picture to a file
cv2.imwrite("bottom_camera.jpg", frame)



#tello.land()

# disconnect from the Tello drone
tello.streamoff()
tello.disconnect()
