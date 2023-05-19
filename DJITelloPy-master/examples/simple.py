from djitellopy import Tello
import cv2
tello = Tello()

tello.connect()
#tello.takeoff()

#tello.move_left(20)
#tello.rotate_clockwise(90)
#tello.move_forward(20)
#tello.enable_mission_pads()
#tello.set_mission_pad_detection_direction(0)


tello.streamon()
# Set video direction to downwards
tello.set_video_direction(1)

# Initialize OpenCV video stream
cv2.namedWindow("Drone Feed")
drone_feed = cv2.VideoCapture("udp://0.0.0.0:11111")

print("bosh1")


# Loop to continuously receive and display video frames
while True:
    ret, frame = drone_feed.read()
    #if not ret:
    #    break

    cv2.imshow("Drone Feed", frame)
    
    print("bosh2")

    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("bosh3")

        # Cleanup
        drone_feed.release()
        cv2.destroyAllWindows()
        tello.streamoff()
        tello.land()
    print("bosh4")

print("bosh5")






#tello.rotate_anticlockwise(90)

