import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()
#tello.takeoff()
#tello.set_video_encoder_rate(2)
tello.streamon()

# Set video direction to downwards
tello.set_video_direction(1)

# Initialize OpenCV video stream
cv2.namedWindow("Drone Feed")
drone_feed = cv2.VideoCapture("udp://0.0.0.0:11111")



lower_white = (0, 0, 150)
upper_white = (255, 30, 255)


# Loop to continuously receive and display video frames
while True:
    ret, frame = drone_feed.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Threshold the image to detect white objects
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    #Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Draw bounding boxes around detected objects
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)







    cv2.imshow("Drone Feed", frame)


    


    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
drone_feed.release()
cv2.destroyAllWindows()
tello.streamoff()
<<<<<<< HEAD
tello.land()
=======
tello.land()
>>>>>>> ca0eb6c775deef0ec777bd03fa3304cfc0f15aba
