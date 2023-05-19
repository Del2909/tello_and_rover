from threading import Thread
from rover_controller import RoverController
from video_receiver import VideoReceiver
# Define the IP address and port for the video stream
VIDEO_HOST = 'raspberry-pi-ip-address'
VIDEO_PORT = 8000

# Define the IP address and port for the rover controller
ROVER_HOST = 'raspberry-pi-ip-address'
ROVER_PORT = 5000

# Create instances of the VideoReceiver and RoverController classes
video_receiver = VideoReceiver(VIDEO_HOST, VIDEO_PORT)
rover_controller = RoverController(ROVER_HOST, ROVER_PORT)

# Start the VideoReceiver and RoverController on separate threads
video_thread = Thread(target=video_receiver.start)
rover_thread = Thread(target=rover_controller.start)

video_thread.start()
rover_thread.start()

# Wait for both threads to finish
video_thread.join()
rover_thread.join()