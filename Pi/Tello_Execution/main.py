
from threading import Thread
#from tello_controller import TelloController
#from Video_Controller import RoverController

from tello_commands import run_drone
from Rover_V3 import runRover
import time
# Define the IP address and port for the video stream
'''VIDEO_HOST = 'raspberry-pi-ip-address'
VIDEO_PORT = 8000

# Define the IP address and port for the rover controller
ROVER_HOST = 'raspberry-pi-ip-address'
ROVER_PORT = 5000

# Create instances of the VideoReceiver and RoverController classes
tello_controller = TelloController(VIDEO_HOST, VIDEO_PORT)
rover_controller = RoverController(ROVER_HOST, ROVER_PORT)

# Start the VideoReceiver and RoverController on separate threads
video_thread = Thread(target=video_receiver.start)
rover_thread = Thread(target=rover_controller.start)

video_thread.start()
rover_thread.start()

# Wait for both threads to finish
video_thread.join()
rover_thread.join()'''

try:
    movements = run_drone()
    #movements = [(0, 0, 1), (171.5372313503388, 0, 0), (0, -90, 0), (1112.6545904961395, 0, 0), (0, 90, 0), (2671.7176303350193, 0, 0), (0, 90, 0), (1311.9848020393156, 0, 0), (0, 0, 2)]
    
    #movements = [(0, 0, 1), (334.45438292460744, 0, 0), (0, -90, 0), (956.0545724908719, 0, 0), (0, 90, 0), (2482.885213156706, 0, 0), (0, 90, 0), (1321.0675800209817, 0, 0), (0, 0, 2)]
    #movements = [ (0,0,1),(0, -90, 0), (834.180516534058, 0, 0), (0, 90, 0), (2775.4141932834336, 0, 0), (0, 90, 0), (980.8750298567671, 0, 0),(0,0,2)]
    #movements = [(0, 0, 1), (509.29791400366287, 0, 0), (0, -90, 0), (1143.7953190821904, 0, 0), (0, 90, 0), (3036.0574302079513, 0, 0), (0, 90, 0), (1412.0911411940517, 0, 0), (0, 0, 2)]
    #print("MOVEMENTS: ", movements + (0,0,2))
    #time.sleep(60)
    #movements = [(571.3807777832913, 0, 0), (0, 90, 0), (871.4558327160037, 0, 0), (0, -90, 0), (1714.850236591327, 0, 0), (0, -90, 0), (767.9613949561266, 0, 0)]
    #movements = [(1000,0,0)]
    runRover(movements)
except KeyboardInterrupt():
    tello.land()
