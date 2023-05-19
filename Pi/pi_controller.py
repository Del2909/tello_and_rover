import socket
import RPi.GPIO as GPIO
from Pi.tello_controller import TelloController

class PiController:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        # Set up GPIO pins for motor control
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(22, GPIO.OUT)
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(24, GPIO.OUT)

        # Set up socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen()

        # Accept incoming connections and receive messages
        self.conn, self.addr = self.sock.accept()
        print(f'Connected by {self.addr}')

        # Start listening for messages
        self.listen()

    def handle_message(self, data):
        direction = data.decode().strip()

        # Drive motors forward
        if direction == 'forward':
            GPIO.output(18, GPIO.HIGH)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
        # Drive motors backward
        elif direction == 'backward':
            GPIO.output(18, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)
        # Turn left
        elif direction == 'left':
            GPIO.output(18, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
        # Turn right
        elif direction == 'right':
            GPIO.output(18, GPIO.HIGH)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)
        # Activate Tello Controller
        elif direction == "spacebar":

            controller = TelloController()
            img = controller.start()
            self.conn.sendall(img.tobytes())

            # Send captured image back to rover controller


    def listen(self):
        while True:
            data = self.conn.recv(1024)
            if not data:
                break
            print(f'Received message: {data.decode()}')
            self.handle_message(data)

        # Clean up GPIO pins
        GPIO.cleanup()
