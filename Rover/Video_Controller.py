import socket
import io
import picamera

# Set up socket connection to send video feed
HOST = 'first-machine-ip-address'
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Set up camera object
with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 24
    stream = io.BytesIO()

    # Continuously capture video and send to first machine when arrow key is pressed
    while True:
        camera.capture(stream, format='jpeg', use_video_port=True)
        data = stream.getvalue()
        stream.seek(0)
        stream.truncate()

        # Check for arrow key press and send video feed to first machine
        try:
            conn, addr = sock.accept()
            print(f'Connected by {addr}')
            while True:
                message = conn.recv(1024)
                if not message:
                    break
                elif message == b'up':
                    sock.sendall(data)
        except:
            pass
