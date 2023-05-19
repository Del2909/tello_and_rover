import socket
from pynput import keyboard

# Set up socket connection
HOST = 'remote-machine-ip-address'
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Define callback function for key release events
def on_release(key):
    message = None

    # Check if arrow key is released
    if key == keyboard.Key.up:
        message = 'Up arrow key released'
    elif key == keyboard.Key.down:
        message = 'Down arrow key released'
    elif key == keyboard.Key.left:
        message = 'Left arrow key released'
    elif key == keyboard.Key.right:
        message = 'Right arrow key released'

    # Send message to remote machine
    if message:
        sock.sendall(message.encode())

# Set up keyboard listener
with keyboard.Listener(on_release=on_release) as listener:
    listener.join()
