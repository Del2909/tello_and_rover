import datetime, threading, time
from gpiozero import RotaryEncoder

ppr = 341

rightEncoder = RotaryEncoder (10, 9, max_steps = 0)
leftEncoder = RotaryEncoder (27, 22, max_steps = 0)

def foo():
    next_call = time.time()
    while True:
        print (datetime.datetime.now())
        print("Right Encoder = {:0.0f}".format(rightEncoder.steps))
        print("Left Encoder = {:0.0f}".format(leftEncoder.steps))
        next_call = next_call+0.01;
        time.sleep(next_call - time.time())

timerThread = threading.Thread(target=foo)
#timerThread.daemon = True
timerThread.start()
