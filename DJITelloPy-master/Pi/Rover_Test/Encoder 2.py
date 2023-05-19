import time
import numpy as np
#from utils import plot_line
from gpiozero import RotaryEncoder

# FIT0521
ppr = 341

# TODO
# Verify program would not block encoder readings

encoder = RotaryEncoder (19, 26, max_steps = 0)

while True:	
	print("Started, sleeping now")
	time.sleep(5)
	print("Turn = {:0.0f}".format(encoder.steps))
encoder.close()

	
