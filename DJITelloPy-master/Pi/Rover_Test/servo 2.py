from gpiozero import Servo
from time import sleep

leftServo = Servo (6, min_pulse_width = 0.0005, max_pulse_width = 0.002)
rightServo = Servo (5, min_pulse_width = 0.0005, max_pulse_width = 0.002)

while True:
	
	leftServo.value = 1
	sleep(1)
	leftServo.value = 0
	sleep(1)
	leftServo.value = -1
	sleep(1)
	leftServo.value = 0
	sleep(1)
	rightServo.value = 1
	sleep(1)
	rightServo.value = 0
	sleep(1)
	rightServo.value = -1
	sleep(1)
	rightServo.value = 0
	sleep(5)
