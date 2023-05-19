import curses
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory
import MotorClass as MC
from gpiozero import Servo, Motor

screen = curses.initscr()
curses.noecho() 
curses.cbreak()
screen.keypad(True)

#GPIO.setmode(GPIO.BCM)

motorSpeed = 0.8

#M1 = MC.Motor(17,10)
rightMotor = Motor(17,10)
leftMotor = Motor(22,27)
#M2 = MC.Motor(27,22)

factory = PiGPIOFactory()
leftServo = Servo (19, min_pulse_width = 0.001, max_pulse_width = 0.005, pin_factory = factory)
rightServo = Servo (26, min_pulse_width = 0.001, max_pulse_width = 0.005, pin_factory = factory)

try:
    temp = 1
    while temp:   
        char = screen.getch()
        if char == ord('p'):
            #GPIO.cleanup()
            break
        elif char == curses.KEY_UP:
            print("up")
            leftMotor.forward(motorSpeed)
            rightMotor.forward(motorSpeed)
            #M1.forward(speed)
            #M2.forward(speed)
        elif char == curses.KEY_DOWN:
            print("down")
            leftMotor.backward(motorSpeed)
            rightMotor.backward(motorSpeed)
            #M1.backward(speed)
            #M2.backward(speed)
        elif char == curses.KEY_RIGHT:
            print("right")
            leftMotor.forward(motorSpeed)
            rightMotor.backward(motorSpeed)
            #M1.backward(speed)
            #M2.forward(speed)
        elif char == curses.KEY_LEFT:
            print("left")
            leftMotor.backward(motorSpeed)
            rightMotor.forward(motorSpeed)
            #M1.forward(speed)
            #M2.backward(speed)
        elif char == ord('s'):
            print("stop") 
            leftMotor.stop() 
            rightMotor.stop()  
            #M1.stop()
            #M2.stop()
        elif char == ord('i'):
            motorSpeed += 0.1
            if motorSpeed > 1:
                motorSpeed = 1
            print(motorSpeed)
        elif char == ord('u'):
            motorSpeed -= 0.1
            if motorSpeed < 0.5:
                motorSpeed = 0.5
            print(motorSpeed)
        elif char == ord('q'):
            leftServo.value = -1
        elif char == ord('w'):
            leftServo.value = 0
        elif char == ord('e'):
            leftServo.value = 1
        elif char == ord('r'):
            rightServo.value = 1
        elif char == ord('t'):
            rightServo.value = 0
        elif char == ord('y'):
            rightServo.value = -1
finally:
    print("Closed")
    #Close down curses properly, inc turn echo back on!
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()

