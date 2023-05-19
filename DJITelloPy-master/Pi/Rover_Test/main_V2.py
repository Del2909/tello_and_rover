from smbus import SMBus
import datetime, threading, time
import math
import random
from gpiozero import Button, Servo, Motor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import LSM6DSO
import pygame

####################### SETUP ####################### 

factory = PiGPIOFactory()

pygame.init()
window = pygame.display.set_mode((300, 300))

##### IMU
i2cbus = SMBus(1)
LSM6DSOAddr = 0x6B
INT1 = Button(13, pull_up = False, bounce_time = 0.001, pin_factory = factory)  
INT2 = Button(6, pull_up = False, bounce_time = 0.001, pin_factory = factory)  
acc = (0.0, 0.0, 0.0)
gyro = (0.0, 0.0, 0.0)
gyroRead = 0
accRead = 0

cummulativeAngle = 0.0
yawAcc = 0.0

prevDistDiff = 0
sumDistDiff = 0

gyroTimeNew = 0.0
gyroTimeOld = 0.0


##### Motors
rightMotorSpeed = 0.62
leftMotorSpeed = 0.62
#rightMotorSpeed = 0.57
#leftMotorSpeed = 0.57
rightMotor = Motor(26,19)
leftMotor = Motor(21,20)

# Encoder Current Value
rightEncoderVal = 0
leftEncoderVal = 0

# Previous Encoder Value
prevRightStep = 0
prevLeftStep = 0

# Encoder PID
prevRightStepError = 0
sumRightStepError = 0
prevLeftStepError = 0
sumLeftStepError = 0

rightEncoderBuf = 0
leftEncoderBuf = 0


dist = 0
distSetPoint = 9000

ppr = 373
rightEncoder = RotaryEncoder (10, 9, max_steps = 0, pin_factory = factory)
leftEncoder = RotaryEncoder (22, 27, max_steps = 0, pin_factory = factory)

move = 0
turn = 0
right = 0
left = 0

prevTime = 0
currTime = 0

underSpeedCount = 0

speedPreset = 33
turnSetPoint = 90
turnAngle = 0

prevEncoderError = 0

angleReset = 0
prevAngle = 0
complete = 0

##### Charging Plates

leftServo = Servo (12, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)
rightServo = Servo (16, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)

rightEndSw = Button(14, pull_up = False, bounce_time = 0.1, pin_factory = factory)
rightTravelSw = Button(15, pull_up = False, bounce_time = 0.1, pin_factory = factory)
leftEndSw = Button(7, pull_up = False, bounce_time = 0.1, pin_factory = factory)
leftTravelSw = Button(18, pull_up = False, bounce_time = 0.1, pin_factory = factory)

####################### FUNCTIONS ####################### 

##### IMU
def LSM6DSO_readAcc():
	global acc, accRead
	accRead = 1
	x = LSM6DSO.readAccX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readAccY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readAccZ(LSM6DSOAddr, i2cbus)
	acc = (x, y, z)
	return (x, y, z)
	
def LSM6DSO_readGyro():
	global gyro, gyroTimeNew, gyroTimeOld, gyroRead
	gyroRead = 1;
	gyroTimer = time.time()
	x = LSM6DSO.readGyroX(LSM6DSOAddr, i2cbus)
	y = LSM6DSO.readGyroY(LSM6DSOAddr, i2cbus)
	z = LSM6DSO.readGyroZ(LSM6DSOAddr, i2cbus)
	gyroTimeOld = gyroTimeNew
	gyroTimeNew = time.time()
	#print("Gyro Read")
	gyro = (x, y, z)
	return (x, y, z)
	
def IMU_Gyro_Cal():
	global gyroRead, gyro
	gyroCal = []
	i = 0
	while i < 100:
		if gyroRead:
			gyroCal.append(gyro)
			gyroRead = 0
			i += 1
			print(i)
	x = 0.0;
	y = 0.0;
	z = 0.0;
	for j in range(len(gyroCal)):
		temp = gyroCal[j]
		x += temp[0]
		y += temp[1]
		z += temp[2]
	
	return (x/(i+1), y/(i+1), z/(i+1))

def IMU_Acc_Cal():
	global accRead, acc
	AccCal = []
	i = 0
	while i < 100:
		if accRead:
			AccCal.append(acc)
			accRead = 0
			i += 1
			print(i)
	x = 0.0;
	y = 0.0;
	z = 0.0;
	for j in range(len(AccCal)):
		temp = AccCal[j]
		x += temp[0]
		y += temp[1]
		z += temp[2]
	
	return (x/(i+1), y/(i+1), z/(i+1))
	
INT1.when_pressed = LSM6DSO_readAcc
INT2.when_pressed = LSM6DSO_readGyro

movementComplete = 0
rightForward = 0
rightBackward = 0
leftForward = 0
leftBackward = 0

chargeCount = 0

def rightServoStop():
    global movementComplete, rightForward, rightBackward, chargeCount
    rightServo.value = 0
    chargeCount += 1
    rightForward = 0
    rightBackward = 0
    print("End Reached")
    
def leftServoStop():
    global movementComplete, leftForward, leftBackward, chargeCount
    leftServo.value = 0
    chargeCount += 1
    leftForward = 0
    leftBackward = 0
    print("End Reached")

def servoMove():
    global movementComplete, rightForward, rightBackward, leftForward, leftBackward
    if ~movementComplete:
        if rightForward:
            rightServo.value = 0.3
            movementComplete = 1
        elif rightBackward:
            rightServo.value = -0.6
            movementComplete = 1
        if leftForward:
            leftServo.value = 0.3
            movementComplete = 1
        elif leftBackward:
            leftServo.value = -0.6
            movementComplete = 1

def chargingMechanism(charge):
    global movementComplete, rightForward, rightBackward, leftForward, leftBackward
    if charge == 1:
        rightForward = 1
        leftForward = 1
        movementComplete = 0
        servoMove()
    elif charge == 2:
        rightBackward = 1
        leftBackward = 1
        movementComplete = 0
        servoMove()
        

rightEndSw.when_pressed = rightServoStop
rightTravelSw.when_pressed = rightServoStop
leftEndSw.when_pressed = leftServoStop
leftTravelSw.when_pressed = leftServoStop


# ENCODER UPDATES



def getEncoder():
    global move, turn, rightEncoderBuf, leftEncoderBuf, rightEncoderVal, leftEncoderVal
    if move or turn:
        rightEncoderVal = rightEncoder.steps - rightEncoderBuf
        leftEncoderVal = leftEncoder.steps - leftEncoderBuf
    else:
        print("Encoder Reset")
        rightEncoderBuf = rightEncoder.steps
        leftEncoderBuf = leftEncoder.steps

def stepCalibrate(Kp, Ki, Kd, currCount, prevCount, presetCount, prevError, sumError):
    # +ve = CCW rotation
    diff = presetCount - (abs(currCount) - abs(prevCount))
    #print("Corrected: %0.04f, %0.04f, %0.04f" %(yawKp*distDiff, yawKd*(distDiff-prevDistDiff), yawKi*sumDistDiff))
    correction = (Kp*diff + Kd*(diff-prevError) + Ki*sumError)
    #print("Correction: %.4f, %.4f, %.4f" % (Kp*diff, Kd*(diff-prevError), Ki*sumError))
    prevError = diff
    sumError += diff
    sumError = max(min(200, sumError), -200)
    #print("Step Size: %.4f, %.4f" % (diff, (abs(currCount) - abs(prevCount))))
       
    return correction, prevError, sumError
        
def distTravel(dist, rightEncoderVal, leftEncoderVal, ppr):
    dist = (rightEncoderVal + leftEncoderVal)*math.pi*80/(2*ppr)
    return dist
	
####################### MAIN ####################### 	
	
'''
# SETUP
error = LSM6DSO.begin(LSM6DSOAddr, i2cbus)
if (~error):
	print("IMU Found")
else:
	print("IMU Not Found")

LSM6DSO.intialise(LSM6DSOAddr, i2cbus)
print("IMU initialised")

##### IMU end

print("Calibrate Gyro")
time.sleep(5)
gyroCal = IMU_Gyro_Cal();
accCal = IMU_Acc_Cal();
print("Gyro calibrated: %.04f, %.04f, %.04f" % gyroCal)
print("Acc calibrated: %.04f, %.04f, %.04f" % accCal)
'''
# 1 = forward, 2 = turn, 3 = charge, 4 = charge release
seq = [4]
seqTarget = [0,0,0]
turnAngle = 0

sequenceStep = 0

speedSet = 0

chargingMechanismStart = 0

while True:
    currTime = time.time()
    '''
    if temp == 0:
        chargingMechanism(1)
        temp = 1
    '''
    #rightServoStop()
    #leftServoStop()
    '''
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))
    keys = pygame.key.get_pressed()   
    if keys[pygame.K_i]:
        leftServo.value = -0.5
        rightServo.value = -0.5
    elif keys[pygame.K_o]:
        leftServo.value = 0
        rightServo.value = 0
    elif keys[pygame.K_p]:
        leftServo.value = 0.2
        rightServo.value = 0.2
    '''
    
    if seq[sequenceStep] == 1:
        distSetPoint = seqTarget[sequenceStep]
        if speedSet == 0:
            rightMotorSpeed = 0.65
            leftMotorSpeed = 0.65
            speedSet = 1
            move = 1

    else:
        if seq[sequenceStep] == 2:
            turnSetPoint = (seqTarget[sequenceStep] - prevAngle)*1.2
            turn = 1
            
            if speedSet == 0:
                  rightMotorSpeed = 0.85
                  leftMotorSpeed = 0.85
                  speedSet = 1
                  cummulativeAngle = 0
    '''
    if gyroRead:
        gyroYaw = ((gyro[2] - gyroCal[2]))*(gyroTimeNew - gyroTimeOld)/10
        
        temp = round(math.floor(-(acc[1]-accCal[1])*100)/100.0,1)
        temp2 = round(math.floor((acc[0]-accCal[0])*100)/100.0,1)
        if temp == 0 or temp2 == 0:
            accYaw = 0
        else:
            accYaw = 180 * math.atan2(temp, temp2) / math.pi
            
        cummulativeAngle = (cummulativeAngle + gyroYaw)*1 
        gyroRead = 0
        accRead = 0
    '''
       
    if angleReset:
        print("Angle Reset")
        prevAngle = turnAngle
        turnAngle = 0
        angleReset = 0
        complete = 0
        sequenceStep += 1
        speedSet = 0
        dist = 0 
        turn = 0
        move = 0
        getEncoder()
        time.sleep(2)
        
    if chargeCount == 2:
        print("charge complete")
        chargingMechanismStart = 0
        chargeCount = 0
        sequenceStep += 1
        time.sleep(2)

      
    if (currTime-prevTime) >= 0.05:
        print("")
        print("Time: %0.04f"%(currTime-prevTime))
        print("Set Speed: %.4f" % speedSet)
        getEncoder()
        
        prevTime = currTime
        if move:
            if distSetPoint - dist < 500:
                speedPreset = 0
            if distSetPoint - dist < 200:
                speedPreset = 0
        else:
            print("Within last 100mm")
            if move:
                complete = 1
            move = 0
        
        if turn:  
            move = 0
            print(turnSetPoint)
            if turnSetPoint - turnAngle > 1:
                right = 0
                left = 1
            elif turnSetPoint - turnAngle < -1:
                left = 0
                right = 1
            else:
                print("Turn else")
                if turn:
                    complete = 1
                turn = 0
                left = 0
                right = 0
                turnSetPoint = 0
        
        
       
        print("Move: %.1f" % chargeCount)
        if move:
            # Kp, Ki, Kd, currCount, prevCount, presetCount, prevError, sumError   
            if (rightEncoderVal - prevRightStep > 0) and (leftEncoderVal - prevLeftStep > 0):
                #if (rightEncoderVal - prevRightStep > speedPreset*0.5) and (leftEncoderVal - prevLeftStep > speedPreset*0.5):
                if (abs(prevEncoderError) <= abs(rightEncoderVal - leftEncoderVal)):
                    print("PID On")
                    rightCorrection = stepCalibrate(0.00004, 0.00000, 0.000015, rightEncoderVal, prevRightStep, max(min(200, (speedPreset - (rightEncoderVal - leftEncoderVal)*1)), 0), prevRightStepError, sumRightStepError)
                    prevRightStepError = rightCorrection[1]
                    sumRightStepError = rightCorrection[2]
                    rightMotorSpeed += rightCorrection[0]
                
                    leftCorrection = stepCalibrate(0.00004, 0.00000, 0.000015, leftEncoderVal, prevLeftStep, max(min(200, (speedPreset + (rightEncoderVal - leftEncoderVal)*1)), 0), prevLeftStepError, sumLeftStepError)
                    prevLeftStepError = leftCorrection[1]
                    sumLeftStepError = leftCorrection[2]
                    leftMotorSpeed += leftCorrection[0]
                    
            else:			
                print("Unmoved")
                rightCorrection = stepCalibrate(0.00005, 0, 0, rightEncoderVal, prevRightStep, max(min(200, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.25)), 0), prevRightStepError, sumRightStepError)
                prevRightStepError = rightCorrection[1]
                sumRightStepError = rightCorrection[2]
                rightMotorSpeed += rightCorrection[0]
                
                leftCorrection = stepCalibrate(0.00005, 0, 0, leftEncoderVal, prevLeftStep, max(min(200, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.25)), 0), prevLeftStepError, sumLeftStepError)
                prevLeftStepError = leftCorrection[1]
                sumLeftStepError = leftCorrection[2]
                leftMotorSpeed += leftCorrection[0]
                       
            #print("Set Speed: %.4f, %.4f" % (max(min(200, (speedPreset - (rightEncoderVal - leftEncoderVal)*0.25)), 0), (max(min(200, (speedPreset + (rightEncoderVal - leftEncoderVal)*0.25)), 0))))
            
            speedDiff = (rightMotorSpeed - leftMotorSpeed)
            if speedDiff > 0.05:
                rightMotorSpeed -= (speedDiff-0.05)/2
                leftMotorSpeed += (speedDiff-0.05)/2
            elif speedDiff < -0.05:
                rightMotorSpeed -= (speedDiff+0.05)/2
                leftMotorSpeed += (speedDiff+0.05)/2
            
            rightMotorSpeed = max(min(1, rightMotorSpeed), 0.48)
            #print("%.6f" % rightMotorSpeed)
            leftMotorSpeed = max(min(1, leftMotorSpeed), 0.48)
            
            
            
            leftMotor.forward(leftMotorSpeed)
            rightMotor.forward(rightMotorSpeed)
            print("Speed, left = % .08f, right = % .08f" % (leftMotorSpeed, rightMotorSpeed))
            prevEncoderError = rightEncoderVal - leftEncoderVal
            print("%0.04f" % (rightEncoderVal - leftEncoderVal))
                        
        elif right:
            print("Turning Right")
            leftMotor.forward(leftMotorSpeed)
            rightMotor.stop()
        elif left:
            print("Turning Left")
            leftMotor.backward(leftMotorSpeed)
            rightMotor.stop()
        else:
            leftMotor.stop() 
            rightMotor.stop()
            if complete and ((rightEncoderVal - prevRightStep) == 0) and ((leftEncoderVal - prevLeftStep) == 0):
                complete = 0
                
                angleReset = 1
            
        prevRightStep = rightEncoderVal
        prevLeftStep = leftEncoderVal
        turnAngle = ((rightEncoderVal-leftEncoderVal)*80*math.pi/ppr)*(360/(math.pi*810))
        #print("Turn Angle: %.4f degree" % turnAngle)
        #print("Left Turn Angle: %.4f degree" % (((abs(leftEncoderVal)*282.74/(ppr))/(1319.47))*360))
        #print("Right: %.4f" % rightEncoderVal)
        #print("Left: %.4f" % leftEncoderVal)
        #print("Encoder Yaw: %.4f" % (turnAngle))
        #print("Gyro Angle: %.4f deg" % cummulativeAngle)
        #print("Prev Angle: %.4f deg" % prevAngle)
        dist = distTravel(dist, rightEncoderVal, leftEncoderVal, ppr)
        #print("Distance Travelled: %.2f mm" % dist)
            #print("Encoder Count %0.04f, %0.04f" % (prevRightStep, prevLeftStep))
            #print("%0.04f" % (rightEncoder.steps))
        if seq[sequenceStep] == 3 and chargingMechanismStart == 0:
            chargingMechanism(1)
            chargingMechanismStart = 1
        elif seq[sequenceStep] == 4 and chargingMechanismStart == 0:
            chargingMechanism(2)
            chargingMechanismStart = 1
            
        print(seq[sequenceStep])
    #print("%0.04f" % (rightEncoder.steps))
    #print("%0.04f" % (rightEncoderVal))
    '''
    if leftEncoder.steps > ppr:
        leftMotor.stop()
    else:
        leftMotor.forward(0.67)
    print("right: %.4f, left: %.4f" % (rightEncoder.steps, leftEncoder.steps))
    '''
   
    #print("")
    
print("Closed")

