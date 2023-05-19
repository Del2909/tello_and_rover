from smbus import SMBus
import datetime, threading, time
import math
from gpiozero import Button, Servo, Motor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import LSM6DSO
import pygame
from multiprocessing import Queue

####################### SETUP ####################### 

factory = PiGPIOFactory()

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

##### Keyboard Input
pygame.init()
window = pygame.display.set_mode((300, 300))

##### Motors
rightMotorSpeed = 0.61
leftMotorSpeed = 0.59
rightMotor = Motor(26,19)
leftMotor = Motor(21,20)

rightEncoderVal = 0
leftEncoderVal = 0

prevRightStep = 0
prevLeftStep = 0
rightSpeedCount = 0
rightSpeedSteps = 0
leftSpeedSteps = 0
prevSpeedTime = 0

prevRightDiffSpeed = 0
prevLeftDiffSpeed = 0
prevRightSumSpeed = 0
prevLeftSumSpeed = 0

dist = 0

ppr = 341
rightEncoder = RotaryEncoder (10, 9, max_steps = 0, pin_factory = factory)
leftEncoder = RotaryEncoder (22, 27, max_steps = 0, pin_factory = factory)

##### Charging Plates

leftServo = Servo (12, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)
rightServo = Servo (16, min_pulse_width = 0.0005, max_pulse_width = 0.0025, pin_factory = factory)

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

yawKp = 0.00001
yawKd = 0.000005
yawKi = 0.000001

# ENCODER UPDATES

rightEncoderBuf = 0
leftEncoderBuf = 0

def getEncoder():
    global move, rightEncoderBuf, leftEncoderBuf, rightEncoderVal, leftEncoderVal
    if move:
        rightEncoderVal = rightEncoder.steps - rightEncoderBuf
        leftEncoderVal = leftEncoder.steps - leftEncoderBuf
    else:
        rightEncoderBuf = rightEncoder.steps
        leftEncoderBuf = leftEncoder.steps

def stepCalibrate(rightSpeed, leftSpeed, presetSpeed):
    global move, rightEncoderBuf, leftEncoderBuf, leftMotorSpeed, rightMotorSpeed, prevDistDiff, sumDistDiff, rightEncoderVal, leftEncoderVal, prevRightStep, prevLeftStep
    # +ve = CCW rotation
    if move:
        distDiff = rightEncoderVal - leftEncoderVal
        #distDiff = max(min(150, distDiff), -150)
        #print("Correction: %0.04f, %0.04f, %0.04f" %(distDiff, (distDiff-prevDistDiff), sumDistDiff))
        print("Corrected: %0.04f, %0.04f, %0.04f" %(yawKp*distDiff, yawKd*(distDiff-prevDistDiff), yawKi*sumDistDiff))
        if (rightSpeed >= 0.1 and leftSpeed >= 0.1):
            rightMotorSpeed -= (yawKp*distDiff + yawKd*(distDiff-prevDistDiff) + yawKi*sumDistDiff)
            leftMotorSpeed += (yawKp*distDiff + yawKd*(distDiff-prevDistDiff) + yawKi*sumDistDiff)
            prevDistDiff = distDiff
            sumDistDiff += distDiff
            #sumDistDiff = max(min(200, sumDistDiff), -200)
            sumDistDiff = sumDistDiff * 0.75
    else:
        rightEncoderBuf = rightEncoder.steps
        leftEncoderBuf = leftEncoder.steps
    print("%.4f, %.4f" % (rightEncoderVal, leftEncoderVal))
    #print(rightEncoder.steps)
    #print(leftEncoder.steps)
    #print(distDiff)
    
     
def speedCal(speed, setPoint, Kp, Kd, Ki, prevDiff, sumDiff):
    global move
    if move:	
	    diff = setPoint-speed
	    #print(diff)
	    correction = Kp*diff + Kd*(diff-prevDiff) + Ki*sumDiff
	    #print("SpeedCal = %0.08f, %.08f" % (diff, (diff-prevDiff)))
	    sumDiff += diff
	    sumDiff = max(min(200, sumDiff), -200)
	    return correction, diff, sumDiff
    else:
        return 0, 0, 0
        
def distTravel(dist, rightEncoderVal, leftEncoderVal, ppr):
    dist = (rightEncoderVal + leftEncoderVal)*251.33/(2*ppr)
    return dist
	
####################### MAIN ####################### 	
	
# SETUP
#error = LSM6DSO.begin(LSM6DSOAddr, i2cbus)
#if (~error):
#	print("IMU Found")
#else:
#	print("IMU Not Found")

#LSM6DSO.intialise(LSM6DSOAddr, i2cbus)
#print("IMU initialised")

##### IMU end

#print("Calibrate Gyro")
#time.sleep(5)
#gyroCal = IMU_Gyro_Cal();
#accCal = IMU_Acc_Cal();
#print("Gyro calibrated: %.04f, %.04f, %.04f" % gyroCal)
#print("Acc calibrated: %.04f, %.04f, %.04f" % accCal)

move = 0

#timerThread = threading.Thread(target=speedCalibrate)
#timerThread.daemon = True
#timerThread.start()

prevTime = 0
currTime = 0

complete = 0

speedPreset = 2.5

while True:
    currTime = time.time()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))
    keys = pygame.key.get_pressed()   
	#char = screen.getch()
    if keys[pygame.K_p]:
        print("stop")
        break
    elif keys[pygame.K_UP]:
        #print("up")
        move = 1
        leftMotor.forward(leftMotorSpeed)
        rightMotor.forward(rightMotorSpeed)
    elif keys[pygame.K_DOWN]:
        print("down")
        leftMotor.backward(leftMotorSpeed)
        rightMotor.backward(rightMotorSpeed)
    elif keys[pygame.K_RIGHT]:
        print("right")
        cummulativeAngle = 0
        leftMotor.forward(leftMotorSpeed)
        rightMotor.backward(rightMotorSpeed)
    elif keys[pygame.K_LEFT]:
        print("left")
        cummulativeAngle = 0
        leftMotor.backward(leftMotorSpeed)
        rightMotor.forward(rightMotorSpeed)
    elif keys[pygame.K_s]:
        leftMotor.stop() 
        rightMotor.stop() 
        move = 0 
    elif keys[pygame.K_q]:
        leftServo.value = -0.5
        rightServo.value = -0.5
    elif keys[pygame.K_w]:
        leftServo.value = 0
        rightServo.value = 0
    elif keys[pygame.K_e]:
        leftServo.value = 0.1
        rightServo.value = 0.1
    elif keys[pygame.K_r]:
        cummulativeAngle = 0
        leftMotorSpeed = 0.75
        rightMotorSpeed = 0.75
    '''
    if gyroRead or accRead and ~keys[pygame.K_LEFT] and ~keys[pygame.K_RIGHT]:
        gyroYaw = ((gyro[2] - gyroCal[2]))*(gyroTimeNew - gyroTimeOld)/10
        temp = round(math.floor(-(acc[1]-accCal[1])*100)/100.0,1)
        temp2 = round(math.floor((acc[0]-accCal[0])*100)/100.0,1)
        if temp == 0 or temp2 == 0:
            accYaw = 0
        else:
            accYaw = 180 * math.atan2(temp, temp2) / math.pi
        cummulativeAngle = (cummulativeAngle + gyroYaw)*1 + accYaw*0
        gyroRead = 0
        accRead = 0
        
        # ~ if cummulativeAngle < -1:
           # ~ rightMotorSpeed += 0.001
           # ~ leftMotorSpeed -= 0.001
        # ~ elif cummulativeAngle > 1:
            # ~ leftMotorSpeed += 0.001
            # ~ rightMotorSpeed -= 0.001
        # ~ if rightMotorSpeed > 1:
            # ~ rightMotorSpeed = 1
        # ~ if rightMotorSpeed < 0.6:
            # ~ rightMotorSpeed = 0.6
        # ~ if leftMotorSpeed > 1:
            # ~ leftMotorSpeed = 1
        # ~ if leftMotorSpeed < 0.6:
            # ~ leftMotorSpeed = 0.6
        # ~ print(leftMotorSpeed)
        # ~ print(rightMotorSpeed)

      '''
    if (currTime-prevTime) >= 0.1:
        #print("%0.04f"%(currTime-prevTime))
        print("")
        '''
        if rightSpeedCount == 10 and move:
            rightSpeed = (rightSpeedSteps - prevRightStep)/(ppr*(currTime-prevSpeedTime))
            leftSpeed = (leftSpeedSteps - prevLeftStep)/(ppr*(currTime-prevSpeedTime))
            print("Right Speed: %.04f, %0.04f" % ((rightSpeedSteps - prevRightStep), rightSpeed))
            print("Left Speed: %.04f, %0.04f" % ((leftSpeedSteps - prevLeftStep), leftSpeed))
            
            rightNewSpeed = speedCal(rightSpeed, 15, 0.001, 0.0005, 0, prevRightDiffSpeed, prevRightSumSpeed)
            prevRightDiffSpeed = rightNewSpeed[1]
            prevRightSumSpeed = rightNewSpeed[2]
            rightMotorSpeed += rightNewSpeed[0]
            #print(rightNewSpeed[0])
            leftNewSpeed = speedCal(leftSpeed, 15, 0.001, 0.0005, 0, prevLeftDiffSpeed, prevLeftSumSpeed)
            prevLeftDiffSpeed = leftNewSpeed[1]
            prevLeftSumSpeed = leftNewSpeed[2]
            leftMotorSpeed += leftNewSpeed[0]
            
            prevRightStep = rightSpeedSteps
            prevLeftStep = leftSpeedSteps
            rightSpeedSteps = 0
            leftSpeedSteps = 0
            prevSpeedTime = currTime
            rightSpeedCount = 0
        elif rightSpeedCount == 10:
            rightSpeedCount = 0
            '''
        
        
        if move:
            rightSpeedCount += 1
            if complete == 1:
                print("Speed reduced")
                rightMotorSpeed -=0.0
                leftMotorSpeed -= 0.0	
                complete = 2			
        
        
        getEncoder()
        #rightSpeedSteps += rightEncoderVal
        #leftSpeedSteps += leftEncoderVal
        
        rightSpeed = (rightEncoderVal - prevRightStep)/(ppr*(currTime-prevTime))
        leftSpeed = (leftEncoderVal - prevLeftStep)/(ppr*(currTime-prevTime))
        stepCalibrate(rightSpeed, leftSpeed, speedPreset)
        if rightSpeed >= speedPreset and complete == 0:
            complete = 1
        print("Right Speed: %.04f, %0.04f" % ((rightEncoderVal - prevRightStep), rightSpeed))
        print("Left Speed: %.04f, %0.04f" % ((leftEncoderVal - prevLeftStep), leftSpeed))
        
        if rightSpeed >= speedPreset*0.5 or leftSpeed >= speedPreset*0.5:    
            rightNewSpeed = speedCal(rightSpeed, speedPreset, 0.0001, 0.0000, 0.0000, prevRightDiffSpeed, prevRightSumSpeed)
            prevRightDiffSpeed = rightNewSpeed[1]
            prevRightSumSpeed = rightNewSpeed[2]
            rightMotorSpeed += rightNewSpeed[0]
            print(rightNewSpeed[0])
            leftNewSpeed = speedCal(leftSpeed, speedPreset, 0.0001, 0.0000, 0.0000, prevLeftDiffSpeed, prevLeftSumSpeed)
            prevLeftDiffSpeed = leftNewSpeed[1]
            prevLeftSumSpeed = leftNewSpeed[2]
            leftMotorSpeed += leftNewSpeed[0]
        
        
        #print(currTime-prevTime)
        
        prevTime = currTime
        
        rightMotorSpeed = max(min(0.75, rightMotorSpeed), 0.55)
        #print("%.6f" % rightMotorSpeed)
        leftMotorSpeed = max(min(0.75, leftMotorSpeed), 0.55)
        if move:
            dist = distTravel(dist, rightEncoderVal, leftEncoderVal, ppr)
            leftMotor.forward(leftMotorSpeed)
            rightMotor.forward(rightMotorSpeed)
            print("left = % .08f, right = % .08f" % (leftMotorSpeed, rightMotorSpeed))
            print("%0.04f" % (rightEncoderVal - leftEncoderVal))
            print("Distance Travelled: %.2f mm" % dist)
        prevRightStep = rightEncoderVal
        prevLeftStep = leftEncoderVal
            #print("Encoder Count %0.04f, %0.04f" % (prevRightStep, prevLeftStep))
            #print("%0.04f" % (rightEncoder.steps))
            
    #print("%0.04f" % (rightEncoder.steps))
    #print("%0.04f" % (rightEncoderVal))
    
    
   
    #print("")

print("Closed")

#Close down curses properly, inc turn echo back on!
    


while False:
	
	
	#print("Acc: %.4f, %.4f, %.4f" % (acc))
	
	#print("Gyro: %.4f, %.4f, %.4f" % (gyro))
	#print("Yaw: %.4f" % (gyro[2] - gyroCal[2]))
	#print("Elapsed: %.4f" % (gyroTimeNew - gyroTimeOld))
	if (abs((gyro[2] - gyroCal[2])) > 1 and gyroRead):
		print((gyro[2] - gyroCal[2])*(gyroTimeNew - gyroTimeOld))
		cummulativeAngle += (gyro[2] - gyroCal[2])*(gyroTimeNew - gyroTimeOld)/10
		gyroRead = 0
	
	#print((gyro[2] - gyroCal[2]))
	print(cummulativeAngle)
	print("")
	#time.sleep(0.01)
	#continue
