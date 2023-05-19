from smbus import SMBus
import datetime, time
import math
from gpiozero import Button, Servo, Motor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory

####################### SETUP ####################### 

def runRover(instruction):

	factory = PiGPIOFactory()

	cummulativeAngle = 0.0

	##### Motors
	rightMotorSpeed = 0.62
	leftMotorSpeed = 0.62
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
		
	# 1 = forward, 2 = turn, 3 = charge, 4 = charge release

	for distance, bearing, chargeSeq in instruction:
		movement = distance
		turning = bearing
		chargeSeq = chargeSeq
	turnAngle = 0

	sequenceStep = 0

	speedSet = 0

	chargingMechanismStart = 0

	while True:
		currTime = time.time()
		
		if sequenceStep > len(seqTarget):
			break
		if bearing[sequenceStep] == 0:
			distSetPoint = movement[sequenceStep]
			if speedSet == 0:
				rightMotorSpeed = 0.65
				leftMotorSpeed = 0.65
				speedSet = 1
				move = 1

		elif movement[sequenceStep] == 0:
			turnSetPoint = (bearing[sequenceStep] - prevAngle)*1.2
			turn = 1
				
			if speedSet == 0:
				rightMotorSpeed = 0.85
				leftMotorSpeed = 0.85
				speedSet = 1
				cummulativeAngle = 0
				
		elif chargeSeq[sequenceStep] == 1 and chargingMechanismStart == 0:
				chargingMechanism(1)
				chargingMechanismStart = 1
		elif chargeSeq[sequenceStep] == 0 and chargingMechanismStart == 0:
				chargingMechanism(2)
				chargingMechanismStart = 1

		   
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
			
				
			print(seq[sequenceStep])

    
#print("Closed")

