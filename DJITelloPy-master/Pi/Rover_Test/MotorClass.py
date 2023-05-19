import RPi.GPIO as GPIO

class Motor:
	def __init__(self, pin1 , pin2):
		self.pin1 = pin1
		self.pin2 = pin2
		GPIO.setup(pin1,GPIO.OUT)
		GPIO.setup(pin2,GPIO.OUT)
		
		self.pwm1 = GPIO.PWM(self.pin1,100)
		self.pwm2 = GPIO.PWM(self.pin2,100)
		
		
	def forward(self,duty):
		self.pwm1.start(duty)
		self.pwm2.start(0)
		
	def backward(self,duty):
		self.pwm1.start(0)
		self.pwm2.start(duty)
		
	def stop(self):
		self.pwm1.start(0)
		self.pwm2.start(0)
		
	
