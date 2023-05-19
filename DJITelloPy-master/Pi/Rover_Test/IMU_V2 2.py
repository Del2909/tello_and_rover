import time
from smbus import SMBus
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from cv2 import waitKey
from multiprocessing import Queue

i2c = I2C(1)

IMU = LSM6DSOX(i2c)

IMU.accelerometer_range = AccelRange.RANGE_2G
IMU.accelerometer_data_rate = Rate.RATE_12_5_HZ
IMU.gyro_data_rate = Rate.RATE_12_5_HZ

IMU._route_int1 &= 1
IMU._tap_latch = 1
IMU._tap_clear = 1

samples = 20
count = 0;
acc_X = []
x = []
x2=[]
acc_Y = []
acc_Z = []

gyro_X = []
gyro_Y = []
gyro_Z = []


def get_acc():
	global x, acc_X, acc_Y, acc_Z
	
	
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x.append(curr_time)
	x = x[-samples:]
	
	acc_X.append(IMU.acceleration[0])
	acc_Y.append(IMU.acceleration[1])
	acc_Z.append(IMU.acceleration[2])
	
	
	acc_X = acc_X[-samples:]
	acc_Y = acc_Y[-samples:]
	acc_Z = acc_Z[-samples:]
	print("Accelerometer")
	print("%.4f, %.4f, %.4f" % (acc_X[-1], acc_Y[-1], acc_Z[-1]))
	print("")
	
	
def get_gyro():
	global x2, gyro_X, gyro_Y, gyro_Z
	
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x2.append(curr_time)
	x2 = x2[-samples:]

	gyro_X.append(IMU.gyro[0])
	gyro_Y.append(IMU.gyro[1])
	gyro_Z.append(IMU.gyro[2])
	
	gyro_X = gyro_X[-samples:]
	gyro_Y = gyro_Y[-samples:]
	gyro_Z = gyro_Z[-samples:]
	
	print("Gyro")
	print("%.4f, %.4f, %.4f" % (gyro_X[-1], gyro_Y[-1], gyro_Z[-1]))
	print("")
	

while True:
	
	get_acc()
	get_gyro()
	
	key = waitKey(1) & 0xFF
	if key == ord('q'): break
	
	
	
	
	
    


