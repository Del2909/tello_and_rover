import time
from smbus import SMBus
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cv2 import waitKey

i2c = I2C(1)

IMU = LSM6DSOX(i2c)

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

fig = plt.figure(1)
ax = fig.subplots(3,1)

fig1 = plt.figure(2)
ax1 = fig1.subplots(3,1)

def get_acc(i):
	global x, acc_X, acc_Y, acc_Z
	
	
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x.append(curr_time)
	x = x[-samples:]
	
	acc_X.append(round(IMU.acceleration[0],2))
	acc_Y.append(round(IMU.acceleration[1],2))
	acc_Z.append(round(IMU.acceleration[2],2))
	
	
	acc_X = acc_X[-samples:]
	acc_Y = acc_Y[-samples:]
	acc_Z = acc_Z[-samples:]
	#print("%4.0f, %4.0f" % (len(x), len(acc_X)))
	#print("")
	
	ax1[0].clear()
	ax1[1].clear()
	ax1[2].clear()
	ax1[0].plot(x, acc_X)
	ax1[0].set_title('X acceleration')
	ax1[1].plot(x, acc_Y)
	ax1[1].set_title('Y acceleration')
	ax1[2].plot(x, acc_Z)
	ax1[2].set_title('Z acceleration')
	
def get_gyro(i):
	global x2, gyro_X, gyro_Y, gyro_Z
	
	t = time.localtime()
	curr_time = time.strftime("%S",t)
	x2.append(curr_time)
	x2 = x2[-samples:]

	gyro_X.append(round(IMU.gyro[0],2))
	gyro_Y.append(round(IMU.gyro[1],2))
	gyro_Z.append(round(IMU.gyro[2],2))
	
	gyro_X = gyro_X[-samples:]
	gyro_Y = gyro_Y[-samples:]
	gyro_Z = gyro_Z[-samples:]
	#print("%4.0f, %4.0f" % (len(x2), len(gyro_X)))
	#print("")
	
	
	ax[0].clear()
	ax[1].clear()
	ax[2].clear()
	ax[0].plot(x2, gyro_X)
	ax[0].set_title('Pitch')
	ax[1].plot(x2, gyro_Y)
	ax[1].set_title('Roll')
	ax[2].plot(x2, gyro_Z)
	ax[2].set_title('Yaw')
	

while True:
	
	ani = animation.FuncAnimation(fig, get_gyro, interval = 10)

	ani2 = animation.FuncAnimation(fig1, get_acc, interval = 10)
	
	plt.show()
	time.sleep(0.01)
	plt.close()
	
	key = waitKey(1) & 0xFF
	if key == ord('q'): break
	
	
	
	
	
    


