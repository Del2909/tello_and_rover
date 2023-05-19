import numpy as np



### register locations

FUNC_CFG_ACCESS		= 0x01
PIN_CTRL 			= 0x02

FIFO_CTRL1			= 0x07
FIFO_CTRL2			= 0x08
FIFO_CTRL3			= 0x09
FIFO_CTRL4			= 0x0A

COUNTER_BDR_REG1	= 0x0B
COUNTER_BDR_REG2	= 0x0C

INT1_CTRL			= 0x0D
INT2_CTRL 			= 0x0E
WHO_AM_I			= 0x0F

CTRL1_XL			= 0x10
CTRL2_G				= 0x11
CTRL3_C				= 0x12
CTRL4_C				= 0x13
CTRL5_C				= 0x14
CTRL6_C				= 0x15
CTRL7_G				= 0x16
CTRL8_XL			= 0x17
CTRL9_XL			= 0x18
CTRL10_C			= 0x19

ALL_INT_SRC			= 0x1A
WAKE_UP_SRC			= 0x1B
TAP_SRC				= 0x1C
D6D_SRC				= 0x1D
STATUS_REG			= 0x1E

OUT_TEMP_L			= 0x20
OUT_TEMP_H			= 0x21
OUTX_L_G			= 0x22
OUTX_H_G			= 0x23
OUTY_L_G			= 0x24
OUTY_H_G			= 0x25
OUTZ_L_G			= 0x26
OUTZ_H_G			= 0x27
OUTX_L_A			= 0x28
OUTX_H_A			= 0x29
OUTY_L_A			= 0x2A
OUTY_H_A			= 0x2B
OUTZ_L_A			= 0x2C
OUTZ_H_A			= 0x2D

EMB_FUNC_STATUS_MP	= 0x35
FSM_STATUS_A_MP		= 0x36
FSM_STATUS_B_MP 	= 0x37
STATUS_MASTER_MP	= 0x39

FIFO_STATUS1		= 0x3A
FIFO_STATUS2		= 0x3B

TIMESTAMP0 			= 0x40
TIMESTAMP1			= 0x41
TIMESTAMP2			= 0x42
TIMESTAMP3			= 0x43

TAP_CFG0			= 0x56
TAP_CFG1			= 0x57
TAP_CFG2			= 0x58
TAP_THS_6D			= 0x59

INT_DUR2			= 0x5A
WAKE_UP_THS			= 0x5B
WAKE_UP_DUR			= 0x5C
FREE_FALL			= 0x5D
MD1_CFG				= 0x5E
MD2_CFG				= 0x5F

I3C_BUS_AVB			= 0x62
INTERNAL_FREQ_FINE	= 0x63

INT_OIS				= 0x6F
CTRL1_OIS			= 0x70
CTRL2_OIS			= 0x71
CTRL3_OIS			= 0x72
X_OFS_USR			= 0x73
Y_OFS_USR			= 0x74
Z_OFS_USR			= 0x75
FIFO_DATA_OUT_TAG	= 0x78
FIFO_DATA_OUT_X_L	= 0x79
FIFO_DATA_OUT_X_H	= 0x7A
FIFO_DATA_OUT_Y_L	= 0x7B
FIFO_DATA_OUT_Y_H	= 0x7C
FIFO_DATA_OUT_Z_L	= 0x7D
FIFO_DATA_OUT_Z_H	= 0x7E

def begin(address, i2cport):
	if (address != 0x6A and address != 0x6B):
		print("No known address found")
		return 0
		
	partID = i2cport.read_byte_data(address, WHO_AM_I)
	if partID != 0x6C:
		print("Incorrect ID")
		return 0
	else:
		return 1

def setInterruptOne(address, i2cport):
	
	# Enable acc interrupt
	regVal = i2cport.read_byte_data(address, INT1_CTRL)
	regVal &= 0xFE
	regVal |= 0x01
	i2cport.write_byte_data(address, INT1_CTRL, regVal)
	
	returnVal = i2cport.read_byte_data(address, INT1_CTRL)
	
	if returnVal == regVal:
		return 0
	else:
		return 1

def setInterruptTwo(address, i2cport):
	
	# Enable acc interrupt
	regVal = 0x02
	i2cport.write_byte_data(address, INT2_CTRL, regVal)
	
	returnVal = i2cport.read_byte_data(address, INT2_CTRL)
	
	if returnVal == regVal:
		return 0
	else:
		return 1
	
def setAccRange(address, i2cport, accRange):
	
	if (accRange < 0 or accRange > 16):
		print("Acc range: Out of range (2, 4, 8, 16)")
		return 0
	
	regVal = i2cport.read_byte_data(address, CTRL1_XL)
	
	regVal &= 0xF3
	
	if accRange == 2:
		regVal |= 0x00

	elif accRange == 4:
		regVal |= 0x04

	elif accRange == 8:
		regVal |= 0x08

	elif accRange == 16:
		regVal |= 0x0C

	else:
		print("Acc range: None recognised range (2, 4, 8, 16)")

	i2cport.write_byte_data(address, CTRL1_XL, regVal)
	
	returnVal = i2cport.read_byte_data(address, CTRL1_XL)
	
	if returnVal == regVal:
		return 0
	else:
		return 1
	
	
def setAccDataRate(address, i2cport, rate):
	
	if (rate < 0 and rate > 6660):
		print("Acc rate: Out of range")
		return 0
	regVal = i2cport.read_byte_data(address, CTRL1_XL)
	
	regVal &= 0x0F
	
	if rate == 0:
		regVal |= 0x00
	elif rate == 16:
		regVal |= 0xB0
	elif rate == 125:
		regVal |= 0x10
	elif rate == 26:
		regVal |= 0x20
	elif rate == 52:
		regVal |= 0x30
	elif rate == 104:
		regVal |= 0x40
	elif rate == 208:
		regVal |= 0x50
	elif rate == 416:
		regVal |= 0x60
	elif rate == 833:
		regVal |= 0x70
	elif rate == 1660:
		regVal |= 0x80
	elif rate == 3330:
		regVal |= 0x90
	elif rate == 6660:
		regVal |= 0xA0
	else:
		print("ACC rate: None recognised rate")
	
	i2cport.write_byte_data(address, CTRL1_XL, regVal)
	
	returnVal = i2cport.read_byte_data(address, CTRL1_XL)
	
	if returnVal == regVal:
		return 0
	else:
		return 1

def setGyroRange(address, i2cport, gyroRange):
	
	if (gyroRange < 125 or gyroRange > 2000):
		print("Gyro range: Out of range")
		return 0
	
	regVal = 0xF0
	
	if gyroRange == 125:
		regVal |= 0x02
	elif gyroRange == 250:
		regVal |= 0x00
	elif gyroRange == 500:
		regVal |= 0x04
	elif gyroRange == 1000:
		regVal |= 0x08
	elif gyroRange == 2000:
		regVal |= 0x0C
	else:
		print("Gyro range: None recognised range ")

	i2cport.write_byte_data(address, CTRL2_G, regVal)
	
	returnVal = i2cport.read_byte_data(address, CTRL2_G)
	
	if returnVal == regVal:
		return 0
	else:
		return 1

def setGyroDataRate(address, i2cport, rate):
	
	if (rate < 0 and rate > 6660):
		print("Gyro rate: Out of range")
		return 0
	regVal = i2cport.read_byte_data(address, CTRL2_G)
	
	regVal &= 0x0F
	
	if rate == 0:
		regVal |= 0x00
	elif rate == 125:
		regVal |= 0x10
	elif rate == 26:
		regVal |= 0x20
	elif rate == 52:
		regVal |= 0x30
	elif rate == 104:
		regVal |= 0x40
	elif rate == 208:
		regVal |= 0x50
	elif rate == 416:
		regVal |= 0x60
	elif rate == 833:
		regVal |= 0x70
	elif rate == 1660:
		regVal |= 0x80
	elif rate == 3330:
		regVal |= 0x90
	elif rate == 6660:
		regVal |= 0xA0
	else:
		print("Gyro rate: None recognised rate")
	
	i2cport.write_byte_data(address, CTRL2_G, regVal)
	
	returnVal = i2cport.read_byte_data(address, CTRL2_G)
	
	if returnVal == regVal:
		return 0
	else:
		return 1


def intialise(address, i2cport):
	count = 0
	count += setInterruptOne(address, i2cport)
	count += setInterruptTwo(address, i2cport)
	count += setAccRange(address, i2cport, 2);
	count += setAccDataRate(address, i2cport, 104)
	count += setGyroRange(address, i2cport, 125);
	count += setGyroDataRate(address, i2cport, 104)
	print("%2f" % count)
	
def readAccX(address, i2cport):
	output = np.int16(i2cport.read_word_data(address, OUTX_L_A))
	return(calAcc(address, i2cport, output))
	
def readAccY(address, i2cport):
	output = np.int16(i2cport.read_word_data(address, OUTY_L_A))
	return(calAcc(address, i2cport, output))
	
def readAccZ(address, i2cport):
	global acc_Z
	output = np.int16(i2cport.read_word_data(address, OUTZ_L_A))
	return(calAcc(address, i2cport, output))
		
		
def readGyroX(address, i2cport):
	regVal = np.int16(i2cport.read_word_data(address, OUTX_L_G))
	return(calGyro(address, i2cport, regVal))

def readGyroY(address, i2cport):
	regVal = np.int16(i2cport.read_word_data(address, OUTY_L_G))
	return(calGyro(address, i2cport, regVal))

def readGyroZ(address, i2cport):
	regVal = np.int16(i2cport.read_word_data(address, OUTZ_L_G))
	return(calGyro(address, i2cport, regVal))
	
	
def calAcc(address, i2cport, inp):
	accRange = i2cport.read_byte_data(address, CTRL1_XL)
	scale = (accRange >> 1) & 0x01
	accRange = (accRange >> 2) & (0x03)
	
	if scale == 0:
		if accRange == 0:
			output = float(inp) * 0.061 * 0.00980665
		elif accRange == 1:
			output = float(inp) * 0.488 * 0.00980665
		elif accRange == 2:
			output = float(inp) * 0.122 * 0.00980665
		elif accRange == 3:
			output = float(inp) * 0.244 * 0.00980665
		else:
			print("Scale error, %1f, %1f" % (scale, accRange))
	
	if scale == 1:
		if accRange == 0:
			output = (float(inp) * 0.061) * 0.00980665
		elif accRange == 1:
			output = float(inp) * 0.061 * 0.00980665
		elif accRange == 2:
			output = float(inp) * 0.122 * 0.00980665
		elif accRange == 3:
			output = float(inp) * 0.244 * 0.00980665
		else:
			print("Scale error, %1f, %1f" % (scale, accRange))
	
	return output
	
def calGyro(address, i2cport, inp):
	gyroRange = i2cport.read_byte_data(address, CTRL2_G)
	fullscale = (gyroRange >> 1) & 0x01
	gyroRange = (gyroRange >> 2) & 0x03
	
	if fullscale:
		output = float(inp) * 4.375 * 0.00980665
	else:
		if gyroRange == 0:
			output = float(inp) * 8.75 / 1000
		elif gyroRange == 1:
			output = float(inp) * 17.5 / 1000
		elif gyroRange == 2:
			output = float(inp) * 35.0 / 1000
		elif gyroRange == 3:
			output = float(inp) * 70.0 / 1000
		else:
			print("Gyro Scale error, %1f, %1f" % (fullscale, gyroRange))
			
	return output

##### TO BE MOVED TO MAIN FUNC



