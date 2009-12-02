import serial
import atexit

from binascii import a2b_hex, b2a_hex


# Instruction packet
# 0xff 0xff ID LENGTH INSTRUCTION PARAMETER....PARAMETER(N), CHECKSUM
# 0xff 0xff  start packet
# ID : 0~254
# LENGHT : Packet Length, Number of Parameter + 2
# CHECKSUM : ~(ID+LENGTH+INSTRUCTION+PARAMETER1+...PARAMETER N)
# INSTRUCTION : 01 PING  02 READ  03 WRITE  04 REG WRITE  05 ACTION  06 RESET  83  SYNCWRITE
INST_PING       = 10
INST_READ       = 2
INST_WRITE      = 3
INST_REG_WRITE  = 4
INST_ACTION     = 5
INST_RESET      = 6
INST_SYNC_WRITE = 131

P_MODEL_NUMBER_L         = 0  # Model Number(L)
P_MODEL_NUMBER_H         = 1  # Model Number(H)
P_VERSION                = 2  # Version of Firmware
P_ID                     = 3  # ID
P_BAUDRATE               = 4  # Baud Rate
P_RETURN_DELAY           = 5  # Return Delay Time
P_CW_ANGLE_LIMIT_L       = 6  # CW Angle Limit(L)
P_CW_ANGLE_LIMIT_H       = 7  # CW Angle Limit(H)
P_CCW_ANGLE_LIMIT_L      = 8  # CCW Angle Limit(L)
P_CCW_ANGLE_LIMIT_H      = 9  # CCW Angle Limit(H)
P_HIGHEST_TEMPERATURE    = 11 # Highest Limit Temperature
P_LOWEST_VOLTAGE         = 12 # Lowest Limit Voltage
P_HIGHEST_VOLTAGE        = 13 # Highest Limit Voltage
P_MAX_TORQUE_L           = 14 # Max Torque(L)
P_MAX_TORQUE_H           = 15 # Max Torque(H)
P_STATUS_RETURN_LEVEL    = 16 # Status Return Level
P_ALARM_LED              = 17 # Alarm LED
P_ALARM_SHUTDOWN         = 18 # Alarm Shutdown
P_TORQUE_ENABLE          = 24 # Torque Enable
P_LED                    = 25 # LED
P_CW_COMPLIANCE_MARGIN   = 26 # CW Compliance Margin
P_CCW_COMPLIANCE_MARGIN  = 27 # CCW Compliance Margin
P_CW_COMPLIANCE_SLOPE    = 28 # CW Compliance Slope
P_CCW_COMPLIANCE_SLOPE   = 29 # CCW Compliance Slope
P_GOAL_POSITION_L        = 30 # Goal Position(L)
P_GOAL_POSITION_H        = 31 # Goal Position(H)
P_MOVING_SPEED_L         = 32 # Moving Speed(L)
P_MOVING_SPEED_H         = 33 # Moving Speed(H)
P_TORQUE_LIMIT_L         = 34 # Torque Limit(L)
P_TORQUE_LIMIT_H         = 35 # Torque Limit(H)
P_PRESENT_POSITION_L     = 36 # Present Position(L)
P_PRESENT_POSITION_H     = 37 # Present Position(H)
P_PRESENT_SPEED_L        = 38 # Present Speed(L)
P_PRESENT_SPEED_H        = 39 # Present Speed(H)
P_PRESENT_LOAD_L         = 40 # Present Load(L)
P_PRESENT_LOAD_H         = 41 # Present Load(H)
P_PRESENT_VOLTAGE        = 42 # Present Voltage
P_PRESENT_TEMPERATURE    = 43 # Present Temperature
P_REGISTERED_INSTRUCTION = 44 # Registered Instruction
P_MOVING                 = 46 # Moving
P_LOCK                   = 47 # Lock
P_PUNCH_L                = 48 # Punch(L)
P_PUNCH_H                = 49 # Punch(H)



# STATUS Packet (Return Packet)
# 0xff 0xff ID LENGTH Error PARAMETER ... N CHECKSUM


# FF FF ID : 12 LENGTH : 07 INST : 03(WRITE) 1E(30, Goal Position) F4(244) 01 64 00 checksum : 6C
data = "FFFF1207031EF40164006C"


HOKUYO_DXL_ID = 1

CENTER_POSITION = 500


class Dxl_AX12(object):
	def __init__(self, port, baudrate = 57600, timeout= 0.1 ):
		try :
			self.ser = serial.Serial(port, baudrate=baudrate, timeout = timeout)
		except serial.serialutil.SerialException :
			self.ser = None
			print "serial port is not connected"			

	def isConnected(self) :
		if self.ser == None :
			return False
		else :
			return True

	def initPosition(self) :
		self.setPosition(HOKUYO_DXL_ID, CENTER_POSITION, 100)

	def setPosition(self, id ,position, speed) :
		msg = self.__generateMsg(id, position, speed)
#		print msg
#		print len(msg)
		self.__sendMsg(msg)

	def __sendMsg(self, msg) :
		self.ser.write(a2b_hex(msg))
		self.ser.write("\r\n")

	def __hexStringToInt(self, str) :
		return int(str,16)

	def __generateMsg(self, id, rotate, speed) :

		if id == HOKUYO_DXL_ID :
			if rotate <200 :
				raise "Out of range exception"
			elif rotate >800 :
				raise "Out of range Exception"
			
		rotateData = self.__generateHexstring(rotate)
		speedData = self.__generateHexstring(speed)

		msg = "FFFF"
		if id < 16 :
			msg = msg +"0"

		msg = msg+hex(id)[2:4]+"07031E"+rotateData[0]+rotateData[1]+speedData[0]+speedData[1]
#		print rotateData[0]+ " " + rotateData[1]+ " " + speedData[0] + " " +speedData[1]
		crc = hex(self.__crc(id,7,3,30,self.__hexStringToInt(rotateData[0]),self.__hexStringToInt(rotateData[1]),self.__hexStringToInt(speedData[0]),self.__hexStringToInt(speedData[1])))[2:4]
		if len(crc) == 1 :
			crc = "0"+crc

		return msg+crc
	
	def __crc( self, *args ) :
		return   (0xFF & ~( sum( args ) ) )

	def __generateHexstring(self, num) :
		hexData = hex(num)

		if len(hexData)==5 :
			result =[hexData[3:5],"0"+hexData[2]]
		if len(hexData)==4 :
			result =[hexData[2:4],"00"]

		return result

if __name__=="__main__" :
	print "start dynamixel driver "
	dxl = Dxl_AX12("/dev/ttyUSB1", 57600)
#	dxl.initPosition()
#	dxl.setPosition(1, 500, 100)
