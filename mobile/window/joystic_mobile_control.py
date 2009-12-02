import serial

serial_port = "/dev/ttyUSB0"
serial_baudrate = 115200
MOBILE_MAX_VOLTAGE = 3
USING_NI = True

class joysticController() :
	def __init__(self) :
		print "Mobile Controller with Joystic program is started"

		if USING_NI :
			import NIDeviceWrapper
			self.NI = NIDeviceWrapper
			self.NI.initDevice()
	def connect(self) :
		self.ser = serial.Serial(port = serial_port, baudrate=serial_baudrate)
		print "Successfully find a port"

	def sendMsgToNI(self,leftSign, leftValue, rightSign, rightValue) :

		if leftSign == 1 :
			lVel = leftValue - 30
		else :
			lVel = leftValue + 30

		if rightSign == 1 :
			rVel = rightValue - 30
		else :
			rVel = rightValue +30
		
		lVel = round(MOBILE_MAX_VOLTAGE * lVel / 70.0, 2)
		rVel = round(MOBILE_MAX_VOLTAGE * rVel / 70.0, 2)

		if(abs(lVel) < 0.1) :
			lVel = 0

		if(abs(rVel) < 0.1) :
			rVel = 0

		print lVel, " ", rVel

		if USING_NI :
			self.NI.updateVoltages(lVel, rVel)
		

	#msg structure : <LF30RF31>
	def receiveStart(self) :
		while True :
			msg = self.ser.read()

			if msg == '<' :
				msg = self.ser.read()
				if msg == 'L' :
					msg = self.ser.read()

					if msg == 'F' :
						leftSign = 1
					elif msg == 'B' :
						leftSign = -1

					msg1 = self.ser.read()
					msg2 = self.ser.read()

					leftValue = int(msg1+msg2) * leftSign

					self.ser.read()
					msg = self.ser.read()

					if msg == 'F' :
						rightSign = 1
					elif msg == 'B' :
						rightSign = -1

					msg1 = self.ser.read()
					msg2 = self.ser.read()

					rightValue = int(msg1+msg2) * rightSign
					
					self.sendMsgToNI(leftSign, leftValue, rightSign, rightValue)


if __name__ == "__main__" :
	cont=joysticController()
	cont.connect()
	cont.receiveStart()
