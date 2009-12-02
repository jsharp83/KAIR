import NIDeviceWrapper
from threading import Thread

NI = NIDeviceWrapper
NI.initDevice()

class ReadEncoderThread(Thread) :
	def __init__(self) :
		Thread.__init__(self)
	def run(self) :
		left = 0
		right = 0
		
		while True :
			left += NI.countLeftEncoderTic()
			right += NI.countRightEncoderTic()
			print left, " ", right

if __name__ == "__main__" :
	print "Start Test code"
	NI.updateVoltages(1.0, 1.0)
	print "Update Voltage"
	encoder = ReadEncoderThread()
	encoder.start()
