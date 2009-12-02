import serial
import math
from threading import Thread

BASE_ROBOT = "BASE"
LINEAR_ROBOT = "LINEAR"
ARM_ROBOT = "ARM"
PANTILT_UNIT = "PANTILT"

class AbstractController(object) :
	def __init__(self, port):
		self.port = port
		self.connectSerial()

	def connectSerial(self) :
		try :
			self.ser = serial.Serial(self.port)
		except serial.serialutil.SerialException :
			self.ser = None
			print "serial port is not connected"
		
	def sendMsg(self, msg) :
		pass

	def sendMsgToSerial(self,msg) :
		if self.ser is not None :
			self.ser.write(msg)
			print msg
			
		else :
			print "Port is not connected"

class BaseRobotController(AbstractController) :
	def __init__(self, port) :
		self.parent = AbstractController(port)
		self.ser=self.parent.ser

	def generateMsg(self, point) :
		x = point[0]
		y = -point[1]

		power = math.sqrt(x*x+y*y)

		rVel = power
		lVel = power

		if x>=0 :
			rVel = rVel -x
		else :
			lVel = lVel +x

		msg = "[V"
		if y >=0 :
			msg = msg+ "FB"
		else :
			msg = msg+ "BF"

		if(rVel /10 >=1) :
			rVel = rVel / 10 + 3
		else :
			rVel = 0

		if(lVel / 10 >= 1) :
			lVel = lVel /10+3
		else :
			lVel = 0

		velstr = ((rVel/10).__int__()).__str__()+((rVel%10).__int__()).__str__() + ((lVel/10).__int__()).__str__() + ((lVel%10).__int__()).__str__()
		
		msg = msg + velstr+"]\n"

		return msg
		

	def sendMsg(self, msg) :
		curMsg = self.generateMsg(msg)
		self.sendMsgToSerial(curMsg)

class ThreadForArm(Thread) :
	def __init__(self, port) :
		Thread.__init__(self)
		self.name = "ARM"
		self.port = port
		self.status = 0
		

	def run(self) :
		while True :
			if self.status == 1 :
				KNI.initKatana(self.port[0], self.port[1])

		

import KNI	  
class ArmRobotController(AbstractController) :
	def __init__(self, port) :
		self.port = port
		self.isConnected = False
#		self.isConnected = KNI.initKatana(self.port[0], self.port[1])

		self.minusMotors = (2,3)
		self.home = None

	def sendMsg(self, msg) :
		(axis, action) = msg
		if axis == 0 :
			if action == 'CALIB' :
				KNI.calibrate(0)
			elif action == 'SETHOME' :
				from KNI import TMovement
				self.home = TMovement()
				KNI.getPosition(self.home.pos)
				self.home.transition = KNI.PTP
				self.home.velocity = 50
				self.home.acceleration = 2
			elif action == 'MOVEHOME' :
				if self.home != None :
					KNI.executeMovement(self.home)

		else :
			if action == 'STOP' :
				KNI.moveMot(axis, KNI.getEncoder(axis), 50, 2)
			elif action == 'RIGHT' :
				if axis in self.minusMotors :
					KNI.moveMot(axis, 0, 50, 2)
				else :
					KNI.moveMot(axis, 31000, 50, 2)

			elif action == 'LEFT' :
				if axis in self.minusMotors :
					KNI.moveMot(axis, -31000, 50, 2)
				else :
					KNI.moveMot(axis, 0, 50, 2)


class LinearRobotController(AbstractController) :
	def __init__(self, port) :
		self.parent = AbstractController(port)
		self.ser=self.parent.ser		

	def sendMsg(self, msg) :
		self.sendMsgToSerial(msg)

class PANTILT_UNIT_Controller(AbstractController) :
	def __init__(self, port) :
		self.parent = AbstractController(port)
		self.ser=self.parent.ser		

	def sendMsg(self, msg) :
		print msg
		
#		self.sendMsgToSerial(msg)
	
		

class Controller(object):
	def __init__(self,ports):
		self.ports = ports
		self.baseRobot = BaseRobotController(ports[BASE_ROBOT])
		self.armRobot = ArmRobotController(ports[ARM_ROBOT])
		self.linearRobot = LinearRobotController(ports[LINEAR_ROBOT])
		self.pantilt_unit = PANTILT_UNIT_Controller(ports[PANTILT_UNIT])

	def setView(self,view) :
		self.view = view
		self.checkPortConnection()

	def checkPortConnection(self) :
		if(self.baseRobot.parent.ser == None) :
			self.view.update(BASE_ROBOT, "not connected")
		if(self.armRobot.isConnected == False) :
			self.view.update(ARM_ROBOT, "not connected")			
		if(self.linearRobot.parent.ser == None) :
			self.view.update(LINEAR_ROBOT, "not connected")
		if(self.pantilt_unit.parent.ser == None) :
			self.view.update(PANTILT_UNIT, "not connected")			

	def getController(self) :
		return self

	def sendMsg(self, name, msg) :
		if name == BASE_ROBOT :
			self.baseRobot.sendMsg(msg)
		elif name == ARM_ROBOT :
			self.armRobot.sendMsg(msg)
		elif name == LINEAR_ROBOT :
			self.linearRobot.sendMsg(msg)
		elif name == PANTILT_UNIT :
			self.pantilt_unit.sendMsg(msg)
