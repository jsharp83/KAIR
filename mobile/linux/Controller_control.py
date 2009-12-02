import sys
sys.path.append('./arm')
sys.path.append('./dynamixel')
sys.path.append('./network')

import serial
import math
from threading import Thread
from Configuration import *

from dynamixel_AX12 import Dxl_AX12
from server import *
from client import *
import KNI

from time import sleep

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

"""
class ReceiveFromSerialThread(Thread) :
	def __init__(self, parent, serial) :
		Thread.__init__(self)
		self.parent = parent
		self.ser = serial

	def setSerialPort(self, serial):
		self.ser = serial

	def run(self) :
		while True :
			parent.msgFromSerial(ser.readline())
"""

class BaseRobotController() :
	def __init__(self, Controller) :
		self.connected = None
		self.Controller = Controller
		self.oldVels = (0.0, 0.0)

		if(BASE_CONNECT_TO_WINDOW) :
			self.client = InitThe(ClientToWindowServer,WINDOW_SERVER_IP, WINDOW_SERVER_PORT)
			self.client.connectToServer()
			self.connected = True
		else :
			self.connected = False
		
	def generateMsg(self, msg) :
		if msg[0] == BASE_VELOCITY :
			generatedMsg = msg[0]+" "

			vels = msg[1]
			(x, y) = vels[0], -vels[1]

			power = math.sqrt(x*x + y*y)

			rVel = power
			lVel = power

			if x>=0 :
				rVel = rVel -x
			else :
				lVel = lVel +x

			if y >=0 :
				lVel = -lVel
			else :
				rVel = -rVel

			lVel = round(MOBILE_MAX_VOLTAGE * lVel / 100, 2)
			rVel = round(MOBILE_MAX_VOLTAGE * rVel / 100, 2)

			generatedMsg = generatedMsg + str(lVel) + " "+ str(rVel)

			return generatedMsg

	def generateMsgs(self, msg) :
		msgs = []
		oldVels = self.oldVels
		for i in (1,2) :
			(x, y) = msg[1]
			if oldVels[0] > x :
				sign1 = -1
			else :
				sign1 = 1

			if oldVels[1] > y :
				sign2 = -1
			else :
				sign2 = 1

			diff_x = abs(abs(x)-abs(oldVels[0]))
			diff_y = abs(abs(y)-abs(oldVels[1]))

			tempMsg = []
			tempMsg.append(msg[0])
			tempX = oldVels[0]+(diff_x/3)*sign1*i
			tempY = oldVels[1]+(diff_y/3)*sign2*i
			tempMsg.append((tempX,tempY))
			msgs.append(self.generateMsg(tempMsg))

		msgs.append(self.generateMsg(msg))
		self.oldVels = msg[1]

		return msgs

	def sendMsg(self, msg) :
		if self.connected == True :
			msgs= self.generateMsgs(msg)
			for i in range(0,len(msgs)) :
				print msgs[i]
				self.client.sendMsgToServer(BASE_ROBOT, msgs[i])
				sleep(0.01)
			
#			self.client.send("BASE_ROBOT")
#			self.client.send(generateMsg)
			#receive ACK
#			self.client.recv(2)
		else :
			msgs= self.generateMsgs(msg)
			for i in range(0,len(msgs)) :
				print msgs[i]
				sleep(0.01)
			
			print "Not connected"
			
		"""

		self.client.send("BASE_ROBOT")

#		ACK
		self.client.recv(2)
		
		client.send(msg)
		"""

"""
class BaseRobotController(AbstractController) :
	def __init__(self, port) :
		self.parent = AbstractController(port)
		self.ser=self.parent.ser

		self.receiveThread = ReceiveFromSerialThread(self, self.ser)

	def connectSerial(self) :
		try :
			self.ser = serial.Serial(self.port)
			self.receiveThread.setSerialPort(self.ser)
			self.receiveThread.start()
		except serial.serialutil.SerialException :
			self.ser = None
			print "serial port is not connected"

	def msgFromSerial(self, msg) :
		print msg

	def generateVelMsg(self, point) :
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
			rVel = rVel / 10 
			if rVel >3 :
				if rVel > 10 :
					rVel = 3
				elif rVel > 6 :
					rVel = 2
				else :
					rVel = 1
		else :
			rVel = 0
			

		if(lVel / 10 >= 1) :
			lVel = lVel /10

			if lVel >3 :
				if lVel > 10 :
					lVel = 3
				elif lVel > 6 :
					lVel = 2
				else :
					lVel = 1
		else :
			lVel = 0

		velstr = ((rVel/10).__int__()).__str__()+((rVel%10).__int__()).__str__() + ((lVel/10).__int__()).__str__() + ((lVel%10).__int__()).__str__()
		
		msg = msg + velstr+"]\n"

		return msg

	def generateDistMsg(self, dists) :
		leftWheel = dists[0]
		rightWheel = dists[1]

		if leftWheel > 0 :
			msgLeft = "[LBD"
		else :
			msgLeft = "[LFD"

		if rightWheel > 0 :
			msgRight = "[RFD"
		else :
			msgRight = "[RBD"

		leftWheel = abs(leftWheel)
		rightWheel = abs(rightWheel)

		msgLeft = msgLeft+str(leftWheel/1000)
		leftWheel = leftWheel%1000
		msgLeft = msgLeft+str(leftWheel/100)
		leftWheel = leftWheel%100
		msgLeft = msgLeft+str(leftWheel/10)
		leftWheel = leftWheel%10
		msgLeft = msgLeft+str(leftWheel)+"]\n"


		msgRight = msgRight+str(rightWheel/1000)
		rightWheel = rightWheel%1000
		msgRight = msgRight+str(rightWheel/100)
		rightWheel = rightWheel%100
		msgRight = msgRight+str(rightWheel/10)
		rightWheel = rightWheel%10
		msgRight = msgRight+str(rightWheel)+"]\n"

		return (msgRight, msgLeft)

	
	def sendMsg(self, msg) :
		print msg
		
		if(msg[0] == BASE_VELOCITY) :
			curMsg = self.generateVelMsg(msg[1])
			self.sendMsgToSerial(curMsg)
		elif (msg[0] == BASE_DISTANCE) :
			curMsgs = self.generateDistMsg(msg[1])
			self.sendMsgToSerial(curMsgs[0])
			self.sendMsgToSerial(curMsgs[1])
		elif (msg == RECEIVE_ENCODER) :
			self.sendMsgToSerial("[D000000]")
		elif (msg == RESET_ENCODER) :
			self.sendMsgToSerial("[C000000]")
"""

"""
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
"""

class ArmRobotController(AbstractController) :
	def __init__(self, port) :
		self.port = port
		self.isConnected = False
#		self.isConnected = KNI.initKatana(self.port[0], self.port[1])

		self.minusMotors = (2,3)
		self.home = None

	def sendMsg(self, msg) :
		if self.isConnected :
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

		else :
			print "Port is not connected"

class LinearRobotController(AbstractController) :
	def __init__(self, port) :
		self.parent = AbstractController(port)
		self.ser=self.parent.ser		

	def sendMsg(self, msg) :
		self.sendMsgToSerial(msg)

class PANTILT_UNIT_Controller(AbstractController) :
	def __init__(self, port) :
		self.port = port
		self.connectSerial()
		self.isConnected = self.dxl.isConnected()

	def connectSerial(self) :
		self.dxl = Dxl_AX12(self.port,57600)

	def sendToDxl(self, id, position) :
		self.dxl.setPosition(id, position, DXL_SPEED)

	def sendMsg(self, msg) :
		
		if self.isConnected :
			if msg[0] == 'HOKUYO' :
				self.sendToDxl(HOKUYO_ID, msg[1])
			elif msg[0] == 'CAMERA' :
				if msg[1] == 'PAN' :
					self.sendToDxl(CAMERA_PAN, msg[2])
				elif msg[1] == 'TILT' :
					self.sendToDxl(CAMERA_TILT,msg[2])
		else :
			print "Port is not connected"

class NetworkController() :
	def __init__(self, Controller) :

		self.Controller = Controller
		
		self.server = None
		self.client = None
		self.isServerCreated = False
		self.isConnected = False
		self.Role = self.Controller.Role
		
	def createServer(self) :
		self.server = InitThe(ServerMain, self.Controller)
		self.server.start()
		self.isCreated = True
		
	def connectServer(self) :
		self.client = InitThe(ClientMain,SERVER_IP, SERVER_PORT)
		self.client.connectToServer()
		self.isConnected = True
		print "Connected server"

	def changeRole(self, msg) :
		if msg == SERVER_SELECTED :
			print "Role is Server"
			self.Role = SERVER_SELECTED
			self.Controller.Role = SERVER_SELECTED
			
		elif msg == CLIENT_SELECTED :
			self.Role = CLIENT_SELECTED
			self.Controller.Role = CLIENT_SELECTED
			print "Role is Client"
			self.connectServer()

	def sendMsg(self, msg) :
		if msg[0] == SELECT_ROLE :
			self.changeRole(msg[1])
		elif msg == CREATE_SERVER :
			self.createServer()

	def sendMsgToServer(self, name,msg) :
		self.client.sendMsgToServer(name,msg)

class LocalNetworkController(NetworkController) :
	def __init__(self, Controller) :
		self.Controller = Controller
		self.server = None
		self.client = None
		self.isServerCreated = False
		self.isConnected = False

	def createServer(self) :
		self.server = InitThe(LocalServer, self.Controller)
		self.server.start()
		self.isCreated = True
		import os
		os.system('./cplus/Client')

	def icpSLAMOn(self) :
		socket = clientSocketList["cplus"]
		print "Find socket : ", socket
		socket.send("ICP_SLAM")
		print "send msg"

	def sendMsg(self, msg) :
		print msg
		if msg == "server on" :
			self.createServer()
		elif msg == "ICP_SLAM" :
			self.icpSLAMOn()

					

class Controller(object):
	def __init__(self,ports):
		self.Role = None		
		self.ports = ports
		self.baseRobot = BaseRobotController(self)
		self.armRobot = ArmRobotController(ports[ARM_ROBOT])
		self.linearRobot = LinearRobotController(ports[LINEAR_ROBOT])
		self.pantilt_unit = PANTILT_UNIT_Controller(ports[PANTILT_UNIT])
		self.network = NetworkController(self)
		self.localNetwork = LocalNetworkController(self)

	def setView(self,view) :
		self.view = view
		self.checkPortConnection()

	def checkPortConnection(self) :
		if(self.baseRobot.connected == False or self.baseRobot.connected == None) :
			self.view.update(BASE_ROBOT, "not connected")
		if(self.armRobot.isConnected == False) :
			self.view.update(ARM_ROBOT, "not connected")			
		if(self.linearRobot.parent.ser == None) :
			self.view.update(LINEAR_ROBOT, "not connected")
		if(self.pantilt_unit.isConnected == False) :
			self.view.update(PANTILT_UNIT, "not connected")			

	def getController(self) :
		return self

	def sendMsg(self, name, msg) :
		if self.Role == SERVER_SELECTED :
			if name == BASE_ROBOT :
				self.baseRobot.sendMsg(msg)
			elif name == ARM_ROBOT :
				self.armRobot.sendMsg(msg)
			elif name == LINEAR_ROBOT :
				self.linearRobot.sendMsg(msg)
			elif name == PANTILT_UNIT :
				self.pantilt_unit.sendMsg(msg)
			elif name == NETWORK :
				self.network.sendMsg(msg)
			elif name == LOCAL_NETWORK :
				self.localNetwork.sendMsg(msg)
		elif self.Role == CLIENT_SELECTED :
			if name == NETWORK :
				self.network.sendMsg(msg)
			else :
				self.network.sendMsgToServer(name, msg)
		elif self.Role == None :
			if name == NETWORK :
				self.network.sendMsg(msg)
