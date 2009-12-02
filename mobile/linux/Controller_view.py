import sys
sys.path.append('../configuration/')

import wx
import math
from Controller_control import Controller
from Configuration import *

# Global Variable
controller = Controller(ports)
panels ={}

class BasicRobotPanel(wx.Panel) :
	""" Parent Class of each Robot Panel """
	def __init__(self, parent, id, panelName) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		self.panelName = panelName
		wx.StaticText(self, -1, panelName, (10,10))
		self.mainPanel = wx.Panel(self,-1,pos=ROBOT_PANEL_POS,size=ROBOT_PANEL_SIZE, style=wx.BORDER_SUNKEN)
		wx.StaticText(self, -1, "Port :", ROBOT_SUB_POS)
		self.port = wx.StaticText(self,-1,"/dev/ttyUSB0", (ROBOT_SUB_POS[0], ROBOT_SUB_POS[1]+30))
	def updatePortState(self, msg) :
		self.port.SetLabel(msg)
		


class NetworkPanel(wx.Panel) :
	def __init__(self, parent, id) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		wx.StaticText(self, -1, "Network", (10,10))

		self.roleList = ['Server', 'Client']
		self.radioBox = wx.RadioBox(self, -1, "Role", (10, 40), choices=self.roleList)
		self.radioBox.Bind(wx.EVT_RADIOBOX, self.selectRole)

		self.btnCreateServer = wx.Button(self, -1,"Create Server", (220, 50))
		self.btnCreateServer.Bind(wx.EVT_LEFT_DOWN, self.createServer)


		wx.StaticText(self, -1, "Server IP : ", (10, 100))
		wx.StaticText(self, -1, SERVER_IP, (150, 100))

		controller.sendMsg(NETWORK, (SELECT_ROLE,self.radioBox.GetSelection()))

	def selectRole(self, event) :
		controller.sendMsg(NETWORK, (SELECT_ROLE,self.radioBox.GetSelection()))

	def createServer(self, event) :
		controller.sendMsg(NETWORK, CREATE_SERVER)
		

class BaseRobotPanel(BasicRobotPanel) :
	
	def __init__(self, parent, id) :
		BasicRobotPanel.__init__(self, parent, id, "Base Robot")
		self.port.SetLabel(BASE_ROBOT_PORT)

		wx.StaticText(self, -1, "Wheel distance", (220, 110))
		self.leftWheel = wx.TextCtrl(self, -1, "100", pos=(220, 140), size=(70,30))
		self.rightWheel = wx.TextCtrl(self, -1, "100", pos=(295, 140), size=(70,30))
		self.sendDist = wx.Button(self, -1, "Send",pos=(295, 180), size=(70, 30))
		self.receiveButton = wx.Button(self, -1, "Receive", pos=(210, 220), size=(90, 30))
		self.resetButton = wx.Button(self, -1, "Reset", pos=(300, 220), size=(70, 30))

		self.InitBuffer()
		self.mainPanel.Bind(wx.EVT_PAINT, self.OnPaint)

		self.Bind(wx.EVT_TIMER, self.OnTimeout)

		self.timer = wx.Timer(self)
		self.timer.Start(30)
		self.mainPanel.Bind(wx.EVT_LEFT_DOWN, self.mouseDown)
		self.mainPanel.Bind(wx.EVT_LEFT_UP, self.mouseUp)
		self.mainPanel.Bind(wx.EVT_MOTION, self.mouseMove)
		self.isClicked = False

		self.sendDist.Bind(wx.EVT_LEFT_DOWN, self.sendDistMsg)

		self.receiveButton.Bind(wx.EVT_LEFT_DOWN, self.receiveEncoder)
		self.resetButton.Bind(wx.EVT_LEFT_DOWN, self.resetEncoder)

	def receiveEncoder(self, evt) :
		controller.sendMsg(BASE_ROBOT, RECEIVE_ENCODER)

	def resetEncoder(self, evt ) :
		controller.sendMsg(BASE_ROBOT, RESET_ENCODER)

	def mouseDown(self, evt) :
		self.isClicked = True

	def mouseUp(self, evt) :
		self.isClicked = False
		self.mousePoint = self.basePoint
		msg = (BASE_VELOCITY,(self.mousePoint[0] -self.basePoint[0], self.mousePoint[1]- self.basePoint[1]))
		controller.sendMsg(BASE_ROBOT, msg )

	def isPointerIn(self,curPoint) :
		(cX, cY) = curPoint
		dist = math.sqrt((cX-self.basePoint[0])*(cX-self.basePoint[0]) + (cY-self.basePoint[1])*(cY-self.basePoint[1]))

		if(dist <self.basePoint[0]-BASE_ROBOT_POINT_SIZE/2) :
			return True
		else :
			return False

	def mouseMove(self, evt) :
		if self.isClicked :
			if self.isPointerIn(evt.GetPosition()) :
				self.mousePoint =  evt.GetPosition()
				msg = (BASE_VELOCITY,(self.mousePoint[0] -self.basePoint[0], self.mousePoint[1]- self.basePoint[1]))
				controller.sendMsg(BASE_ROBOT,msg)				

	def OnTimeout(self, evt) :
		dc = wx.BufferedDC(wx.ClientDC(self.mainPanel), self.buffer)
		self.DrawGraph(dc)

	def InitBuffer(self) :
		self.w, self.h =self.mainPanel.GetClientSize()
		self.basePoint = (self.w/2,self.h/2)
		self.mousePoint = self.basePoint
		self.buffer = wx.EmptyBitmap(self.w, self.h)
		dc = wx.BufferedDC(wx.ClientDC(self.mainPanel), self.buffer)
		self.DrawGraph(dc)

	def OnPaint(self, evt) :
		dc = wx.BufferedPaintDC(self, self.buffer)

	def DrawGraph(self, dc) :
		dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
		dc.Clear()		
		dw, dh = dc.GetSize()
		dc.SetPen(wx.Pen("black", 1))
		dc.DrawCircle(dw/2, dh/2, dw/2)
		dc.DrawCircle(self.mousePoint[0], self.mousePoint[1], BASE_ROBOT_POINT_SIZE)

	def sendDistMsg(self, event) :
		left = self.leftWheel.GetValue();
		right = self.rightWheel.GetValue();
		msg = (BASE_DISTANCE,(int(left), int(right)))

		controller.sendMsg(BASE_ROBOT, msg)
		


class LinearRobotPanel(BasicRobotPanel):
	def __init__(self, parent, id):
		BasicRobotPanel.__init__(self, parent, id, "Linear Robot")
		self.port.SetLabel(LINEAR_ROBOT_PORT)
		self.selectedItem = None

		self.upButton = wx.Button(self.mainPanel, -1,"Up")
		self.downButton = wx.Button(self.mainPanel, -1,"Down",pos=(98,0))
		
		self.text1 = wx.StaticText(self.mainPanel, -1, "Hokuyo",pos=(10,50))		
		self.panSlider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,70),style=wx.SL_HORIZONTAL)

		self.text2 = wx.StaticText(self.mainPanel, -1, "Camera, Pan",pos=(10,100))
		self.t_Slider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,120),style=wx.SL_HORIZONTAL)

		self.text3 = wx.StaticText(self.mainPanel, -1, "Camera, Tilt",pos=(10,150))
		self.p_Slider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,170),style=wx.SL_HORIZONTAL)		

		self.upButton.Bind(wx.EVT_LEFT_DOWN, self.up)
		self.upButton.Bind(wx.EVT_LEFT_UP, self.stop)
		self.downButton.Bind(wx.EVT_LEFT_DOWN,self.down)
		self.downButton.Bind(wx.EVT_LEFT_UP, self.stop)
		self.panSlider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changePanValue)
		self.t_Slider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changeTValue)
		self.p_Slider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changePValue)		
	def up(self, event) :
		controller.sendMsg(LINEAR_ROBOT, "SL 560000\r\n")

	def down(self, event) :
		controller.sendMsg(LINEAR_ROBOT, "SL -560000\r\n")		

	def stop(self, event) :
		controller.sendMsg(LINEAR_ROBOT, "SL 0\r\n")
		
	def changePanValue(self, event) :
		value= self.panSlider.GetValue()		
		controller.sendMsg(PANTILT_UNIT, ("HOKUYO",value))		

	def changeTValue(self, event) :
		value = self.t_Slider.GetValue()
		controller.sendMsg(PANTILT_UNIT, ("CAMERA","PAN",value))				

	def changePValue(self, event) :
		value = self.p_Slider.GetValue()
		controller.sendMsg(PANTILT_UNIT, ("CAMERA","TILT",value))				

class ArmRobotPanel(BasicRobotPanel):
	def __init__(self, parent, id):
		BasicRobotPanel.__init__(self, parent, id, "Arm Robot")

		self.btnCalib = wx.Button(self, -1,"Calibration", (ROBOT_SUB_POS[0], ROBOT_SUB_POS[1]+60))
		self.btnSetHome = wx.Button(self, -1, "Set Home", (ROBOT_SUB_POS[0], ROBOT_SUB_POS[1]+110))
		self.btnMoveHome = wx.Button(self, -1, "Move Home", (ROBOT_SUB_POS[0], ROBOT_SUB_POS[1]+160))
		
		self.port.SetLabel(ARM_ROBOT_PORT[0])

		self.button11 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button12 = wx.Button(self.mainPanel, -1, "RIGHT")
		self.button21 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button22 = wx.Button(self.mainPanel, -1, "RIGHT")
		self.button31 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button32 = wx.Button(self.mainPanel, -1, "RIGHT")
		self.button41 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button42 = wx.Button(self.mainPanel, -1, "RIGHT")
		self.button51 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button52 = wx.Button(self.mainPanel, -1, "RIGHT")
		self.button61 = wx.Button(self.mainPanel, -1, "LEFT")
		self.button62 = wx.Button(self.mainPanel, -1, "RIGHT")


		self.btnCalib.Bind(wx.EVT_LEFT_DOWN, self.calib)
		self.btnSetHome.Bind(wx.EVT_LEFT_DOWN, self.setHome)
		self.btnMoveHome.Bind(wx.EVT_LEFT_DOWN, self.moveHome)

		self.button11.Bind(wx.EVT_LEFT_UP, self.stop1)
		self.button11.Bind(wx.EVT_LEFT_DOWN, self.left1)
		self.button12.Bind(wx.EVT_LEFT_UP, self.stop1)
		self.button12.Bind(wx.EVT_LEFT_DOWN, self.right1)

		self.button21.Bind(wx.EVT_LEFT_UP, self.stop2)
		self.button21.Bind(wx.EVT_LEFT_DOWN, self.left2)
		self.button22.Bind(wx.EVT_LEFT_UP, self.stop2)
		self.button22.Bind(wx.EVT_LEFT_DOWN, self.right2)

		self.button31.Bind(wx.EVT_LEFT_UP, self.stop3)
		self.button31.Bind(wx.EVT_LEFT_DOWN, self.left3)
		self.button32.Bind(wx.EVT_LEFT_UP, self.stop3)
		self.button32.Bind(wx.EVT_LEFT_DOWN, self.right3)

		self.button41.Bind(wx.EVT_LEFT_UP, self.stop4)
		self.button41.Bind(wx.EVT_LEFT_DOWN, self.left4)
		self.button42.Bind(wx.EVT_LEFT_UP, self.stop4)
		self.button42.Bind(wx.EVT_LEFT_DOWN, self.right4)

		self.button51.Bind(wx.EVT_LEFT_UP, self.stop5)
		self.button51.Bind(wx.EVT_LEFT_DOWN, self.left5)
		self.button52.Bind(wx.EVT_LEFT_UP, self.stop5)
		self.button52.Bind(wx.EVT_LEFT_DOWN, self.right5)

		self.button61.Bind(wx.EVT_LEFT_UP, self.stop6)
		self.button61.Bind(wx.EVT_LEFT_DOWN, self.left6)
		self.button62.Bind(wx.EVT_LEFT_UP, self.stop6)
		self.button62.Bind(wx.EVT_LEFT_DOWN, self.right6)


		sizer = wx.GridSizer(rows=3, cols=2)
		sizer.Add(self.button11)
		sizer.Add(self.button12)
		sizer.Add(self.button21)
		sizer.Add(self.button22)
		sizer.Add(self.button31)
		sizer.Add(self.button32)
		sizer.Add(self.button41)
		sizer.Add(self.button42)
		sizer.Add(self.button51)
		sizer.Add(self.button52)
		sizer.Add(self.button61)
		sizer.Add(self.button62)
		
		self.mainPanel.SetSizer(sizer)

	def calib(self, event) :
		msg = (0, 'CALIB')
		controller.sendMsg(ARM_ROBOT, msg)

	def setHome(self, event) :
		msg = (0, 'SETHOME')
		controller.sendMsg(ARM_ROBOT, msg)

	def moveHome(self, event) :
		msg = (0, 'MOVEHOME')
		controller.sendMsg(ARM_ROBOT, msg)

	def stop1(self, event) :
		msg = (1,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)

	def stop2(self, event) :
		msg = (2,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)
		
	def stop3(self, event) :
		msg = (3,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)

	def stop4(self, event) :
		msg = (4,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)

	def stop5(self, event) :
		msg = (5,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)

	def stop6(self, event) :
		msg = (6,'STOP')
		controller.sendMsg(ARM_ROBOT, msg)

	def left1(self, event) :
		msg = (1,'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)

	def left2(self, event) :
		msg = (2, 'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)

	def left3(self, event) :
		msg = (3, 'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)

	def left4(self, event) :
		msg = (4, 'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)

	def left5(self, event) :
		msg = (5, 'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)

	def left6(self, event) :
		msg = (6, 'LEFT')
		controller.sendMsg(ARM_ROBOT, msg)
		
	def right1(self, event) :
		msg = (1, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

	def right2(self, event) :
		msg = (2, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

	def right3(self, event) :
		msg = (3, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

	def right4(self, event) :
		msg = (4, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

	def right5(self, event) :
		msg = (5, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

	def right6(self, event) :
		msg = (6, 'RIGHT')
		controller.sendMsg(ARM_ROBOT, msg)

class LocalNetworkPanel(wx.Panel) :
	def __init__(self, parent, id) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		wx.StaticText(self, -1, "Local Network", (10, 10))
		self.cplusServerStatus = wx.StaticText(self, -1, "C++ Server : OFF", (10, 50))
		self.cplusServerOn = wx.Button(self, -1, "Server On", (200, 40))

		self.slamStatus = wx.StaticText(self, -1, "ICP SLAM : OFF", (10, 80))
		self.slamOnbtn = wx.Button(self, -1, "SLAM On", (200, 80))

		self.cplusServerOn.Bind(wx.EVT_LEFT_DOWN, self.localServerOn)
		self.slamOnbtn.Bind(wx.EVT_LEFT_DOWN, self.slamOn)

	def localServerOn(self, event) :
		controller.sendMsg(LOCAL_NETWORK, "server on")
		self.cplusServerStatus.SetLabel("C++ Server : ON")

	def slamOn(self, event) :
		controller.sendMsg(LOCAL_NETWORK, "ICP_SLAM")
		self.slamStatus.SetLabel("ICP SLAM : ON")
		
        
class LeftPanel(wx.Panel) :
	def __init__(self, parent, id) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		self.networkPanel = NetworkPanel(self, -1)
		self.basePanel = BaseRobotPanel(self, -1)

		panels["networkPanel"] = self.networkPanel
		panels["basePanel"] = self.basePanel

		hbox = wx.BoxSizer(wx.VERTICAL)
		hbox.Add(self.networkPanel, 1, wx.EXPAND | wx.ALL, 5)
		hbox.Add(self.basePanel, 1, wx.EXPAND | wx.ALL, 5)		
		self.SetSizer(hbox)
		self.Centre()

class MiddlePanel(wx.Panel) :	
	def __init__(self, parent, id) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		self.linearPanel = LinearRobotPanel(self, -1)
		self.armPanel = ArmRobotPanel(self, -1)

		panels["linearPanel"] = self.linearPanel
		panels["armPanel"] = self.armPanel

		hbox = wx.BoxSizer(wx.VERTICAL)
		hbox.Add(self.linearPanel, 1, wx.EXPAND | wx.ALL, 5)
		hbox.Add(self.armPanel, 1, wx.EXPAND | wx.ALL, 5)		
		self.SetSizer(hbox)
		self.Centre()


	
class RightPanel(wx.Panel) :
	def __init__(self, parent, id) :
		wx.Panel.__init__(self, parent, id, style=wx.BORDER_SUNKEN)
		self.localNetwork = LocalNetworkPanel(self, -1)

		panels["LocalNetworkPanel"] = self.localNetwork

		hbox = wx.BoxSizer(wx.VERTICAL)
		hbox.Add(self.localNetwork, 1, wx.EXPAND | wx.ALL, 5)
		self.SetSizer(hbox)
		self.Centre()

		
		"""
		self.linearPanel = LinearRobotPanel(self, -1)
		self.armPanel = ArmRobotPanel(self, -1)

		panels["linearPanel"] = self.linearPanel
		panels["armPanel"] = self.armPanel

		hbox = wx.BoxSizer(wx.VERTICAL)
		hbox.Add(self.linearPanel, 1, wx.EXPAND | wx.ALL, 5)
		hbox.Add(self.armPanel, 1, wx.EXPAND | wx.ALL, 5)		
		self.SetSizer(hbox)
		self.Centre()
		"""

ID_ABOUT = 101
ID_EXIT = 110

class Menubar_Decorator():
	def __init__(self, frame):
		self.frame = frame
		statusBar = frame.CreateStatusBar()
		statusBar.SetStatusText("Mobile Controller V0.1 ISNL Lab, KAIST, jsharp83@gmail.com")
		
		filemenu = wx.Menu()
		filemenu= wx.Menu()
		filemenu.Append(ID_ABOUT, "&About"," Information about this program")
		filemenu.AppendSeparator()
		filemenu.Append(ID_EXIT,"E&xit"," Terminate the program")
        # Creating the menubar.
		menuBar = wx.MenuBar()
		menuBar.Append(filemenu,"&File") # Adding the "filemenu" to the MenuBar

		wx.EVT_MENU(self.frame, ID_ABOUT, self.OnAbout)
		wx.EVT_MENU(self.frame, ID_EXIT, self.OnExit)
		
		frame.SetMenuBar(menuBar)  # Adding the MenuBar to the Frame content.

	def OnAbout(self, e) :
		d = wx.MessageDialog( self.frame, " Mobile Controller V0.1","About Program", wx.OK)
		d.ShowModal()
		d.Destroy()
	def OnExit(self, e) :
		self.frame.Close(True)

class MainWindow(wx.Frame):
	def __init__(self, parent, id, title, position=(100,100), size=(300,200)):
		wx.Frame.__init__(self, parent, id, title, position, size)

		self.mainPanel = wx.Panel(self, -1)
		self.leftPanel = LeftPanel(self.mainPanel, -1)
		self.rightPanel = RightPanel(self.mainPanel, -1)
		self.middlePanel = MiddlePanel(self.mainPanel, -1)

		panels["main"] = self.mainPanel
		panels["leftPanel"] = self.leftPanel
		panels["rightPanel"] = self.rightPanel
		panels["middlePanel"] = self.middlePanel

		hbox = wx.BoxSizer(wx.HORIZONTAL)
		hbox.Add(self.leftPanel, 1, wx.EXPAND | wx.ALL, 5)
		hbox.Add(self.middlePanel, 1, wx.EXPAND | wx.ALL, 5)
		hbox.Add(self.rightPanel, 1, wx.EXPAND | wx.ALL, 5)		
		self.mainPanel.SetSizer(hbox)
		
		self.Centre()
		controller.setView(self)

	def update(self, name, msg) :
		if name == BASE_ROBOT :
			panels["basePanel"].updatePortState(msg)
		elif name == LINEAR_ROBOT :
			panels["linearPanel"].updatePortState(msg)
		elif name == ARM_ROBOT :
			panels["armPanel"].updatePortState(msg)
		

if __name__=="__main__" :
	app = wx.PySimpleApp()
	frm = MainWindow(None, -1,"Mobile Controller V0.1",Initial_Position,Initial_Window_Size)
	Menubar_Decorator(frm)
	frm.Show()
	app.MainLoop()

