import wx
from dynamixel_AX12 import Dxl_AX12

PANTILT_MIN = 200
PANTILT_MAX = 800
PANTILT_MIDDLE = 500

HOKUYO_ID = 2
CAMERA_PAN = 0
CAMERA_TILT = 1
DXL_SPEED = 200
DX_PORT = "/dev/ttyUSB0"

class MainWindow(wx.Frame):
	def __init__(self, parent, id, title, position=(100,100), size = (400,200)):
		wx.Frame.__init__(self, parent, id, title, position, size)

		self.dxl = Dxl_AX12(DX_PORT,57600)
		
		self.mainPanel = wx.Panel(self, -1)

		self.text1 = wx.StaticText(self.mainPanel, -1, "Hokuyo",pos=(10,50))		
		self.panSlider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,70),style=wx.SL_HORIZONTAL)

		self.text2 = wx.StaticText(self.mainPanel, -1, "Camera, Pan",pos=(10,100))
		self.t_Slider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,120),style=wx.SL_HORIZONTAL)

		self.text3 = wx.StaticText(self.mainPanel, -1, "Camera, Tilt",pos=(10,150))
		self.p_Slider = wx.Slider(self.mainPanel, -1, value = PANTILT_MIDDLE, minValue=PANTILT_MIN, maxValue=PANTILT_MAX, size= (180,-1),pos=(10,170),style=wx.SL_HORIZONTAL)
	
		self.panSlider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changePanValue)
		self.t_Slider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changeTValue)
		self.p_Slider.Bind(wx.EVT_SCROLL_THUMBTRACK, self.changePValue)		

		self.mainPanel.Centre()
		self.Centre()

	def changePanValue(self, event) :
		value= self.panSlider.GetValue()
		self.sendToDxl(HOKUYO_ID, value)

	def changeTValue(self, event) :
		value = self.t_Slider.GetValue()
		self.sendToDxl(CAMERA_PAN, value)
		
	def changePValue(self, event) :
		value = self.p_Slider.GetValue()
		self.sendToDxl(CAMERA_TILT, value)

	def sendToDxl(self, id, position) :
		self.dxl.setPosition(id, position, DXL_SPEED)
		

if __name__ == "__main__" :

	app = wx.PySimpleApp()
	frm = MainWindow(None, -1,"Dynamixel Pantilt Test")
	frm.Show()
	app.MainLoop()
	
