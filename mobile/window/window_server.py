import sys
sys.path.append('../configuration')

from socket import *
from threading import Thread
from Configuration import *

# singleton
def The( singleton, *args ) :
    if not hasattr( singleton, "__instance" ) :
        singleton.__instance = singleton( *args )

    return singleton.__instance

def InitThe( singleton, *args ) :
    if hasattr( singleton, "__instance" ) :
        delattr( singleton, "__instance" )

    return The( singleton, *args )


class ServerThread(Thread) :
	def __init__(self, parent) :
		Thread.__init__(self)
		self.parent = parent

		import NIDeviceWrapper
		self.NI = NIDeviceWrapper
		self.NI.initDevice()

	def setClient(self, clientSocket, addr) :
		self.clientSocket = clientSocket
		self.clientSocket.setblocking(1)
		self.addr = addr

	def run(self) :
		while True :
			recvName = self.clientSocket.recv(32)
			if recvName == '' :
				break
			self.clientSocket.send("OK")
			
			recvMsg = self.clientSocket.recv(128)
			if recvMsg == '' :
				break
			self.clientSocket.send("OK")			

			print recvName, recvMsg, "from ", self.addr

			if recvName == BASE_ROBOT :
				splitedMsgs = recvMsg.split(' ')
				if splitedMsgs[0] == BASE_VELOCITY :
					(left, right)= float(splitedMsgs[1]), float(splitedMsgs[2])
					print left, right
					self.NI.updateVoltages(left, right)
			else :
				print "Recv Error in Server Thread"
			
		self.parent.removeClient(self.addr)

class WindowServerMain() :
	def __init__(self) :
		print "Python Window server start..\n"
		self.svrsock = socket(AF_INET, SOCK_STREAM)
		self.svrsock.bind((WINDOW_SERVER_IP, WINDOW_SERVER_PORT))
		self.svrsock.listen(5)		
		self.clientList = {}

	def serverStart(self) :
		while True :
			print "Wait connection in WindowServerMain"
			conn, addr = self.svrsock.accept()
			print "Connect from ", addr
			serverThread = ServerThread(self)
			serverThread.setClient(conn, addr)
			serverThread.start()
			self.clientList[addr] = (conn, serverThread)

	def removeClient(self,addr) :
		print "Remove ", addr, "from client list"
		del self.clientList[addr]

if __name__ == "__main__" :
	server = InitThe(WindowServerMain)
	server.serverStart()
