import sys
sys.path.append('../')

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
	def __init__(self, parent, Controller) :
		Thread.__init__(self)
		self.parent = parent
		self.Controller = Controller

	def setClient(self, clientSocket, addr) :
		self.clientSocket = clientSocket
		self.clientSocket.setblocking(1)
		self.addr = addr

	def run(self) :
		while True :
			recvName = self.clientSocket.recv(32)
			self.clientSocket.send('OK')
			recvMsg = self.clientSocket.recv(64)
			self.clientSocket.send('OK')
			
			if recvName == '' :
				break
			elif recvMsg == '' :
				break

			print recvName, recvMsg, "from ", self.addr

			if recvName == BASE_ROBOT :
				recvDatas = recvMsg.split(',')
				recvMsg = (recvDatas[0],( int(recvDatas[1]), int(recvDatas[2])))
			elif recvName == PANTILT_UNIT :
				recvDatas = recvMsg.split(',')
				if len(recvDatas) == 2 :
					recvMsg = (recvDatas[0], int(recvDatas[1]))
				elif len(recvDatas) == 3 :
					recvMsg = (recvDatas[0], recvDatas[1], int(recvDatas[2]))
			elif recvName == ARM_ROBOT :
				recvDatas = recvMsg.split(',')				
				recvMsg = (int(recvDatas[0]), recvDatas[1])

			self.Controller.sendMsg(recvName, recvMsg)

		self.parent.removeClient(self.addr)

clientSocketList = {}

class LocalServerThread(ServerThread) :
	def __init__(self, parent, Controller) :
		ServerThread.__init__(self, parent, Controller)
		self.parent = parent
		self.Controller = Controller

	def run(self) :
		while True :

			recvName = self.clientSocket.recv(15)

			if recvName == "C++LOCALCLIENT" :
				self.clientSocket.send("ack\0")
				print "c++ LOCAL Client ..."

				clientSocketList["cplus"] = self.clientSocket

			if recvName == '' :
				break

			"""
			elif recvMsg == '' :
				break
			"""

		self.parent.removeClient(self.addr)

"""
class LinuxServerThread(ServerThread) :
	def __init__(self, parent, Controller) :
		ServerThread.__init__(self, parent, Controller)
		self.parent = parent
		self.Controller = Controller

	def run(self) :
		while True :

			recvName = self.clientSocket.recv(15)

			if recvName == "WINDOWSERVER" :
				self.clientSocket.send("ack\0")
				print "connect to window server"

				clientSocketList["WINDOWSERVER"] = self.clientSocket

			if recvName == '' :
				break

		self.parent.removeClient(self.addr)


class LinuxServer(Thread) :
	def __init__(self,Controller) :
		Thread.__init__(self)
		print "Python Linux-Local server start..\n"
		self.svrsock = socket(AF_INET, SOCK_STREAM)
		self.svrsock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
		self.svrsock.bind((LINUX_SERVER_IP, LINUX_SERVER_PORT))
		self.svrsock.listen(5)		
		self.clientList = {}
		self.Controller = Controller

	def run(self) :
		while True :
			print "Wait connection in LocalMain"
			conn, addr = self.svrsock.accept()
			print "Connect from ", addr
			LinuxServerThread = LinuxServerThread(self,self.Controller)
			LinuxServerThread.setClient(conn, addr)
			LinuxServerThread.start()
			self.clientList[addr] = (conn, LinuxServerThread)

	def removeClient(self,addr) :
		print "Remove ", addr, "from client list"
		del self.clientList[addr]
"""		

class LocalServer(Thread) :
	def __init__(self,Controller) :
		Thread.__init__(self)
		print "Python Local server start..\n"
		self.svrsock = socket(AF_INET, SOCK_STREAM)
		self.svrsock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
		self.svrsock.bind((LOCALHOST, LOCAL_PORT))
		self.svrsock.listen(5)		
		self.clientList = {}
		self.Controller = Controller

	def run(self) :
		while True :
			print "Wait connection in LocalMain"
			conn, addr = self.svrsock.accept()
			print "Connect from ", addr
			localServerThread = LocalServerThread(self,self.Controller)
			localServerThread.setClient(conn, addr)
			localServerThread.start()
			self.clientList[addr] = (conn, localServerThread)

	def removeClient(self,addr) :
		print "Remove ", addr, "from client list"
		del self.clientList[addr]
	
	

class ServerMain(Thread) :
	def __init__(self,Controller) :
		Thread.__init__(self)
		print "Python main server start..\n"
		self.svrsock = socket(AF_INET, SOCK_STREAM)
		self.svrsock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
		self.svrsock.bind((SERVER_IP, SERVER_PORT))
		self.svrsock.listen(5)		
		self.clientList = {}
		self.Controller = Controller

	def run(self) :
		while True :
			print "Wait connection in ServerMain"
			conn, addr = self.svrsock.accept()
			print "Connect from ", addr
			serverThread = ServerThread(self,self.Controller)
			serverThread.setClient(conn, addr)
			serverThread.start()
			self.clientList[addr] = (conn, serverThread)

	def removeClient(self,addr) :
		print "Remove ", addr, "from client list"
		del self.clientList[addr]

if __name__ == "__main__" :
	server = InitThe(ServerMain)
	server.start()
