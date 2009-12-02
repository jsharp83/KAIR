from socket import *


from Configuration import *


class ClientMain() :
	def __init__(self, serverIP, serverPort) :
		self.clientsock = socket(AF_INET, SOCK_STREAM)
		self.serverInfo = (serverIP, serverPort)

	def connectToServer(self) :
		print "Connect to Server..."
		self.clientsock.connect(self.serverInfo)

	def sendMsgToServer(self, name, msg) :
#		print "SendMsg : ", name,msg
		self.clientsock.send(name)

		
		"""ACK"""
		self.clientsock.recv(2)

		if name == BASE_ROBOT :
			value = msg[1]			
			sendMsg = msg[0]+","+str(value[0])+","+str(value[1])
			self.clientsock.send(sendMsg)

		elif name == PANTILT_UNIT :
			if len(msg) == 2 :
				value = msg[1]
				sendMsg = msg[0]+","+str(value)
				self.clientsock.send(sendMsg)
			elif len(msg) == 3 :
				sendMsg = msg[0]+ ","+msg[1]+","+str(msg[2])
				self.clientsock.send(sendMsg)
		elif name == ARM_ROBOT :
			sendMsg = str(msg[0])+","+msg[1]
			self.clientsock.send(sendMsg)

		else :	
			self.clientsock.send(msg)

		"""ACK"""
		self.clientsock.recv(2)

class ClientToWindowServer() :
	def __init__(self, serverIP, serverPort) :
		print "Linux Client Start.."
		self.clientsock = socket(AF_INET, SOCK_STREAM)
		self.serverInfo = (serverIP, serverPort)

	def connectToServer(self) :
		print "Connect to Server..."
		self.clientsock.connect(self.serverInfo)
		
	def sendMsgToServer(self, name, msg) :
		self.clientsock.send(name)
		self.clientsock.recv(2) # ACK
		self.clientsock.send(msg)
		self.clientsock.recv(2) # ACK
		print "Send msg to Window Server"


if __name__ == "__main__" :
	clientsock = socket(AF_INET, SOCK_STREAM)
	clientsock.connect(('127.0.0.1', 7799))
	while True :
		print "Send HI"
		clientsock.send("hi,!!!")
		time.sleep(3)
		
	clientsock.close()
	print "client end"
