#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
using namespace std;

void DieWithError(char *errorMessage);
int icp_Main();

static const int BUFSIZE = 512;

int sock;
struct sockaddr_in serverAddr;

void initSocket(){
  //  struct sockaddr_in serverAddr;
  unsigned short servPort = 7798;
  char* servIP = "127.0.0.1";
  //  char buffer[BUFSIZE];

  if((sock=socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	DieWithError("socket() failed");

  memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = inet_addr(servIP);
  serverAddr.sin_port = htons(servPort);
}

void handleClient(){

  int recvBufSZ = 64;
  char recvBuf[recvBufSZ];
  int pid;

  pid = fork();

  if(pid >0){
	cout << "Create Client Thread... " << endl;
  }else if (pid == 0){
  
	while(1){
	  recv(sock, recvBuf, recvBufSZ, 0);

	  if(strcmp(recvBuf, "ICP_SLAM")){
		cout << "ICP SLAM START...." << endl;
        icp_Main();
		

	  }else{
		cout << "receive wrong message..." << endl;
	  }

	}
  }else if(pid == -1){
	cout << "fork error..." << endl;
  }

}


int main()
{
  cout << "C++ Client Start...\n" << endl;

  initSocket();
  cout << " Init Socket..." << endl;

  if(connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	DieWithError("connect() failed");

  char buf[15] = "C++LOCALCLIENT";
  char ackBuf[4];
  
  send(sock, buf, strlen(buf), 0);
  recv(sock, ackBuf,strlen(ackBuf), 0);

  handleClient();


  cout << "finish to send a message" << endl; 
  
  return 0;
}
	
void DieWithError(char *errorMessage){
  printf("%s", errorMessage);
  exit(1);
}
