#include <mrpt/core.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <mrpt/system/os.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>


using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace std;

void encoderTest();
void DieWithError(char *errorMessage);
void initSocket();

static const int BUFSIZE = 512;

int sock;
struct sockaddr_in serverAddr;

void initSocket(){
  //  struct sockaddr_in serverAddr;
  unsigned short servPort = 7797;
  char* servIP = "143.248.135.121";
  int flag = 1;
  //  char buffer[BUFSIZE];

  if((sock=socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	DieWithError("socket() failed");

  memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = inet_addr(servIP);
  serverAddr.sin_port = htons(servPort);
  
}

int main(int argc, char **argv){
  cout << "Encoder test\n" << endl;
  encoderTest();
}

void encoderTest(){

  cout << "C++ Client Start...\n" << endl;

  initSocket();
  cout << " Init Socket..." << endl;

  if(connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	DieWithError("connect() failed\n");

  char* name1 = "BASE\0";
  char* name2 = "RECEIVE_ENCODER\0";
  char ack[4];
  char data[64];

  FILE *fp;


  write(sock, name1, strlen(name1));
  fp = fdopen(sock,"rw");
  fflush(fp);
    
  printf("strlen %d", strlen(name1));

  //  int i = recv(sock, ack, strlen(ack), 0);


  int i = read(sock, ack, 4);
  printf("recv ack %s %d", ack, i);

  write(sock, name2, strlen(name2));
  fflush(fp);

  i = read(sock, ack, 4);
  printf("recv ack %s %d\n", ack, i);
  

#if MRPT_HAS_WXWIDGETS
  CDisplayWindowPlots		win("Encoder Test.");
#endif

  CActionRobotMovement2D                              actMov;
  CActionRobotMovement2D::TMotionModelOptions         opts;

  opts.modelSelection = CActionRobotMovement2D::mmThrun;
  opts.thrunModel.alfa3_trans_trans = 0.10f;

  actMov.hasEncodersInfo = true;
  actMov.encoderLeftTicks = 0;
  actMov.encoderRightTicks = 0;
  double tickRatio = 2*3.14*0.2/37000;
  double diameter = 0.55;
  char *p;

  while(1){
    write(sock, "OK\0", 4);
    fflush(fp);
    i = read(sock, data, 64);

    p = strtok(data, " ");
    double left = atof(p);
    p = strtok(NULL, " ");
    double right = atof(p);

    cout << "LEFT " << left << " RIGHT " << right;

    actMov.encoderLeftTicks = left;
    actMov.encoderRightTicks = right;
    
        
    actMov.computeFromEncoders(tickRatio,tickRatio,diameter);

    vector_float xs, ys;

    for(int i = 0 ; i < 1000; i++){
      CPose2D     sample;
      actMov.drawSingleSample(sample);

      xs.push_back(sample.x());
      ys.push_back(sample.y());

    }

#if MRPT_HAS_WXWIDGETS

    win.plot(xs,ys,".r3","t1");
    win.axis_equal();

    //	win.axis_fit();
#endif
    sleep(10);
  }
}

void DieWithError(char *errorMessage){
  printf("%s", errorMessage);
  exit(1);
}
