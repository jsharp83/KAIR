#include "SickLMS200_flymake.h"
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
//#include <stdio.h>
//#include <sys/time.h>
//#include <time.h>
#include <signal.h>
//#include <getopt.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <stdbool.h>
#include <iostream>

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;


//#include <rtt/Logger.hpp>

//#define FALSE 0
//#define TRUE 1
#define MAXRETRY 100
#define MAXNDATA 802
#define STX 0x02   /*every PC->LMS packet is started by STX*/
#define ACKSTX 0x06 /*every PC->LMS packet is started by ACKSTX*/
typedef unsigned char uchar;

//using namespace RTT;


namespace SickDriver {
/*the cmd and ack packets for the 5 different range/resolution modes*/
const uchar PCLMS_RES1[]={0x02,0x00,0x05,0x00,0x3b,0x64,0x00,0x64,0x00,0x1d,0x0f};
const uchar PCLMS_RES2[]={0x02,0x00,0x05,0x00,0x3b,0x64,0x00,0x32,0x00,0xb1,0x59};
const uchar PCLMS_RES3[]={0x02,0x00,0x05,0x00,0x3b,0x64,0x00,0x19,0x00,0xe7,0x72};
const uchar PCLMS_RES4[]={0x02,0x00,0x05,0x00,0x3b,0xb4,0x00,0x64,0x00,0x97,0x49};
const uchar PCLMS_RES5[]={0x02,0x00,0x05,0x00,0x3b,0xb4,0x00,0x32,0x00,0x3b,0x1f};
const uchar LMSPC_RES1_ACK[]={0x06,0x02,0x80,0x07,0x00,0xbb,0x01,0x64,0x00,0x64,0x00,0x10,0x4f,0xbd};
const uchar LMSPC_RES2_ACK[]={0x06,0x02,0x80,0x07,0x00,0xbb,0x01,0x64,0x00,0x32,0x00,0x10,0x17,0x10};
const uchar LMSPC_RES3_ACK[]={0x06,0x02,0x80,0x07,0x00,0xbb,0x01,0x64,0x00,0x19,0x00,0x10,0xbb,0x46};
const uchar LMSPC_RES4_ACK[]={0x06,0x02,0x80,0x07,0x00,0xbb,0x01,0xb4,0x00,0x64,0x00,0x10,0x5b,0x30};
const uchar LMSPC_RES5_ACK[]={0x06,0x02,0x80,0x07,0x00,0xbb,0x01,0xb4,0x00,0x32,0x00,0x10,0x03,0x9d};

/*the cmd and ack packets different measurement unit modes*/
const uchar PCLMS_SETMODE[]={0x02,0x00,0x0a,0x00,0x20,0x00,0x53,0x49,0x43,0x4b,0x5f,0x4c,0x4d,0x53,0xbe,0xc5};
const uchar PCLMS_CM[]={0x02,0x00,0x21,0x00,0x77,0x00,0x00,0x00,0x00,0x00,0x0d,0x00,0x00,0x00,\
                        0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0xcb};
const uchar PCLMS_MM[]={0x02,0x00,0x21,0x00,0x77,0x00,0x00,0x00,0x00,0x00,0x0d,0x01,0x00,0x00,\
                        0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x34,0xc7};
const uchar LMSPC_CM_ACK[]={0x06,0x02,0x80,0x25,0x00,0xf7,0x00,0x00,0x00,0x46,0x00,0x00,0x0d,0x00,\
                            0x00,0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0xcb,0x10};
const uchar LMSPC_MM_ACK[]={0x06,0x02,0x80,0x25,0x00,0xf7,0x00,0x00,0x00,0x46,0x00,0x00,0x0d,0x01,\
                            0x00,0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0xc7,0x10};

/*the cmd packets for setting transfer speedn and  controlling the start and stop of measurement*/
const uchar PCLMS_START[]={0x02,0x00,0x02,0x00,0x20,0x24,0x34,0x08};
const uchar PCLMS_STOP[]={0x02,0x00,0x02,0x00,0x20,0x25,0x35,0x08};
const uchar PCLMS_STATUS[]={0x02,0x00,0x01,0x00,0x31,0x15,0x12};
const uchar PCLMS_B9600[] ={0x02,0x00,0x02,0x00,0x20,0x42,0x52,0x08};
const uchar PCLMS_B19200[]={0x02,0x00,0x02,0x00,0x20,0x41,0x51,0x08};
const uchar PCLMS_B38400[]={0x02,0x00,0x02,0x00,0x20,0x40,0x50,0x08};
/*the ack packet for the above*/
const uchar LMSPC_CMD_ACK[]={0x06,0x02,0x80,0x03,0x00,0xa0,0x00,0x10,0x16,0x0a};
const int CRC16_GEN_POL = 0x8005;
	



bool SickLMS200::msgcmp(int len1, const uchar *s1, int len2, const uchar *s2)
{
  int i;
  if(len1!=len2) return false;
  for(i=0; i<len1; i++)
    if(s1[i]!=s2[i]) return false;
  return true;
}


void SickLMS200::wtLMSmsg(int fd, int len, const uchar *msg)
{
  write(fd,(const void*) msg,len);
}

int SickLMS200::rdLMSmsg(int fd, int len, const uchar *buf)
{
  int sumRead=0,nRead=0,toRead=len,n;
  while(toRead>0){
    n=toRead>255 ? 255:toRead;
    toRead-=n;
    nRead=read(fd,(void *)(buf+sumRead),n);
    sumRead+=nRead;
    if(nRead!=n) break;
  }
  return nRead;
}

uchar SickLMS200::rdLMSbyte(int fd)
{
  uchar buf;
  read(fd,&buf,1);
  return buf;
}



/*return true if the ACK packet is as expected*/
bool SickLMS200::chkAck(int fd, int ackmsglen, const uchar *ackmsg)
{
  int i,buflen;
  uchar buf[MAXNDATA];

  /*the following is to deal with a possibly timing issue*/
  /*the communication occasionally breaks without this*/
  usleep(1000);
  for(i=0;i<MAXRETRY;i++) {
    if(rdLMSbyte(fd)==0x06)
    {
       break;
    }
  }
  buflen=rdLMSmsg(fd,ackmsglen-1,buf);
  return msgcmp(ackmsglen-1,ackmsg+1,buflen,buf);
}



/*set the communication speed and terminal properties*/
bool SickLMS200::initLMS(const char *serialdev, struct termios *oldtio, int& fd)
{
  struct termios newtio_struct, *newtio=&newtio_struct;

  fd = open(serialdev, O_RDWR | O_NOCTTY );
  if (fd <0) {
//    log(Error)<< " SickLMS200 IOException."<<endlog();
	  cout << "SickLMS200 Init Error, fd = " << fd <<" \n" << endl;

    return false;
  }else
  {
	  cout << "fd is " << fd << endl;
  }
  

  tcgetattr(fd,oldtio); /* save current port settings */

  /*after power up, the laser scanner will reset to 9600bps*/
  memset(newtio, 0, sizeof(struct termios));
  newtio->c_cflag = B9600 | CS8 | CLOCAL | CREAD;
  newtio->c_iflag = IGNPAR;
  newtio->c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio->c_lflag = 0;
  newtio->c_cc[VTIME]    = 10;   /* inter-character timer unused */
  newtio->c_cc[VMIN]     = 255;   /* blocking read until 1 chars received */
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,newtio);


  /* step to the 38400bps mode*/
  wtLMSmsg(fd,sizeof(PCLMS_B38400)/sizeof(uchar),PCLMS_B38400);
  if(!chkAck(fd,sizeof(LMSPC_CMD_ACK)/sizeof(uchar),LMSPC_CMD_ACK)){
//    log(Error)<< " SickLMS200 BaudRateChangeException."<<endlog();
	  cout << "SickLMS200 BaudRateChangeException." << endl;
	  return true;
//    throw BaudRateChangeException();
  }

  /* set the PC side as well*/
  close(fd);
  fd = open(serialdev, O_RDWR | O_NOCTTY );
  if (fd <0) {
//    log(Error)<< " SickLMS200 IOException."<<endlog();
	  cout << "SICKLMS200 Reopen Error" << endl;
    return false;
  }
  newtio->c_cflag = B38400 | CS8 | CLOCAL | CREAD;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,newtio);

  return true;
}


/*set both the angular range and resolutions*/
bool SickLMS200::setmode(int fd, int mode)
{
  const uchar *msg, *ackmsg;
  int msglen, ackmsglen, unit, res;

  /*change the resolution*/
  res=mode & 0x3e;
  switch(res){
  case (RES_1_DEG | RANGE_100):
    msg=PCLMS_RES1;
    ackmsg=LMSPC_RES1_ACK;
    break;
  case (RES_0_5_DEG | RANGE_100):
    msg=PCLMS_RES2;
    ackmsg=LMSPC_RES2_ACK;
    break;
  case (RES_0_25_DEG | RANGE_100):
    msg=PCLMS_RES3;
    ackmsg=LMSPC_RES3_ACK;
    break;
  case (RES_1_DEG | RANGE_180):
    msg=PCLMS_RES4;
    ackmsg=LMSPC_RES4_ACK;
    break;
  case (RES_0_5_DEG | RANGE_180):
    msg=PCLMS_RES5;
    ackmsg=LMSPC_RES5_ACK;
    break;
  default:
    msg=PCLMS_RES1;
    ackmsg=LMSPC_RES1_ACK;
//    log(Error)<< " SickLMS200 InvalidResolutionException."<<endlog();
	cout << "SickLMS200 invalidResolution" << endl;
    return false;
    break;
  }

  /*the following two line works only because msg and ackmsg are const uchar str*/
  msglen=sizeof(PCLMS_RES1)/sizeof(uchar);
  ackmsglen=sizeof(LMSPC_RES1_ACK)/sizeof(uchar);
  wtLMSmsg(fd,msglen,msg);
  if(!chkAck(fd,ackmsglen,ackmsg)){
//    log(Error)<< " SickLMS200 ResolutionFailureException."<<endlog();
	  cout << "SickLMS200 ResolutionFailure" << endl;
    return false;
  }

  /*change the measurement unit*/
  unit=mode & 0x1;
  /*may need to increase the timeout to 7sec here*/
  if(unit==MMMODE){
    msg=PCLMS_MM;
    ackmsg=LMSPC_MM_ACK;
  }else if(unit==CMMODE){
    msg=PCLMS_CM;
    ackmsg=LMSPC_CM_ACK;
  }
  /*invoking setting mode*/
  msglen=sizeof(PCLMS_SETMODE)/sizeof(uchar);
  ackmsglen=sizeof(LMSPC_CMD_ACK)/sizeof(uchar);
  wtLMSmsg(fd,msglen,PCLMS_SETMODE);
  if(!chkAck(fd,ackmsglen,LMSPC_CMD_ACK)){
//    log(Error)<< " SickLMS200 ModeFailureException."<<endlog();
	  cout << "SickLMS200 ModeFailureExcetpion" << endl;
    return false;
  }
  /*the following two line works only because msg and ackmsg are const uchar str*/
  msglen=sizeof(PCLMS_MM)/sizeof(uchar);
  ackmsglen=sizeof(LMSPC_MM_ACK)/sizeof(uchar);
  wtLMSmsg(fd,msglen,msg);
  if(!chkAck(fd,ackmsglen,ackmsg)){
//    log(Error)<< " SickLMS200 ModeFailureException."<<endlog();
	  cout << "SickLMS200 ModelfailureException " << endl;
    return false;
  }
  return true;
}

/*tell the scanner to enter the continuous measurement mode*/
bool SickLMS200::startLMS(int fd)
{
//  log(Debug)<< " SickLMS200::startLMS entered."<<endlog();
	cout << "SickLMS200 StartLMS entered " << endl;
  int ackmsglen;

  wtLMSmsg(fd,sizeof(PCLMS_START)/sizeof(uchar),PCLMS_START);
  ackmsglen=sizeof(LMSPC_CMD_ACK)/sizeof(uchar);
  return chkAck(fd,ackmsglen,LMSPC_CMD_ACK);
}

/*stop the continuous measurement mode*/
bool SickLMS200::stopLMS(int fd)
{
  int ackmsglen;

  wtLMSmsg(fd,sizeof(PCLMS_STOP)/sizeof(uchar),PCLMS_STOP);
  ackmsglen=sizeof(LMSPC_CMD_ACK)/sizeof(uchar);
  return chkAck(fd,ackmsglen,LMSPC_CMD_ACK);
}


bool SickLMS200::checkErrorMeasurement() {
  switch(meas_state & 0x07){
    case 0: return false;
    case 1: return false;
    case 2: return false;
    case 3: return true;
    case 4: {
//      log(Error)<< " SickLMS200 FatalMeasurementException."<<endlog();
		cout << "SickLMS200 FatalMeseumentException" << endl;
      return false;
    }
    default: {
//		log(Error)<< " SickLMS200 FatalMeasurementException."<<endlog();
		cout << "SickLMS200 FatalMeseumentException" << endl;
      return false;
    }
  }
}

bool SickLMS200::checkPlausible() {
    return (meas_state & 0xC0)==0;
}



/*reset terminal and transfer speed of laser scanner before quitting*/
bool SickLMS200::resetLMS(int fd, struct termios *oldtio)
{
  wtLMSmsg(fd,sizeof(PCLMS_B9600)/sizeof(uchar),PCLMS_B9600);
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,oldtio);
  close(fd);
  return chkAck(fd,sizeof(LMSPC_CMD_ACK)/sizeof(uchar),LMSPC_CMD_ACK);
}

SickLMS200::SickLMS200(const char* _port, uchar _range_mode, uchar _res_mode, uchar _unit_mode) {
	cout << "Create Sick Driver..." << endl;
    port = _port;
    range_mode = _range_mode;
    res_mode   = _res_mode;
    unit_mode  = _unit_mode;
}

bool SickLMS200::start() {
	cout << "Start Sick Driver..." << endl;
	cout << "PORT : " << port << " range_mode : " << range_mode << endl;
    if (!initLMS(port,&oldtio,fd))
	{
		return false;
	}
	
    if (!setmode(fd, range_mode | res_mode | unit_mode))
	{
		return false;
	}
	
    return startLMS(fd);
}
bool SickLMS200::stop() {
		cout << "Stop LMS 200 ...." << endl;
    bool ret1 = stopLMS(fd);
    bool ret2 = resetLMS(fd,&oldtio);

	close(fd);

    return ret1 && ret2;
}
bool SickLMS200::reset() {
  return resetLMS(fd,&oldtio);
}

SickLMS200::~SickLMS200() {
}

/**
 * returns :
 *   -1 : when the input does not begin with the correct header
 */
int SickLMS200::readMeasurement(uchar* buf,int& datalen) {
    if(rdLMSbyte(fd)!=STX) return -1;
    rdLMSbyte(fd); /*should be the ADR byte, ADR=0X80 here*/
    /*LEN refers to the packet length in bytes, datalen
      refers to the number of measurements, each of them are 16bits long*/
    rdLMSbyte(fd); /*should be the LEN low byte*/
    rdLMSbyte(fd); /*should be the LEN high byte*/
    rdLMSbyte(fd); /*should be the CMD byte, CMD=0xb0 in this mode*/
    datalen=rdLMSbyte(fd);
    datalen |= (rdLMSbyte(fd) & 0x1f) << 8; /*only lower 12 bits are significant*/
    datalen *= 2; /*each reading is 2 bytes*/
    rdLMSmsg(fd,datalen,buf);
    meas_state = rdLMSbyte(fd);
    rdLMSbyte(fd); /*should be CRC low byte*/
    rdLMSbyte(fd); /*should be CRC high byte*/
    return 0;
}

/*-------------------------------------------------------------
  doProcessSimple
  -------------------------------------------------------------*/
void  SickLMS200::doProcessSimple(
	bool							&outThereIsObservation,
	mrpt::slam::CObservation2DRangeScan	&outObservation,
	bool							&hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	if ( !checkControllerIsConnected() )
	{
		hardwareError = true;
		return;
	}

	cout << "doProcessSimple Start..." << endl;

	vector_float 	ranges;
//	unsigned char	LMS_stat;
//	uint32_t		board_timestamp;
	bool			is_mm_mode = true;

//    m_state = ssWorking;

	uchar buf[1024];
	int datalen=0;

	cout << "ready to read..." << endl;
	// Wait for a scan:
	readMeasurement(buf, datalen);
	cout << " end to read..." << endl;

	if(datalen<=0)
	{
		cout << "Error in readMeasurement" << endl;
		return;
	}
	

//	ranges.resize();
//	double tempBuf[1024];
	int j = 0;
	ranges.resize(datalen/2);
	cout << "Range Resize : " << datalen/2 << endl;

	for(int i = 0 ; i< datalen;i=i+2)
	{
		ranges[j]= ((double)( (buf[i+1] & 0x1f) <<8  |buf[i]))/1000.0;
		j++;
	}

		
//	if (!waitContinuousSampleFrame( ranges,LMS_stat, board_timestamp, is_mm_mode ))
//		return;

	// Yes, we have a new scan:

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
//	uint32_t AtUI = 0;
//	if( m_timeStartUI == 0 )
//	{
//		m_timeStartUI = board_timestamp;
//		m_timeStartTT = mrpt::system::now();
//	}
//	else	AtUI = board_timestamp - m_timeStartUI;

//	mrpt::system::TTimeStamp AtDO	=  mrpt::system::secondsToTimestamp( AtUI * 1e-3 /* Board time is ms */ );
//	outObservation.timestamp = m_timeStartTT + AtDO;
	outObservation.timestamp = mrpt::system::now();
	outObservation.sensorLabel = "LMS200";

poses:CPose3D  m_sensorPose = CPose3D(0,0,0,0,0,0);
//		configSource.read_float(iniSection,"pose_x",0),
//		configSource.read_float(iniSection,"pose_y",0),
//		configSource.read_float(iniSection,"pose_z",0),
//		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
//		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
//		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
//		);


//	outObservation.sensorLabel  = m_sensorLabel;	// Set label

	// Extract the timestamp of the sensor:

	// And the scan ranges:
	outObservation.rightToLeft = true;
	outObservation.aperture = M_PIf;
	outObservation.maxRange	= is_mm_mode ? 32.7 : 81.0;
	outObservation.stdError = 0.003f;
	outObservation.sensorPose = m_sensorPose;

	outObservation.scan = ranges;
	outObservation.validRange.resize(ranges.size());

	for (size_t i=0;i<ranges.size();i++)
		outObservation.validRange[i] = (outObservation.scan[i] <= outObservation.maxRange);

	outThereIsObservation = true;
}

bool SickLMS200::checkControllerIsConnected()
{
	return true;
}


/*-------------------------------------------------------------
  waitContinuousSampleFrame
  -------------------------------------------------------------*/
	bool  SickLMS200::waitContinuousSampleFrame(
		vector_float 	&out_ranges_meters,
		unsigned char 	&LMS_status,
		uint32_t 		&out_board_timestamp,
		bool 			&is_mm_mode )
	{
//		size_t 	nRead,nBytesToRead;
		int nRead, nBytesToRead;
		size_t	nFrameBytes = 0;
		size_t	lenghtField;
		unsigned char	buf[2000];
		buf[2]=buf[3]=0;

		while ( nFrameBytes < (lenghtField=( 6 + (buf[2] | (buf[3] << 8))) ) + 5  /* for 32bit timestamp + end-flag */  )
		{
			if (lenghtField>800)
			{
				cout << "#";
				nFrameBytes = 0;	// No es cabecera de trama correcta
				buf[2]=buf[3]=0;
			}

			if (nFrameBytes<4)
				nBytesToRead = 1;
			else
				nBytesToRead = (5 /* for 32bit timestamp + end-flag */ + lenghtField) - nFrameBytes;

			try
			{
				readMeasurement( buf+nFrameBytes,nBytesToRead );
				nRead = nBytesToRead; // Temporary....
			}
			catch (std::exception &e)
			{
				// Disconnected?
//				printf_debug("[CSickLaserUSB::waitContinuousSampleFrame] Disconnecting due to comms error: %s\n", e.what());
				cout << "Error in readMeasument..." << endl;
				stop();
//				m_usbConnection->Close();
//				m_timeStartUI = 0;
				return false;
			}

			if ( nRead != nBytesToRead )
			{
				return false;
			}
			else
				if (nRead>0)
				{
					// Lectura OK:
					// Era la primera?
					if (nFrameBytes>1 || (!nFrameBytes && buf[0]==0x02) || (nFrameBytes==1 && buf[1]==0x80))
						nFrameBytes+=nRead;
					else
					{
						nFrameBytes = 0;	// No es cabecera de trama correcta
						buf[2]=buf[3]=0;
					}
				}
		}

		// Frame received
		// --------------------------------------------------------------------------
		// | STX | ADDR | L1 | L2 | COM | INF1 | INF2 |	DATA	| STA | CRC1 | CRC2 |
		// --------------------------------------------------------------------------

		// Trama completa:
		//  Checkear que el byte de comando es 0xB0:
		if ( buf[4]!=0xB0 )	return false;

		// GET FRAME INFO
		int  info	 = buf[5] | (buf[6] << 8);	// Little Endian
		int  n_points = info & 0x01FF;
		is_mm_mode = 0 != ((info & 0xC000) >> 14);	// 0x00: cm 0x01: mm

		out_ranges_meters.resize(n_points);
		cout << "resize point : " << n_points << endl;

		// Copiar rangos:
		short mask = is_mm_mode ? 0x7FFF : 0x1FFF;
		float meters_scale = is_mm_mode ? 0.001f : 0.01f;

		for (int i=0;i<n_points;i++)
			out_ranges_meters[i] = ( (buf[7+i*2] | (buf[8+i*2] << 8)) & mask ) * meters_scale;

		// Status
		LMS_status = buf[729];

		// CRC:
		uint16_t CRC = computeCRC(buf,lenghtField-2);
		uint16_t CRC_packet = buf[lenghtField-2] | ( buf[lenghtField-1] << 8);
		if (CRC_packet!=CRC)
		{
			cerr << format("[CSickLaserUSB::waitContinuousSampleFrame] bad CRC len=%u nptns=%u: %i != %i", unsigned(lenghtField),unsigned(n_points), CRC_packet, CRC) << endl;
			return false; // Bad CRC
		}

		// Get USB board timestamp:
		out_board_timestamp =
			(uint32_t(buf[nFrameBytes-5]) << 24) |
			(uint32_t(buf[nFrameBytes-4]) << 16) |
			(uint32_t(buf[nFrameBytes-3]) << 8) |
			uint32_t(buf[nFrameBytes-2]);

		// All OK?
		return (buf[nFrameBytes-1]==0x55);
	}

	
/*-------------------------------------------------------------
  ComputeCRC
  -------------------------------------------------------------*/
	uint16_t SickLMS200::computeCRC(unsigned char *data, unsigned long len)
	{
		uint16_t uCrc16;
		uint8_t  abData[2];

		uCrc16 = 0;
		abData[0] = 0;

		while(len-- )
		{
			abData[1] = abData[0];
			abData[0] = *data++;

			if( uCrc16 & 0x8000 )
			{
				uCrc16 = (uCrc16 & 0x7fff) << 1;
				uCrc16 ^= CRC16_GEN_POL;
			}
			else
			{
				uCrc16 <<= 1;
			}
			uCrc16 ^= (abData[0] | (abData[1]<<8));
		}
		return (uCrc16);
	}

	


static SickLMS200* sick=0;
static void SickLMS200_trap(int sig) {
    if (sick!=0) {
        sick->reset();
    }
};

bool registerSickLMS200SignalHandler() {
  return !(signal(SIGTERM, SickLMS200_trap) ==SIG_ERR || signal(SIGHUP, SickLMS200_trap) ==SIG_ERR ||
           signal(SIGINT, SickLMS200_trap) ==SIG_ERR || signal(SIGQUIT, SickLMS200_trap) ==SIG_ERR ||
           signal(SIGABRT, SickLMS200_trap) ==SIG_ERR);
};


Exception::Exception(const char* _descr):descr(_descr) {}
const char* Exception::getDescription() { return descr; }

BaudRateChangeException::BaudRateChangeException():Exception("Failure to set the baud rate at Sick side") {};

InvalidResolutionException::InvalidResolutionException():Exception("Invalid resolution selected") {}

ResolutionFailureException::ResolutionFailureException():Exception("Failure to set the resolution mode at Sick side") {}

ModeFailureException::ModeFailureException():Exception("Failure to set the measurement mode at Sick side") {}

StartFailureException::StartFailureException():Exception("Failure to start the Sick sensor") {}

StopFailureException::StopFailureException():Exception("Failure to stop the Sick sensor") {}

RegisterException::RegisterException():Exception("Failure to register a signal handler") {}

IOException::IOException(const char* _descr):Exception(_descr) {}

FatalMeasurementException::FatalMeasurementException():Exception("Fatal measurement exception") {}


};

