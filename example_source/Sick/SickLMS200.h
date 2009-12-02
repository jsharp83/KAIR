#ifndef SICK_LMS200_H
#define SICK_LMS200_H
/*More information:
The serial link is the bottleneck. Therefore, it makes sense to use the fastest transfer
 mode (38400 bps) once after power up.  On the other hand, it should be reset to the
default of 9600bps after use.  The SIGINT, SIGTERM etc are thus trapped to carry out
the import scanner transfer reset process.
*/
#include <termios.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/utils/types.h>

#define MAXNDATA 802

using namespace mrpt;

namespace SickDriver {
typedef unsigned char uchar;
class Exception {
        const char* descr;
    public:
    Exception(const char* _descr);
    const char* getDescription();
};

class BaudRateChangeException : public Exception {
    public:
    BaudRateChangeException();
};

class InvalidResolutionException:public Exception {
    public:
    InvalidResolutionException();
};

class ResolutionFailureException:public Exception {
    public:
        ResolutionFailureException();
};


class ModeFailureException:public Exception {
    public:
    ModeFailureException();
};

class StartFailureException:public Exception {
    public:
    StartFailureException();
};

class StopFailureException:public Exception {
    public:
        StopFailureException();
};


class RegisterException:public Exception {
    public:
        RegisterException();
};

class IOException:public Exception {
    public:
    IOException(const char* _descr);
};

class FatalMeasurementException:public Exception {
    public:
        FatalMeasurementException();
};


class SickLMS200 {
    const char* port;
    int range_mode;
    int res_mode;
    int unit_mode;
    termios oldtio;
    int fd;
    uchar meas_state;
public:
    typedef enum {
        RANGE_100=32,
        RANGE_180=16,
        RES_1_DEG=8,
        RES_0_5_DEG=4,
        RES_0_25_DEG=2,
        MMMODE=1,
        CMMODE=0 } Constants;

    SickLMS200(
       const char* _port,
       uchar _range_mode=RANGE_100,
       uchar _res_mode  = RES_1_DEG,
       uchar _unit_mode = MMMODE );
    bool start();
    int readMeasurement(uchar* buf,int& datalen);
    bool reset();
    bool stop();
    ~SickLMS200();
    bool checkErrorMeasurement();
    bool checkPlausible();

	void  doProcessSimple(
		bool							&outThereIsObservation,
		mrpt::slam::CObservation2DRangeScan	&outObservation,
		bool							&hardwareError );
	
private:

	bool 	checkControllerIsConnected();
	bool  	waitContinuousSampleFrame( vector_float &ranges, unsigned char &LMS_status, uint32_t &out_board_timestamp, bool &is_mm_mode );
	uint16_t computeCRC(unsigned char *data, unsigned long len);


    bool msgcmp(int len1, const uchar *s1, int len2, const uchar *s2);
    void wtLMSmsg(int fd, int len, const uchar *msg);

    int rdLMSmsg(int fd, int len, const uchar *buf);

    uchar rdLMSbyte(int fd);

    /*return true if the ACK packet is as expected*/
    bool chkAck(int fd, int ackmsglen, const uchar *ackmsg);

    /*set the communication speed and terminal properties*/
    bool initLMS(const char *serialdev, struct termios *oldtio, int& fd);


    /*set both the angular range and resolutions*/
    bool setmode(int fd, int mode);

    /*tell the scanner to enter the continuous measurement mode*/
    bool startLMS(int fd);

    /*stop the continuous measurement mode*/
    bool stopLMS(int fd);

    /*check the status bit of the measurement*/
    void chkstatus(uchar c);

    /*reset terminal and transfer speed of laser scanner before quitting*/
    bool resetLMS(int fd, struct termios *oldtio);

    /*trap a number of signals like SIGINT, ensure the invoke
    of resetLMS() before quitting*/
    static void sig_trap(int sig);
};


bool registerSickLMS200SignalHandler();
};
#endif

