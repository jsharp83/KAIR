#include <mrpt/core.h>
#include "SickLMS200.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace SickDriver;
using namespace mrpt::hwdrivers;
using namespace std;

#define LASER_MODEL 1  // 0 : sick 1 : Hokuyo
#define SICK 0


SickLMS200 sick("/dev/ttyUSB0", 16,8, 1);

// Forward declaration:
void initLaser();
bool receiveDataFromSensor(CObservation2DRangeScan* pObs, int laser_model);
void laserTest();


int main(int argc, char **argv)
{
  laserTest();
}

void initLaser(){
  cout << "[init Laser start....]" << endl;
  
  if(sick.start()){

    cout << "Sick initialized successfully" << endl;
  }
  else   {
    cout << "Error in initLaser" << endl;
    exit;
  }

  cout << "[init Laser end....]" << endl;
}

bool receiveDataFromSensor(CObservation2DRangeScan* pObs, int laser_model)
{
	bool		thereIsObservation, hardError;

	int count =0;
	while(true)  {

      sick.doProcessSimple(thereIsObservation, *pObs, hardError);

	  if(thereIsObservation && !hardError)
		break;
	  else{
		count++;
	  }

	  if(count>10)	{
		cout << "Receive Data Error... and Exit" << endl;
		//		sick.stop();
		return false;
	  }
	}
	return true;
}

// Test function
void laserTest(){
  initLaser();

#if MRPT_HAS_WXWIDGETS
	CDisplayWindowPlots		win("Sick Laser scaner");
#endif

	CSimplePointsMap receiveMap;
	//	CObservation2DRangeScan* obs = new CObservation2DRangeScan();
	CObservation* obs = new CObservation2DRangeScan();

  while(!mrpt::system::os::kbhit()){

	if(receiveDataFromSensor((CObservation2DRangeScan*)obs, SICK))
	{
	  ((CObservation2DRangeScan*)obs)->sensorPose = CPose3D(0,0,0);

	  receiveMap.clear();
	  receiveMap.insertionOptions.minDistBetweenLaserPoints	= 0;
	  receiveMap.insertObservation( obs );
	    
	}

#if MRPT_HAS_WXWIDGETS

	vector_float xs, ys, zs;
	receiveMap.getAllPoints(xs,ys,zs);

	win.plot(xs,ys,".r3","t1");
	win.axis_equal();

	//	win.axis_fit();
#endif
	
  }

  delete obs;
  sick.stop();

}

