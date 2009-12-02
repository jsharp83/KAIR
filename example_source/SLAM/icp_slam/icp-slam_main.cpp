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

//int ICP_method = (int) icpLevenbergMarquardt;
int ICP_method = (int) icpClassic;

#define LASER_TEST 0
#define BREAK 0
#define DELAY 1
//#define READ_FROM_RAWLOG 0
#define OBS_FROM_FILE 0	// 1: File 0 : Sensor
#define LASER_MODEL 1  // 0 : sick 1 : Hokuyo
#define SICK 0
#define HOKUYO 1

std::string				INI_FILENAME;
std::string				RAWLOG_FILE;
unsigned int			rawlog_offset;
std::string				OUT_DIR_STD;
const char				*OUT_DIR;
int						LOG_FREQUENCY;
bool					SAVE_POSE_LOG;
bool					SAVE_3D_SCENE;
bool					CAMERA_3DSCENE_FOLLOWS_ROBOT;

bool 					SHOW_PROGRESS_3D_REAL_TIME = false;
int						SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
bool					matchAgainstTheGrid;

TSetOfMetricMapInitializers  metricMapsOpts;
float						insertionLinDistance;
float						insertionAngDistance;

CICP::TConfigParams			icpOptions;

SickLMS200 sick("/dev/ttyUSB0", 16,8, 1);
CHokuyoURG		hokuyo;

// Forward declaration:
void initLaser();
bool receiveDataFromSensor(CObservation2DRangeScan* pObs, int laser_model);
void laserTest();


// Forward declaration.
void MapBuilding_ICP();

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		printf(" icp-slam version 0.1 - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		if (showVersion)
			return 0;	// Program end

		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2 || showHelp )
		{
			printf("Usage: %s <config_file.ini>\n\n",argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else	return 0;
		}

		INI_FILENAME = std::string( argv[1] );
		ASSERT_(fileExists(INI_FILENAME));

        //        printf("HERE : %s", INI_FILENAME);
        //cout << "HERE : " << INI_FILENAME << endl;

        CConfigFile		iniFile( INI_FILENAME );

		// ------------------------------------------
		//			Load config from file:
		// ------------------------------------------
		RAWLOG_FILE			 = iniFile.read_string("MappingApplication","rawlog_file","",  /*Force existence:*/ true);
		rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0,  /*Force existence:*/ true);
		OUT_DIR_STD			 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  /*Force existence:*/ true);
		LOG_FREQUENCY		 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  /*Force existence:*/ true);
		SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false,  /*Force existence:*/ true);
		SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false,  /*Force existence:*/ true);
		CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*Force existence:*/ true);

		MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME, bool,  iniFile, "MappingApplication");
		MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");

		OUT_DIR = OUT_DIR_STD.c_str();

		MRPT_LOAD_CONFIG_VAR( insertionLinDistance, 	float,		iniFile, "MappingApplication");
		MRPT_LOAD_CONFIG_VAR_DEGREES( insertionAngDistance,		iniFile, "MappingApplication");

		metricMapsOpts.loadFromConfigFile(iniFile, "MappingApplication");
		icpOptions.loadFromConfigFile(iniFile, "ICP");
		matchAgainstTheGrid = iniFile.read_bool("MappingApplication","matchAgainstTheGrid", true );

		// Print params:
		printf("Running with the following parameters:\n");
		printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
		printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
		printf(" matchAgainstTheGrid:\t\t\t%c\n", matchAgainstTheGrid ? 'Y':'N');
		printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);
		printf("  SAVE_3D_SCENE:\t\t\t%c\n", SAVE_3D_SCENE ? 'Y':'N');
		printf("  SAVE_POSE_LOG:\t\t\t%c\n", SAVE_POSE_LOG ? 'Y':'N');
		printf("  CAMERA_3DSCENE_FOLLOWS_ROBOT:\t%c\n",CAMERA_3DSCENE_FOLLOWS_ROBOT ? 'Y':'N');

		printf("\n");

		metricMapsOpts.dumpToConsole();
		icpOptions.dumpToConsole();

		// Checks:
		ASSERT_(RAWLOG_FILE.size()>0);
		ASSERT_(fileExists(RAWLOG_FILE));

		// Run:
		if(LASER_TEST==0)
		  MapBuilding_ICP();
		else if(LASER_TEST==1)
		  laserTest();

		//pause();
		return 0;
	} catch (std::exception &e)
	{
		setConsoleColor(CONCOL_RED,true);
		std::cerr << "Program finished for an exception!!" << std::endl;
		setConsoleColor(CONCOL_NORMAL,true);

		std::cerr << e.what() << std::endl;

		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED,true);
		std::cerr << "Program finished for an untyped exception!!" << std::endl;
		setConsoleColor(CONCOL_NORMAL,true);

		mrpt::system::pause();
		return -1;
	}
}



// ------------------------------------------------------
//				MapBuilding_ICP
// ------------------------------------------------------
void MapBuilding_ICP()
{
	MRPT_TRY_START
	  

	CTicTac								tictac,tictacGlobal,tictac_JH;
	int									step = 0;
	std::string							str;
	CSensFrameProbSequence				finalMap;
	float								t_exec;
	COccupancyGridMap2D::TEntropyInfo	entropy;

	size_t						rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE.c_str() );

	CFileGZOutputStream sensor_data;

	// ---------------------------------
	//		Constructor
	// ---------------------------------
	CMetricMapBuilderICP mapBuilder(
		&metricMapsOpts,
		insertionLinDistance,
		insertionAngDistance,
		&icpOptions
		);

	mapBuilder.ICP_options.matchAgainstTheGrid = matchAgainstTheGrid;

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.options.verbose					= true;
	mapBuilder.options.enableMapUpdating		= true;
    mapBuilder.options.debugForceInsertion		= false;
	mapBuilder.options.insertImagesAlways		= false;

	// Prepare output directory:
	// --------------------------------
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Open log files:
	// ----------------------------------
	CFileOutputStream  f_log(format("%s/log_times.txt",OUT_DIR));
	CFileOutputStream  f_path(format("%s/log_estimated_path.txt",OUT_DIR));
	CFileOutputStream  f_pathOdo(format("%s/log_odometry_path.txt",OUT_DIR));


	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3DPtr( new CDisplayWindow3D("ICP-SLAM @ MRPT C++ Library (C) 2004-2008", 600, 500) );
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif

	if(OBS_FROM_FILE == 0){
	  sensor_data.open("sensor_data.rawlog");
	  printf("Receive From Sensor\n");
	  initLase();
	  printf("OK\n");
	}
	

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations, temp_obs;
	CSensoryFramePtr obs_set;
	CPose2D					odoPose(0,0,0);

	CSimplePointsMap	oldMap, newMap;
	CICP					ICP;

	vector_float		accum_x, accum_y, accum_z;
	
	// ICP Setting
	ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

	ICP.options.maxIterations			= 40;
	ICP.options.thresholdAng =			0.15;
	ICP.options.thresholdDist			= 0.75f;
	ICP.options.ALFA					= 0.30f;
	ICP.options.smallestThresholdDist	= 0.10f;
	ICP.options.doRANSAC = false;
	ICP.options.dumpToConsole();
	//
	CObservationPtr obsSick = CObservationPtr(new CObservation2DRangeScan());
	CObservationPtr obsHokuyo = CObservationPtr(new CObservation2DRangeScan());
	CSimplePointsMap hokuyoMap;
	
	bool isFirstTime = true;
	bool loop = true;
    /*
		  cout << index << "frame, Input the any key to continue" << endl;
		  getc(stdin);
		  fflush(stdin);
    */

	tictacGlobal.Tic();
	while(loop)
	{
      /*
		if(BREAK){
		  cout << index << "frame, Input the any key to continue" << endl;
		  getc(stdin);
		  fflush(stdin);
		}else{

      */
		  if(os::kbhit())
			loop = true;
          //		}

		if(DELAY) {
		  sleep(15);
		}
	  

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------

		if(OBS_FROM_FILE == 1) {
		  if (! CRawlog::readActionObservationPair( rawlogFile, action, temp_obs, rawlogEntry) )
			break; // file EOF

		  obsSick = temp_obs->getObservationByIndex(0);
		  obsHokuyo = temp_obs->getObservationByIndex(1);
		  
		  observations = CSensoryFramePtr(new CSensoryFrame());		  
		  observations->insert((CObservationPtr)obsSick);

		  hokuyoMap.clear();
		  hokuyoMap.insertObservation(obsHokuyo.pointer());
		  
		}else{
		  rawlogEntry = rawlogEntry+2;

		  tictac.Tic();

		  obsSick = CObservationPtr(new CObservation2DRangeScan());
		  obsHokuyo = CObservationPtr(new CObservation2DRangeScan());

		  if(!receiveDataFromSensor((CObservation2DRangeScan*)obsSick.pointer(),SICK)){
			cout << " Error in receive sensor data" << endl;
			return;
		  }

		  if(!receiveDataFromSensor((CObservation2DRangeScan*)obsHokuyo.pointer(),HOKUYO)){
			cout << " Error in receive sensor data" << endl;

			return;
		  }
		  
		  cout << "Time to receive data : " << tictac.Tac()*1000.0f << endl;

		  obsSick->timestamp = mrpt::system::now();
		  obsSick->setSensorPose(CPose3D(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)));
		  ((CObservation2DRangeScan*)obsSick.pointer())->rightToLeft = true;
		  ((CObservation2DRangeScan*)obsSick.pointer())->stdError = 0.003f;

		  obsHokuyo->timestamp = mrpt::system::now();
		  obsHokuyo->setSensorPose(CPose3D(0,0,0.4,DEG2RAD(0),DEG2RAD(-90),DEG2RAD(0)));
		  ((CObservation2DRangeScan*)obsHokuyo.pointer())->rightToLeft = true;
		  ((CObservation2DRangeScan*)obsHokuyo.pointer())->stdError = 0.003f;
		  
		  cout << "rawlogEntry : " << rawlogEntry << endl;

		  CActionRobotMovement2D myAction;

		  newMap.clear();
		  obsSick.pointer()->insertObservationInto(&newMap);

		  if(!isFirstTime){

			static float					runningTime;
			static CICP::TReturnInfo		info;
			static CPose2D initial(0,0,0);

			CPosePDFPtr ICPPdf = ICP.AlignPDF(&oldMap, &newMap, initial, &runningTime, (void*)&info);
			CPose2D		estMean;
			CMatrixDouble33		estCov;
			ICPPdf->getCovarianceAndMean(estCov, estMean);
			printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ", runningTime*1000, info.nIterations, runningTime*1000.0f/info.nIterations, info.goodness*100 );
			cout << "ICP Odometry : " << ICPPdf->getMeanVal() << endl;

			myAction.estimationMethod = CActionRobotMovement2D::emScan2DMatching;
			myAction.poseChange = CPosePDFPtr( new CPosePDFGaussian(estMean, estCov));
		  }else{
			isFirstTime = false;
		  }

		  oldMap.clear();
		  oldMap.copyFrom(newMap);

		  observations = CSensoryFramePtr(new CSensoryFrame());
		  action = CActionCollectionPtr(new CActionCollection());		  

		  observations->insert((CObservationPtr)obsSick);

		  obs_set = CSensoryFramePtr(new CSensoryFrame());

		  obs_set->insert((CObservationPtr)obsSick);
		  obs_set->insert((CObservationPtr)obsHokuyo);
		  
		  action->insert(myAction);

		  sensor_data << action << obs_set;

		  hokuyoMap.clear();
		  hokuyoMap.insertObservation(obsHokuyo.pointer());

		}


		if (rawlogEntry>=rawlog_offset)
		{
			// Update odometry:
			{
				CActionRobotMovement2DPtr act= action->getBestMovementEstimation();
				if (act)
					odoPose = odoPose + act->poseChange->getMeanVal();
			}

			// Execute:
			// ----------------------------------------
			tictac.Tic();
				mapBuilder.processActionObservation( *action, *observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );


			// Info log:
			// -----------
			f_log.printf("%f %i\n",1000.0f*t_exec,mapBuilder.getCurrentlyBuiltMapSize() );

			const CMultiMetricMap* mostLikMap =  mapBuilder.getCurrentlyBuiltMetricMap();

			if (0==(step % LOG_FREQUENCY))
			{
				// Pose log:
				// -------------
				if (SAVE_POSE_LOG)
				{
					printf("Saving pose log information...");
					mapBuilder.getCurrentPoseEstimation()->saveToTextFile( format("%s/mapbuild_posepdf_%03u.txt",OUT_DIR,step) );
					printf("Ok\n");
				}
			}


			// Save a 3D scene view of the mapping process:
			if (0==(step % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3D.present()))
			{
                CPose3D robotPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

				COpenGLScenePtr		scene = COpenGLScene::Create();

                COpenGLViewportPtr view=scene->getViewport("main");
                ASSERT_(view);

                COpenGLViewportPtr view_map = scene->createViewport("mini-map");
                view_map->setBorderSize(2);
                view_map->setViewportPosition(0.01,0.01,0.35,0.35);
                view_map->setTransparent(false);

				{
					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-90);
					cam.setElevationDegrees(90);
					cam.setPointingAt(robotPose);
					cam.setZoomDistance(20);
					cam.setOrthogonal();
				}

				// The ground:
				mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
				groundPlane->setColor(0.4,0.4,0.4);
				view->insert( groundPlane );
				view_map->insert( CRenderizablePtr( groundPlane) ); // A copy

				// The camera pointing to the current robot pose:
				if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
				{
				    scene->enableFollowCamera(true);

					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-45);
					cam.setElevationDegrees(45);
					cam.setPointingAt(robotPose);
				}

				// The maps:
				{
					opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
					mostLikMap->getAs3DObject( obj );
					view->insert(obj);

					// Only the point map:
					opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
					if (mostLikMap->m_pointsMaps.size())
					{
                        mostLikMap->m_pointsMaps[0]->getAs3DObject(ptsMap);
                        view_map->insert( ptsMap );
					}
				}

				// Draw the robot path:
				CPose3DPDFPtr posePDF =  mapBuilder.getCurrentPoseEstimation();
				CPose3D  curRobotPose;
				posePDF->getMean(curRobotPose);
				{
					opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
					obj->setPose( curRobotPose );
					view->insert(obj);
				}
				{
					opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
					obj->setPose( curRobotPose );
					view_map->insert( obj );
				}


				// Draw Hokuyo total data
				{
				  CRenderizablePtr hokuyoRender_t = scene->getByName("hokuyo_total");

				  if(!hokuyoRender_t){
					hokuyoRender_t = CPointCloud::Create();
					hokuyoRender_t->setName("hokuyo_total");
					hokuyoRender_t->setColor(0,0,1);
					hokuyoRender_t->setPose( CPose3D(0,0,0) );
					getAs<CPointCloud>(hokuyoRender_t)->setPointSize(3);
					scene->insert( hokuyoRender_t);					
				  }

				  for(size_t i =0 ; i < accum_x.size(); i++){
					getAs<CPointCloud>(hokuyoRender_t)->insertPoint(accum_x[i], accum_y[i], accum_z[i]);					
				  }
				  cout << "accum_x size : " << accum_x.size() << endl;


				}
				
				// Draw Hokuyo Current data plate
				{
				  CRenderizablePtr hokuyoRender = scene->getByName("hokuyo_cur");
				  hokuyoRender = CPointCloud::Create();
				  hokuyoRender->setName("hokuyo_cur");
				  hokuyoRender->setColor(0,1,0);
				  hokuyoRender->setPose( curRobotPose );
				  getAs<CPointCloud>(hokuyoRender)->setPointSize(0.1);
				  getAs<CPointCloud>(hokuyoRender)->loadFromPointsMap(&hokuyoMap);				  
				  scene->insert( hokuyoRender);

				  vector_float cur_x = getAs<CPointCloud>(hokuyoRender)->getArrayX();
				  vector_float cur_y = getAs<CPointCloud>(hokuyoRender)->getArrayY();
				  vector_float cur_z = getAs<CPointCloud>(hokuyoRender)->getArrayZ();


				  //				  cout << "current pose : " << curRobotPose << endl;				  
				  for(size_t i =0 ; i < cur_x.size(); i++){
					/*
					float rotate_x = cur_x[i]+ curRobotPose.y()*sin(curRobotPose.yaw()*3.14/180.0);
					float rotate_y = cur_y[i]+ curRobotPose.y()*cos(curRobotPose.yaw()*3.14/180.0);
					*/
					float rotate_x = curRobotPose.x() + cur_y[i]*sin(-curRobotPose.yaw());
					float rotate_y = curRobotPose.y() + cur_y[i]*cos(-curRobotPose.yaw());
					//					printf("cur_x, cur_y, cur_z : %f %f %f \n", rotate_x,rotate_y, cur_z[i]);

					accum_x.push_back(rotate_x);
					accum_y.push_back(rotate_y);
					accum_z.push_back(cur_z[i]);
				  }
				}

				// Save as file:
				if (0==(step % LOG_FREQUENCY) && SAVE_3D_SCENE)
				{
					CFileGZOutputStream	f( format( "%s/buildingmap_%05u.3Dscene",OUT_DIR,step ));
					f << *scene;
				}

				// Show 3D?
				if (win3D)
				{
					opengl::COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
					ptrScene = scene;

					win3D->unlockAccess3DScene();

					// Move camera:
					win3D->setCameraPointingToPoint( curRobotPose.x(),curRobotPose.y(),curRobotPose.z() );

					// Update:
					win3D->forceRepaint();

					sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
				}
			}


			// Save the memory usage:
			// ------------------------------------------------------------------
			{
				printf("Saving memory usage...");
				unsigned long	memUsage = getMemoryUsage();
				FILE		*f=os::fopen( format("%s/log_MemoryUsage.txt",OUT_DIR).c_str() ,"at");
				if (f)
				{
					os::fprintf(f,"%u\t%lu\n",step,memUsage);
					os::fclose(f);
				}
				printf("Ok! (%.04fMb)\n", ((float)memUsage)/(1024*1024) );
			}

			// Save the robot estimated pose for each step:
			f_path.printf("%i %f %f %f\n",
				step,
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().x(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().y(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().yaw() );


			f_pathOdo.printf("%i %f %f %f\n",step,odoPose.x(),odoPose.y(),odoPose.phi());

		} // end of if "rawlog_offset"...

		step++;
		printf("\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",step, (unsigned)rawlogEntry);

		// Free memory:
		action.clear_unique();
		observations.clear_unique();
	};

	printf("\n---------------- END!! (total time: %.03f sec) ----------------\n",tictacGlobal.Tac());

	//	hokuyo.turnOff();	
	sick.stop();

	

	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);

	str = format("%s/_finalmap_.simplemap",OUT_DIR);
	printf("Dumping final map in binary format to: %s\n", str.c_str() );
	mapBuilder.saveCurrentMapToFile(str);

	CMultiMetricMap  *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
	str = format("%s/_finalmaps_.txt",OUT_DIR);
	printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
	finalPointsMap->saveMetricMapRepresentationToFile( str );

	if (win3D)
		win3D->waitForKey();

	MRPT_TRY_END
}


void initLaser(){
  cout << "[init Laser start....]" << endl;
  //  if(LASER_MODEL == SICK){
	if(sick.start()){

	  cout << "Sick initialized successfully" << endl;
	}
	else   {
	  cout << "Error in initLaser" << endl;
	  exit;
	}
	//  }else if(LASER_MODEL == HOKUYO){
	hokuyo.m_com_port = "ttyACM0";
	hokuyo.turnOn();
	//  }

  cout << "[init Laser end....]" << endl;
}

bool receiveDataFromSensor(CObservation2DRangeScan* pObs, int laser_model)
{
	bool		thereIsObservation, hardError;

	int count =0;
	while(true)  {

	  if(laser_model == SICK){
		sick.doProcessSimple(thereIsObservation, *pObs, hardError);
	  }else if(laser_model == HOKUYO){
		hokuyo.doProcessSimple(thereIsObservation, *pObs, hardError);
	  }
	  

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

	if(BREAK){
	  cout << index << "frame, Input the any key to continue" << endl;
	  //cin >>  temp;
	  getc(stdin);
	  fflush(stdin);
	}

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

