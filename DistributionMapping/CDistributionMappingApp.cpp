/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Javier Gonzalez Monroy  <jgmonroy@isa.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Module that implements different algorithms for spatial data prediction, generating Distribution maps.
  *  This module is designed to work with different types of data: gas, temperature, wifi, etc.
  *  This module can work in two modes:
  *		Online:  Where the input data values come from the available OpenMORA variables (published by other modules, e-nose, temperature, wifi, etc)
  *		Offline: Where the input data comes through a rawlog file (see module ScopeLogger)
  *  The maps generated will be published as OpenMORA variables to be displayed with the main GUI module in real time, or to be used by other modules.  
*/

#include "CDistributionMappingApp.h"
#include <mrpt/opengl.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;

CDistributionMappingApp::CDistributionMappingApp()
{
}

CDistributionMappingApp::~CDistributionMappingApp()
{	
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CDistributionMappingApp::OnStartUp()
{  // Can load params from m_MissionReader or m_ini
	try
	{
		// ========================================================
		// (1) Load CreationOptions and create GasMap
		// ========================================================
		printf("STARTING ONLINE MAP MODULE\n");
		printf("---------------------------------\n");
		printf("Loading CreationOptions...");
		std::string section = "";
		/** @moos_param mapType The type of map to generate. 
		  * In general the mapType must be a mrpt::slam::CGasConcentrationGridMap2D::TMapRepresentation, 
		  * but additionally it can be "MobileMap" which creates two maps (a my_map covering all the experimental area of type KalmanApproximate and a fixed size mobile_map moving over the my_map)
		*/		
		mapType = m_ini.read_enum<CRandomFieldGridMap2D::TMapRepresentation>(section,"mapType",CRandomFieldGridMap2D::mrKalmanApproximate,true);	
		/** @moos_param use_metricmap_information Indicates for GMRF models, whether a metric map is considered when building the MRF.
		*/
		
		
		double x0,x1,y0,y1;
		// Get initial Main Map dimensions [m] from ini file:
		// ------------------------------------------------------------
		//! @moos_param min_x The initial my_map dimensions [m]
		x0 = m_ini.read_double(section,"min_x",0,true);
		//! @moos_param min_y The initial my_map dimensions [m]
		y0 = m_ini.read_double(section,"min_y",0,true);
		//! @moos_param max_x The initial my_map dimensions [m]
		x1 = m_ini.read_double(section,"max_x",0,true);
		//! @moos_param max_y The initial my_map dimensions [m]
		y1 = m_ini.read_double(section,"max_y",0,true);
		
		
		//! @moos_param resolution  The gascell side length (meters) (must be small for mapType = mrKernelDM)
		const double res = m_ini.read_double(section,"resolution",0,true);
		printf("loaded ok\n");

		//Chek if obstacles
		const bool obstacles = m_ini.read_bool(section,"GMRF_use_occupancy_information",false,false);
        if( (mapType == mrpt::slam::CRandomFieldGridMap2D::mrGMRF_SD) && (obstacles) )
		{            
			//reset map dimensions so "resize" can match to ocupancy-gridmap
			x0=y0=0;
			x1=y1=1;
		}

		//Create GasMap
		my_map = new mrpt::slam::CGasConcentrationGridMap2D(mapType,x0,x1,y0,y1,res);
		cout << "GasGridMap created Successfully" << endl;

		//! @moos_param disableSaveAs3DObject Indicates (true/false) if the map can be saved as a 3D Object
		my_map->m_disableSaveAs3DObject = m_ini.read_bool(section,"disableSaveAs3DObject","false",false);
		
		
		// ========================================================
		// (2) Load Insertion Options
		// ========================================================
		cout << "LOading InsertionOptions" << endl;
		my_map->insertionOptions.loadFromConfigFile(m_ini,"");
		cout << "Options loaded Ok" << endl;
		my_map->clear();	//Necessary in most map types

		// ===============================
		// (3) Dump to screen
		// ===============================
		printf("\nMAP PARAMETERS\n");
		printf("----------------------------------\n");		
		printf("    MapType= %d\n",my_map->getMapType());
		printf("    Dimensions: x=[%.1f %.1f] y=[%.1f %.1f]\n",x0,x1,y0,y1);	
		printf("    Resolution = %.2f\n",res);
		cout << "    ---InsertionOptions parameters---" << endl;
		cout << "    gasSensorLabel: " << my_map->insertionOptions.gasSensorLabel << endl;
		cout << "    enoseID: " << my_map->insertionOptions.enose_id << endl;
		cout << "    gasSensorType: " << format("%x",my_map->insertionOptions.gasSensorType) << endl;
		cout << "    windSensorLabel: " << my_map->insertionOptions.windSensorLabel << endl;
		cout << "    useWindInformation: " << my_map->insertionOptions.useWindInformation << endl;
		cout << "    advection_freq: " << my_map->insertionOptions.advectionFreq << endl;


		// ========================================================
		// (4) Load Advection params
		// ========================================================
		//! @moos_param useWindInformation Indicates (true/false) if wind information is available for advection simulation
		//! @moos_param advectionFreq The frequency (Hz) advection is simulated
		//! @moos_param default_wind_direction The default value of the wind direction (rad) on each Grid cell
		//! @moos_param default_wind_speed The default value of the wind speed (m/s) on each Grid cell
		//! @moos_param std_windNoise_phi The std to consider on wind information measurements (angle)
		//! @moos_param std_windNoise_mod The std to consider on wind information measurements (module)
		
		
		//! @moos_param STD_increae_value The increased value (normalized units) to apply to the kf_std of each cell from the my_map
		STD_increase_value				 = m_ini.read_double("","STD_increase_value",0,true);

		// ========================================================
		// (4) Load module params
		// ========================================================
		//! @moos_param SAVE_MAP_SCENE Indicates if the maps should be saved as 3Dscenes
		SAVE_MAP_SCENE					 = m_ini.read_bool("","SAVE_MAP_SCENE", false,  /*Force existence:*/ true);
		//! @moos_param LOG_OUT_ONLINEMAPS The directory where the log_files and scenes will be generated
		OUT_DIR							 = m_ini.read_string("","logOutput_dir","LOG_OUT_ONLINEMAPS",  /*Force existence:*/ true);
		//! @moos_param LOG_FREQUENCY  The frequency in Ticks to save to log_file. Refer to the general AppTick(Hz) of each OpenMORA module.
		LOG_FREQUENCY					 = m_ini.read_float("","LOG_FREQUENCY",5,  /*Force existence:*/ true);
		//! @moos_param insert_from_rawlog Indicates if data (observations) comes from a rawlog file or online
		insert_from_rawlog				 = m_ini.read_bool("","insert_from_rawlog", false,  /*Force existence:*/ true);
		//! @moos_param rawlog_path The path to the rawlog_file
		rawlog_path						 = m_ini.read_string("","rawlog_path","",  /*Force existence:*/ true);
		

		// Prepare output directory:
		// --------------------------------
		const char* OUT_DIR_C = OUT_DIR.c_str();
		deleteFilesInDirectory(OUT_DIR_C);
		createDirectory(OUT_DIR_C);

		printf("\n\n--------SAVING PARAMETERS--------\n----------------------------------\n");
		printf("  SAVE_MAP_SCENE:\t\t\t%c\n", SAVE_MAP_SCENE ? 'Y':'N');	
		printf("  OUT_DIR:\t\t\t\t%s\n", OUT_DIR_C);
		printf("  LOG_FREQUENCY:\t\t\t%.2f\n", LOG_FREQUENCY);
		
		cout << "-----------------------------------------------------" << endl;
		cout << "-----------------------------------------------------" << endl;

		// If load from rawlog, check that rawlog exists
		if (insert_from_rawlog)
		{
			if (mrpt::system::fileExists(rawlog_path))
			{
				rawlog_input.open( rawlog_path );
				if ( !rawlog_input.fileOpenCorrectly() )
					cout << "Error opening rawlog file" << endl;
				indexMonitoredSensor = -1;
			}
			else
			{
				cout << "Rawlog-file cannot be found\n" << endl;
				return false;
			}
		}

		mrpt::system::sleep(5000);
	
		//update time of last updated.
		timeLastSimulated = mrpt::system::now();		
		timeLastSaved = mrpt::system::now();
		step = 0;
		end_of_rawlog = false;
		// Refresh map in GUI:
		SendGasMap3DToGUI();	
		return DoRegistrations();
	}
	catch (std::exception e) 
	{
		MOOSTrace("Exception Loading parameteres from .moos file:  %s\n",e.what());
		return false;
	}
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CDistributionMappingApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	//const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool CDistributionMappingApp::Iterate()
{
	bool new_gas_measurement = false;
	bool new_wind_measurement = false;
	bool new_location = false;

	mrpt::slam::CObservationGasSensorsPtr gasObs;
	mrpt::poses::CPose2D robotPose2D;	
	mrpt::slam::CObservationWindSensorPtr windObs;	

	if (!insert_from_rawlog)
	{
	 
		// ============================================
		// 1) READ Gas sensors (MCEnose, Full_MCEnose, GDM, ENOSE_SIMUL)
		// ============================================
		try
		{	
			CMOOSVariable * pVar = GetMOOSVar( my_map->insertionOptions.gasSensorLabel );
			if(pVar && pVar->IsFresh())
				{
					CSerializablePtr obj;
					StringToObject(pVar->GetStringVal(),obj);
					pVar->SetFresh(false);				
				
					if ( IS_CLASS(obj, CObservationGasSensors ))
					{
						gasObs = CObservationGasSensorsPtr(obj);
						new_gas_measurement = true;
						//cout << " - New gas Obs" << endl;
					}
					else
						cout << "Obs is not CObservationGasSensors type" << endl;
				}
			else
				{
					//MOOSTrace(mrpt::format("%s: Warning: 'GAS READINGS' not available -> map not updated.\n", GetAppName().c_str()  ));
				}

		}catch(...)
		{
			cout << "Exception obtaining GAS obs: "  << endl;
		}

		// ============================================
		// 2) READ Localization
		// ============================================
		// Read current robot position:
		try
		{
			CMOOSVariable * pVarLoc = GetMOOSVar( "LOCALIZATION" );
			if(pVarLoc && pVarLoc->IsFresh())
			{
				robotPose2D.fromString( pVarLoc->GetStringVal() );
				robotPose3D = CPose3D(robotPose2D);
				new_location = true;			
			}
			else
			{
				MOOSTrace(mrpt::format("%s: Warning: 'LOCALIZATION' not available -> map not updated.\n", GetAppName().c_str()  ));
			}

		}catch(...)
		{
			cout << "Exception obtaining POSE obs: "  << endl;
		}

		// ============================================
		// 3) READ Wind information
		// ============================================	
		//try{
		//	if (my_map->insertionOptions.useWindInformation)
		//	{	
		//		CMOOSVariable * pVar = GetMOOSVar( my_map->insertionOptions.windSensorLabel );
		//		if(pVar && pVar->IsFresh())
		//		{
		//			CSerializablePtr obj;
		//			StringToObject(pVar->GetStringVal(),obj);			
		//			
		//			if ( IS_CLASS(obj, CObservationWindSensor ))
		//			{
		//				windObs = CObservationWindSensorPtr(obj);
		//				new_wind_measurement = true;
		//				cout << " - New Wind obs" << endl;
		//			}
		//			else
		//				cout << "Obs is not CObservationWindSensor type" << endl;
		//		}else
		//		{
		//			//simulate wind measurement	
		//			mrpt::slam::CObservationWindSensor *w = new mrpt::slam::CObservationWindSensor();
		//			w->direction = 3.1416;
		//			w->speed = 2.0;
		//			w->sensorLabel = "windSensor";
		//			w->timestamp = now();
		//			windObs = CObservationWindSensorPtr(w);
		//			new_wind_measurement = true;
		//			//MOOSTrace(mrpt::format("%s: Warning: 'WIND READING' not available.\n", GetAppName().c_str()  ));
		//		}
		//	}
		//}catch(...)
		//{
		//	cout << "Exception obtaining WIND obs: "  << endl;
		//}
	}
	else
	{
		//Load data from Rawlog-file as pairs<gas_observation - Pose>
		if (!end_of_rawlog)
		{
			while (!new_gas_measurement || !new_location)
			{
				try
				{
					CObservationPtr o;
					rawlog_input >> o;

					if ( o ) //ASSERT_(o);
					{
						if (IS_CLASS(o,CObservationGasSensors))
						{
							CObservationGasSensorsPtr obs = CObservationGasSensorsPtr( o );
				   
				   			if (obs->sensorLabel == my_map->insertionOptions.gasSensorLabel)
							{
								gasObs = obs;
								new_gas_measurement = true;
							}
						}
						else if (IS_CLASS(o,CObservationOdometry))					
						{
							CObservationOdometryPtr obs = CObservationOdometryPtr( o );
				   
				   			if (obs->sensorLabel == "LOCALIZATION")
							{	
								robotPose3D = CPose3D(obs->odometry);
								new_location = true;
								//Publish current robot localization, to be displayed in the GUI
								//! @moos_publish	LOCALIZATION   The robot estimated pose in format "[x y phi]"
								string sPose;
								obs->odometry.asString(sPose);
								m_Comms.Notify("LOCALIZATION", sPose );
							}
						}
					}
				}
				catch( exception &e )
				{
					cout << "Exception: " << e.what() << endl;
					rawlog_input.close();
					cout << "END OF RAWLOG" << endl;
					end_of_rawlog = true;
					break;
				}
			}//end while
		}//end ifRawlog

	}//end-if insert_from_rawlog




	// ============================================
	// Update Online Maps:
	// ============================================
#define DO_PROFILE

#ifdef DO_PROFILE
	static mrpt::utils::CTimeLogger timelogger;
	timelogger.enter("insObs");
#endif

	try{
		if (new_gas_measurement && new_location)
		{
			my_map->insertObservation(gasObs.pointer(),&robotPose3D);
			
#ifdef DO_PROFILE	
	cout << "Inserting Gas Obs=" << gasObs->m_readings[0].readingsVoltage[0] << ", in ["<< my_map->getSizeX() << "x" << my_map->getSizeY() <<"]cells map took: " << timelogger.leave("insObs") << "s" << endl;
	cout << timelogger.getStatsAsText();
#endif
			
		}
		
		if (new_wind_measurement && new_location)
		{
			//update wind maps			
			// To Be implemented
		}
	}catch(...){
		cout << "Exception while inserting obs: "  << endl;
	}
	

	// ============================================
	// 4) Simulate Advection
	// ============================================
	try{		
		if( my_map->insertionOptions.useWindInformation )
		{
			double At = mrpt::system::timeDifference(timeLastSimulated, mrpt::system::now());
		
			if( At >= 1/my_map->insertionOptions.advectionFreq )
			{
				cout << endl << "###__________Simulating Advection_________" << endl;
				timeLastSimulated = mrpt::system::now();
				bool adv = my_map->simulateAdvection(STD_increase_value);
				cout << "Done_###" << endl;

				if (!adv)
					cout << "Advection not simulated due to Error." << endl;
			}
			else
			{
				//cout << "still At = " << At << endl;
			}
		}
	}catch(...){
		cout << "Exception while Simulating Advection: "  << endl;
	}

	// ============================================
	// 4) Refresh map in GUI:
	// ============================================
	try{
		SendGasMap3DToGUI(); // This updates "m_last_3D_GUI_time"
		double At = mrpt::system::timeDifference(timeLastSaved, mrpt::system::now());
		
		if ( (SAVE_MAP_SCENE) && (At >= 1/LOG_FREQUENCY) )
		{
			SaveGasMap();
			timeLastSaved = mrpt::system::now();
		}
	}catch(...){
		cout << "Exception while Saving 3D representation: "  << endl;
	}
	return true;
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CDistributionMappingApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CDistributionMappingApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
	//! @moos_subscribe LOCALIZATION
	AddMOOSVariable( "LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0.1 );
	//! @moos_subscribe MCEnose
	AddMOOSVariable( "MCEnose", "MCEnose", "MCEnose", 0.1 );
	//! @moos_subscribe Full_MCEnose
	AddMOOSVariable( "Full_MCEnose", "Full_MCEnose", "Full_MCEnose", 0.1 );
	//! @moos_subscribe GDM
	AddMOOSVariable( "GDM", "GDM", "GDM", 0.1 );
	//! @moos_subscribe ENOSE_SIMUL
	AddMOOSVariable( "ENOSE_SIMUL", "ENOSE_SIMUL", "ENOSE_SIMUL", 0);
	//! @moos_subscribe CURRENT_MAP_GAS_SIMUL
	AddMOOSVariable( "CURRENT_MAP_GAS_SIMUL", "CURRENT_MAP_GAS_SIMUL", "CURRENT_MAP_GAS_SIMUL", 0);
	//! @moos_subscribe GT_GAS_SIMUL
	AddMOOSVariable( "GT_GAS_SIMUL", "GT_GAS_SIMUL", "GT_GAS_SIMUL", 0);
	
	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CDistributionMappingApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;

	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
	//	MOOSTrace(format("New msg received: %s \n",i->GetKey()));
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}

	UpdateMOOSVariables(NewMail);
	return true;
}

void CDistributionMappingApp::SendGasMap3DToGUI()
{
	//cout << "updatign GUI" << endl;

	// Render Maps as 3D objects:
	mrpt::opengl::CSetOfObjectsPtr mean_obj = mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfObjectsPtr var_obj  = mrpt::opengl::CSetOfObjects::Create();
	
	//----------------------------------------
	// gasMap (always send this map to GUI)
	//----------------------------------------
	my_map->getAs3DObject(mean_obj,var_obj);
	
	//my_map(mean map)
	const string sTmpGridmap = mrpt::system::getTempFileName();
	m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap);
	CFileGZOutputStream(sTmpGridmap) << mean_obj;
	//! @moos_publish	GAS_MEAN_ONLINE_MAP   A temporary file with a binary serialization of an opengl object (CSetOfObjects) with the current state of the GAS_MEAN_MAP being simulated.
	m_Comms.Notify("GAS_MEAN_ONLINE_MAP", sTmpGridmap );
	
	//my_map(var map)
	const string sTmpGridmap_var = mrpt::system::getTempFileName();
	m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap_var);
	CFileGZOutputStream(sTmpGridmap_var) << var_obj;
	//! @moos_publish	GAS_VAR_ONLINE_MAP   A temporary file with a binary serialization of an opengl object (CSetOfObjects) with the current state of the (VARIANCE) gas map being simulated.
	m_Comms.Notify("GAS_VAR_ONLINE_MAP", sTmpGridmap_var );
	
	
	//----------------------------------------------
	// windMap (only if useWindInformation = true)
	//----------------------------------------------
	if( my_map->insertionOptions.useWindInformation )
	{	
		mean_obj->clear();		
		my_map->getWindAs3DObject(mean_obj);
		
		//windMap
		const string sTmpGridmap = mrpt::system::getTempFileName();
		m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap);
		CFileGZOutputStream(sTmpGridmap) << mean_obj;
		//! @moos_publish	WIND_ONLINE_MAP   A temporary file with a binary serialization of an opengl object (CSetOfObjects) with the current state of the WIND_MEAN_MAP being simulated.
		m_Comms.Notify("WIND_ONLINE_MAP", sTmpGridmap );		
	}
}

void CDistributionMappingApp::SaveGasMap()
{	
	cout << "- Saving Gas Map - " << endl;

	//Save MapRepresentation (my_map): Means + STDs
	std::string path = format("%s/my_map_%i",OUT_DIR.c_str(),step);
	my_map->saveMetricMapRepresentationToFile(path.c_str());

	// The Ground-Truth (Gas simulated)
	CMOOSVariable * GTVar = GetMOOSVar("GT_GAS_SIMUL");
	if(GTVar && GTVar->IsFresh())
	{
		CMatrix GT;
		CFileGZInputStream fil(GTVar->GetStringVal());
		fil >> GT;		
		GT.saveToTextFile(path + std::string("_gt.txt"), MATRIX_FORMAT_FIXED);
	}

	// The robot current pose
	std::string sPose = robotPose3D.asString();
	
	FILE *file= ::fopen((path+std::string("_RobotPose.txt")).c_str(),"wt");
	fprintf(file,"%f %f %f %f %f %f", robotPose3D.x(), robotPose3D.y(), robotPose3D.z(), RAD2DEG(robotPose3D.yaw()), RAD2DEG(robotPose3D.pitch()), RAD2DEG(robotPose3D.roll()) );
	::fclose(file);

	//------------------------------------------------------
	//					3D Scene
	//------------------------------------------------------
	//Create Scene + viewport
	COpenGLScenePtr		scene = COpenGLScene::Create();
    COpenGLViewportPtr view = scene->getViewport("main");
    ASSERT_(view);

    COpenGLViewportPtr view_map = scene->createViewport("mini-map");
    view_map->setBorderSize(2);
    view_map->setViewportPosition(0.01,0.01,0.35,0.35);
    view_map->setTransparent(true);

	// The ground:
	mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
	groundPlane->setColor(0.4,0.4,0.4);
	view->insert( groundPlane );
	view_map->insert( CRenderizablePtr( groundPlane) ); // A copy	

	// The maps:
	opengl::CSetOfObjectsPtr meanObj = opengl::CSetOfObjects::Create();
	opengl::CSetOfObjectsPtr varObj = opengl::CSetOfObjects::Create();
	{		
		my_map->getAs3DObject( meanObj, varObj );
		view->insert(meanObj);
		view_map->insert(varObj);		
	}

	// The Ground-Truth (Gas simulated)
	CMOOSVariable * gtVar = GetMOOSVar("CURRENT_MAP_GAS_SIMUL");
	if(gtVar && gtVar->IsFresh())
	{
		mrpt::opengl::CSetOfObjectsPtr gt_obj = mrpt::opengl::CSetOfObjects::Create();
		CFileGZInputStream fil(gtVar->GetStringVal());
		fil >> gt_obj;		
		//change the "z" position to avoid overlap of map layers.
		/*mrpt::poses::CPose3D pose_gt = gt_obj->getPose();
		pose_gt.z(0.5);
		gt_obj->setPose(pose_gt);*/

		//mrpt::utils::CSerializablePtr obj = fil.ReadObject();
		view->insert(gt_obj);
		
	}
	

	// Draw the robot path:
	{
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotRhodon();
		obj->setPose( robotPose3D );
		view->insert(obj);
	}
	{
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotRhodon();
		obj->setPose( robotPose3D );
		view_map->insert( obj );
	}


	//Set mini-map camera settings
	{
		mrpt::opengl::CCamera &cam = view_map->getCamera();
		cam.setAzimuthDegrees(-90);
		cam.setElevationDegrees(90);
		//cam.setPointingAt(robotPose3D);
		const double centerX = ( my_map->getXMax() - my_map->getXMin() )/2.0;
		const double centerY = ( my_map->getYMax() - my_map->getYMin() )/2.0;

		cam.setPointingAt(centerX,centerY,0.0);
		cam.setZoomDistance(20);
		cam.setOrthogonal();
	}

	// Save as file:
	CFileGZOutputStream	f( format( "%s/buildingmap_%05u.3Dscene",OUT_DIR.c_str(),step ));
	f << *scene;

	step++;
}
