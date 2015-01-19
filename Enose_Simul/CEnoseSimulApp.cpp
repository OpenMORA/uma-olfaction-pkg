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

/**  @moos_module Electronic nose (E-nose) simulator module.
  *  This module implements different types of ground truth gas maps, and simulate the readings of different sensors (using a first order model).
  *  For information about the first order model see "Overcoming the Slow Recovery of MOX Gas Sensors through a System Modeling Approach" at http://mapir.isa.uma.es/jgmonroy/
*/

#include "CEnoseSimulApp.h"
#include <mrpt/opengl.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::utils;

CEnoseSimulApp::CEnoseSimulApp()  :
	m_gt_gas_maps(NULL),
	m_last_simul_time(INVALID_TIMESTAMP),
	m_last_read_time(INVALID_TIMESTAMP),
	m_last_3D_GUI_time(INVALID_TIMESTAMP),
	m_rate_sensor(0.5),
	m_rate_refresh_gui(2.0),
	m_sensorLabel("ENOSE_SIMUL")
{
}

CEnoseSimulApp::~CEnoseSimulApp()
{
	mrpt::utils::delete_safe(m_gt_gas_maps);
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CEnoseSimulApp::OnStartUp()
{  // Can load params from m_MissionReader or m_ini

	cout << " --> Starting Enose Simulation Module" << endl;

	// ========================================================
	// (1) Create gas maps
	// ========================================================
	// Delete old object, if any (shouldn't!)
	mrpt::utils::delete_safe(m_gt_gas_maps);

	//! @moos_param simulator The simulator type to be used. Existing classes are: GasMapModel_Static and GasMapModel_Dynamic
	std::string sSimulName;
	sSimulName = m_ini.read_string("","simulator","",true);	
	cout << "[CEnoseSimulApp] Simulating with model: " << sSimulName << endl;	

	// This will create the appropriate gas map class, load all params
	//  and initialize the map contents:
	m_gt_gas_maps = GasMapModelBase::createFromName(sSimulName, m_ini);

	if (!m_gt_gas_maps)
	{
		MOOSTrace("*ERROR*: Unknown gas map simulator name!");
		return false;
	}

	// ========================================================
	// (2) Load e-Noses data
	// ========================================================
	/** @moos_param num_enoses The number of enoses to simulate.
	  * Note: all enoses are enabled to detect all types 
	  * of gases (i.e. all the individual maps simultaneously)
	  */
	const int nEnoses = m_ini.read_int("","num_enoses",0, true);
	ASSERT_ABOVE_(nEnoses,0)
	m_eNoses.resize(nEnoses);
	for (unsigned int i=0;i<m_eNoses.size();i++)
	{
		InfoPerENose &info = m_eNoses[i];
		//! @moos_param enose_pose_on_robot_x The (x,y) position on the robot
		info.x = m_ini.read_double("",format("enose%u_pose_on_robot_x",i),0, true);
		//! @moos_param enose_pose_on_robot_y The (x,y) position on the robot
		info.y = m_ini.read_double("",format("enose%u_pose_on_robot_y",i),0, true);

		//! @moos_param enose_noise_offset Offset noise to add to all readings (typ=0)
		info.noise_offset = m_ini.read_double("",format("enose%u_noise_offset",i),info.noise_offset);
		//! @moos_param enose_noise_std Additive gaussian noise (standard deviation)  (readings saturate to [0,1])
		info.noise_std    = m_ini.read_double("",format("enose%u_noise_std",i),info.noise_std);
		//! @moos_param enose_tau_rise 1st order model constant (seconds)
		info.tau_rise     = m_ini.read_double("",format("enose%u_tau_rise",i),info.tau_rise);
		//! @moos_param enose_tau_decay 1st order model constant (seconds)
		info.tau_decay    = m_ini.read_double("",format("enose%u_tau_decay",i),info.tau_decay);
	}

	// ========================================================
	// (3) Load other params
	// ========================================================
	//! @moos_param rate_sensor In Hz: rate of sensor measurements
	m_rate_sensor      = m_ini.read_double("","rate_sensor",m_rate_sensor);
	ASSERT_ABOVE_(m_rate_sensor,0)

	//! @moos_param rate_refresh_gui In Hz: rate of resent gas map to GUI
	m_rate_refresh_gui = m_ini.read_double("","rate_refresh_gui",m_rate_refresh_gui);
	ASSERT_ABOVE_(m_rate_refresh_gui,0)

	//! @moos_param sensor_label The name given to the field "Sensor_label" in the generated CObservation, and also name of the MOOS variable to publish
	m_sensorLabel = m_ini.read_string("","sensor_label",m_sensorLabel);

	// Refresh map in GUI:
	SendGasMap3DToGUI();

	return DoRegistrations();
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CEnoseSimulApp::OnCommandMsg( CMOOSMsg Msg )
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
bool CEnoseSimulApp::Iterate()
{
	// ============================================
	// 1) SIMULATE DYNAMIC GAS MAPS
	// ============================================
	if (m_last_simul_time==INVALID_TIMESTAMP)
	{
		m_last_simul_time = mrpt::system::now();  // Will simulate on next tick
	}
	else
	{
		const double At = mrpt::system::timeDifference(m_last_simul_time,mrpt::system::now());
		m_gt_gas_maps->simulateTimeInterval(At);
		m_last_simul_time = mrpt::system::now();
	}


	// ============================================
	// 2) SIMULATE NOISY GAS READINGS
	// ============================================
	if (m_last_read_time==INVALID_TIMESTAMP)
	{
		m_last_read_time = mrpt::system::now();
	}
	else
	{
		const double At_last_obs = mrpt::system::timeDifference(m_last_read_time,mrpt::system::now());
		if (At_last_obs>1./m_rate_sensor )
		{
			m_last_read_time = mrpt::system::now();

			// Read current robot position:
			mrpt::poses::CPose2D robotPose2D;
			CMOOSVariable * pVarLoc = GetMOOSVar( "LOCALIZATION" );
			if(pVarLoc && pVarLoc->IsFresh())
			{
				robotPose2D.fromString( pVarLoc->GetStringVal() );

				// Create output observation:
				mrpt::obs::CObservationGasSensors  obs;
				m_gt_gas_maps->simulateReadings(
					robotPose2D,
					m_eNoses,
					mrpt::system::now(), // time
					At_last_obs,    // seconds since last obs
					m_sensorLabel, // label
					obs);

				//!  @moos_publish <sensor_label> The MRPT Gas Sensor Observation of the simulated eNose parsed as a string through ObjectToString				
				m_Comms.Notify(obs.sensorLabel, ObjectToString( &obs ) );
			}
			else
			{
				MOOSTrace(mrpt::format("%s: Warning: 'LOCALIZATION' not available -> skipping simulation.\n", GetAppName().c_str()  ));
			}
		}
	}

	// ============================================
	// 3) Refresh map in GUI:
	// ============================================
	if (m_last_3D_GUI_time==INVALID_TIMESTAMP || mrpt::system::timeDifference(m_last_3D_GUI_time,mrpt::system::now())>1./m_rate_refresh_gui )
	{
		SendGasMap3DToGUI(); // This updates "m_last_3D_GUI_time"
	}


	return true;
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CEnoseSimulApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CEnoseSimulApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
	//! @moos_subscribe LOCALIZATION
	AddMOOSVariable( "LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0.1 );

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CEnoseSimulApp::OnNewMail(MOOSMSG_LIST &NewMail)
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

void CEnoseSimulApp::SendGasMap3DToGUI()
{
	// Render as 3D object:
	mrpt::opengl::CSetOfObjectsPtr gl_obj = mrpt::opengl::CSetOfObjects::Create();
	m_gt_gas_maps->getAs3DObject(gl_obj);


	// Send to GUI:
	// NOTE: This should be preferred when MOOS can handle very large blocks of binary data.
//	string sOpenglMapDump;
//	mrpt::utils::ObjectToRawString(gl_obj.pointer(), sOpenglMapDump);

	const string sTmpGridmap = mrpt::system::getTempFileName();
	m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap);
	CFileGZOutputStream(sTmpGridmap) << gl_obj;

	//! @moos_publish	CURRENT_MAP_GAS_SIMUL   A temporary file with a binary serialization of an opengl object (CSetOfObjects) with the current state of the gas map being simulated.
	m_Comms.Notify("CURRENT_MAP_GAS_SIMUL", sTmpGridmap );

	
	// Render as Matrix (GT values)
	mrpt::math::CMatrix M;
	m_gt_gas_maps->getAsMatrix(M);
	
	const string sTmpGridmap2 = mrpt::system::getTempFileName();
	m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap2);
	CFileGZOutputStream(sTmpGridmap2) << M;

	//! @moos_publish GT_GAS_SIMUL A temporary file with a binary serialization with the current state of the gas map being simulated (Ground Truth).
	m_Comms.Notify("GT_GAS_SIMUL", sTmpGridmap2 );

	m_last_3D_GUI_time = mrpt::system::now();
}

