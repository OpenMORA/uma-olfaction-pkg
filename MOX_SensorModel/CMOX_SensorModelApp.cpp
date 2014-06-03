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

/** @moos_module Module that implements a MOX gas sensor inverse model to improve the gas sensor readings.
  *  The objective is to overcome to a certain extent one of their major disadvantages: their slow recovery time (tens of seconds)
  *  This module exploits a double first-order model of the MOX sensor from which a steady-state output is anticipated in real time 
  *  given measurements of the transient state signal.
  *  For detailed description see paper: "Overcoming the slow recovery of MOX gas sensors through a system modeling approach" http://mapir.isa.uma.es/jgmonroy
*/

#include "CMOX_SensorModelApp.h"
#include <mrpt/utils.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;



CMOX_SensorModelApp::CMOX_SensorModelApp()
{
}

CMOX_SensorModelApp::~CMOX_SensorModelApp()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CMOX_SensorModelApp::OnStartUp()
{
	
	//!  @moos_param ENOSE_LABEL The name of the MOOS variable containing the Enose data to be displayed
	m_MissionReader.GetConfigurationParam( "ENOSE_LABEL", enose_label );

	
	//Load MOX model parameters
	//---------------------------
	//!  @moos_param a_rise Constant for the "linear fitting" applied to the rise time constants obtained from calibration (tau = a_rise*abs(reading-min_reading) + b_rise)
	m_MissionReader.GetConfigurationParam( "a_rise", a_rise );

	//!  @moos_param b_rise Constant for the "linear fitting" applied to the rise time constants obtained from calibration (tau = a_rise*abs(reading-min_reading) + b_rise)
	m_MissionReader.GetConfigurationParam( "b_rise", b_rise );

	//!  @moos_param a_decay Constant for the "linear fitting" applied to the decay time constants obtained from calibration (tau = a_decay*abs(reading-min_reading) + b_decay)
	m_MissionReader.GetConfigurationParam( "a_decay", a_decay );

	//!  @moos_param b_decay Constant for the "linear fitting" applied to the decay time constants obtained from calibration (tau = a_decay*abs(reading-min_reading) + b_decay)
	m_MissionReader.GetConfigurationParam( "b_decay", b_decay);

	//!  @moos_param winNoise_size Size of the noise-filtering window (smooth) used within the MOXmodel.
	winNoise_size = m_ini.read_uint64_t("","winNoise_size", 10, false);

	//!  @moos_param decimate_value Decimation value after noise-filtering (use 1 to dont apply decimation)
	m_MissionReader.GetConfigurationParam( "decimate_value", decimate_value );

	//!  @moos_param save_maplog Indicates (true/false) if a "log file" will be saved for offline matlab visualization.
	m_MissionReader.GetConfigurationParam( "save_maplog", save_maplog );		
	
	initialization_done = false;
	return DoRegistrations();
}



//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CMOX_SensorModelApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool CMOX_SensorModelApp::Iterate()
{
	// Get observation from MOX based E-nose
	try
	{
		// Get last observation from specified E-nose
		CMOOSVariable * pVarGas = GetMOOSVar( enose_label );
		if( pVarGas && pVarGas->IsFresh() )
		{
			pVarGas->SetFresh(false);
		
			CSerializablePtr obj;
			mrpt::slam::CObservationGasSensorsPtr gasObs;
			StringToObject(pVarGas->GetStringVal(),obj);
		
			if ( IS_CLASS(obj, CObservationGasSensors ))
				gasObs = CObservationGasSensorsPtr(obj);		
			else
			{
				cout << "[MOXmodel] Error: Obs is not CObservationGasSensors type" << endl;
				return false;
			}
			
			if (!initialization_done)
			{
				//Initialize structure to create a MOXmodel object for each sensor in the Enose
				//For each chamber				
				for( size_t ch=0; ch<gasObs->m_readings.size(); ch++ )
				{
					std::vector<mrpt::slam::CObservationGasSensors::CMOSmodel> CHmodels;
					//For each sensor
					for( size_t its=0; its<gasObs->m_readings[ch].sensorTypes.size(); its++ )
					{
						//Create and initialize model
						mrpt::slam::CObservationGasSensors::CMOSmodel  myMOXmodel;
						myMOXmodel.a_decay = a_decay;
						myMOXmodel.b_decay = b_decay;
						myMOXmodel.a_rise = a_rise;
						myMOXmodel.b_rise = b_rise;
						myMOXmodel.decimate_value = decimate_value;
						myMOXmodel.winNoise_size =  winNoise_size;
						myMOXmodel.save_maplog = save_maplog;
						//Save model in structure
						CHmodels.push_back(myMOXmodel);
					}
					MOXmodels.push_back(CHmodels);
				}
				initialization_done = true;
			}
			
		
			//--------------------------------------------------------------------------------------
			//Apply MOX inverse model to estimate Steady state values of all sensors in the E-nose
			//--------------------------------------------------------------------------------------

			//Make a Copy of original observation and populate with new sensor values.
			mrpt::slam::CObservationGasSensors gasObsMOXmodel;
			gasObsMOXmodel = *gasObs;
			gasObsMOXmodel.sensorLabel = gasObs->sensorLabel + "_MOXMODELED";

			//For each chamber
			for( size_t ch=0; ch<gasObs->m_readings.size(); ch++ )
			{				
				//For each sensor
				for( size_t its=0; its<gasObs->m_readings[ch].sensorTypes.size(); its++ )
				{
					bool have_estimation = false;
					float reading_model = gasObs->m_readings[ch].readingsVoltage[its];
					mrpt::system::TTimeStamp time_model = gasObs->timestamp;

					//Apply inverse model
					have_estimation =  MOXmodels[ch][its].get_GasDistribution_estimation(reading_model, time_model);
			
					if (have_estimation)
					{
						gasObsMOXmodel.m_readings[ch].readingsVoltage[its] = reading_model;
						gasObsMOXmodel.timestamp = time_model;						
					}
				}//end-for its
			}//end-for ch
			
			//!  @moos_publish <ENOSE_LABEL>_MOXMODELED The resulting GasSensor Observation after applying the MOX model.
			/**  @moos_var <ENOSE_LABEL>_MOXMODELED The resulting GasSensor Observation after applying the MOX model.
			   *  The variable name is formed by the input variable <ENOSE_LABEL> with the sufix "MOXMODELED".
			   */
			string sGDM = ObjectToString( &gasObsMOXmodel );
			m_Comms.Notify(gasObsMOXmodel.sensorLabel, sGDM );
			
		}
	
	}
	catch(exception &e)
	{
		cerr << "[CEnoseDisplayApp::Iterate] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
	catch(...)
	{
		cerr << "[CEnoseDisplayApp::Iterate] Returning false due to unknown exception: " << endl;
		return false;
	}
		
	return true;
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CMOX_SensorModelApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CMOX_SensorModelApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );	

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CMOX_SensorModelApp::OnNewMail(MOOSMSG_LIST &NewMail)
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