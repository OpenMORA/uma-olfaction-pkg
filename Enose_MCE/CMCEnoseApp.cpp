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

/**  @moos_module Interface with the Multi Chamber Electronic nose (MCE-nose).
  *  This module comunicates with the MCE-nose through a MRPT CBoardENoses interface
  *  1- Implements different switching strategies for the selection of the new Active Chamber
  *  2- Gets the ID of the actual Active chamber (chamber being odor flooded).  
  *  3- Shares the readings with other OpenMORA modules.
*/

#include "CMCEnoseApp.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;

CMCEnoseApp::CMCEnoseApp():
	m_initialized_ok(false),
	indexMonitoredSensor(INVALID_SENSOR_INDEX),
	delay(3.0),
	maxAverage(0.0)	
{
}

CMCEnoseApp::~CMCEnoseApp()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CMCEnoseApp::OnStartUp()
{
	// Load GenericSensor paremeters
	//!  @moos_param <GenericSensor> The MRPT CGenericSensor parameters
	eNoses.loadConfig( m_ini, "pMCEnose" );

	//!  @moos_param num_chambers The number of chambers in the e-nose device (see the OpenMORA wiki for more info)
	m_MissionReader.GetConfigurationParam( "num_chambers", num_chambers );
	
	//!  @moos_param num_sensors_chamber The number of gas sensors to be read on each chamber.
	m_MissionReader.GetConfigurationParam( "num_sensors_chamber", num_sensors_chamber);
	
	//!  @moos_param strategy The MCE-nose Switching strategy: 0(No Switching), 1(Periodic Switching), 2(Adaptive), 3(Calibration), 4(Dual mode). (see the OpenMORA wiki for more info)
	m_MissionReader.GetConfigurationParam( "strategy", strategy );

	//!  @moos_param sensorID The sensor type (0x2620, 0x2601, etc) to be taken into account for the switching strategy. If sensorID = 0 the mean of all sensors is considered.
	sensorID = m_ini.read_uint64_t("","sensorID", sensorID );

	//!  @moos_param minActiveTime Minimum time (seconds) a chamber must be active (odor flooded). In periodic Switching (strategy = 1) this is the Period.
	m_MissionReader.GetConfigurationParam( "minActiveTime", minActiveTime );

	//!  @moos_param decayValue Decay value(%) in the "active chamber" readings required in the Adaptive switching mode, to consider that a decay phase is starting.(maxValue - maxValue*decayValue/100.0 )
	m_MissionReader.GetConfigurationParam( "decayValue", decayValue );

	//!  @moos_param min_diff Minimum difference (value) that must exists between the readings of the actual active chamber (AC) and the candidate Chamber to switch at, to allow switching.
	m_MissionReader.GetConfigurationParam( "min_diff", min_diff );

	//!  @moos_param delay Delay to introduce to the gas readings as consequence of the pneumatic circuit and electronics (seconds)
	m_MissionReader.GetConfigurationParam( "delay", delay );


	// Initialize default parameters.
	last_time = start_time = mrpt::system::getCurrentTime();
	return DoRegistrations();
}

//-------------------------------------
// parseTextToVector()
//-------------------------------------
bool CMCEnoseApp::parseTextToVector(std::string &aux, mrpt::math::CVectorFloat &outValues)
{
	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		return false;
	}
	else
	{
		// Parse to numeric type:
		const size_t N = tokens.size();
		outValues.resize( N );
		size_t idxDst;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), idxDst=0;itSrc!=tokens.end();itSrc++,idxDst++)
			outValues[idxDst] = atof(itSrc->c_str());
		return true;
	}
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CMCEnoseApp::OnCommandMsg( CMOOSMsg Msg )
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
bool CMCEnoseApp::Iterate()
{
	//-----------------------------------
	// Get observation from the MCE-nose
	//------------------------------------
	try
	{
		mrpt::slam::CObservationGasSensors newObs;
		bool obs_ok;
		obs_ok = eNoses.getObservation( newObs );
		if (!obs_ok )
		{
			cout << "- Could not retrieve an observation from the eNoses..." << endl;
			return true;
		}
		else
			obs = newObs;		
	}
	catch(exception &e){
		printf("%s \n", e.what() );
	}


	//---------------------------------
	// Validate observation structure
	//---------------------------------
	ASSERT_(obs.m_readings.size() == num_chambers);

	for (size_t ch=0; ch<obs.m_readings.size(); ch++)
	{
			ASSERT_(obs.m_readings[ch].sensorTypes.size() == num_sensors_chamber);
			ASSERT_(obs.m_readings[ch].readingsVoltage.size() == num_sensors_chamber);
	}


	//----------------------------
	//Check active Chamber on MCE
	//----------------------------
	bool active_found = false;
	acChamber = 0;
	while ( (!active_found) && (acChamber < num_chambers) )
	{
		if (obs.m_readings[acChamber].isActive)
			active_found = true;
		else
			acChamber++;
	}

	if (!active_found){
		printf("Active chamber NOT found - Calibration mode detected (All chambers being odor flooded.\n"); //For debug
		printf("Selecting Chamber(0) as default active chaber\n"); //For debug
		acChamber = 0;
		//return false;
	}else
		printf("Active chamber=%i \n",acChamber); //For debug
	


	//--------------------------------------------------
	// Get reading from activeChamber - selected sensor
	//--------------------------------------------------
	float activeChamberReading;

	indexMonitoredSensor = INVALID_SENSOR_INDEX;
	if (sensorID == 0)	//compute the mean
	{
		activeChamberReading = math::mean( obs.m_readings[acChamber].readingsVoltage );
	}
	else
	{
		// Get the reading of the specified sensorID
		if (indexMonitoredSensor == INVALID_SENSOR_INDEX)
		{
			//First reading, get the index according to sensorID
			for (indexMonitoredSensor=0; indexMonitoredSensor< obs.m_readings[acChamber].sensorTypes.size(); indexMonitoredSensor++)
			{
				if (obs.m_readings[acChamber].sensorTypes.at(indexMonitoredSensor) == vector_int::value_type(sensorID) )
					break;
			}
		}

		if (indexMonitoredSensor< obs.m_readings[acChamber].sensorTypes.size())
		{
			activeChamberReading = obs.m_readings[acChamber].readingsVoltage.at(indexMonitoredSensor);
		}
		else //Sensor especified not found, compute default mean value
		{
			activeChamberReading = math::mean( obs.m_readings[acChamber].readingsVoltage );
		}
	}

	

	//------------------------------------------------------------
	// PUBLISH VARIABLES
	//------------------------------------------------------------

	// Correct Delay in Sensor Readings	
	if (mrpt::system::timeDifference(start_time,obs.timestamp) > delay)
		obs.timestamp = obs.timestamp - delay*10000000;
	else
		obs.timestamp = start_time;

		
	//-----------------------------------------
	// PUBLISH MCE-NOSE OBS (all chambers)
	//-----------------------------------------
	mrpt::slam::CObservationGasSensorsPtr FullMCEoutput = CObservationGasSensors::Create();
	FullMCEoutput->sensorLabel = "ENOSE_MCE_OBS";
	FullMCEoutput->timestamp = obs.timestamp;
	//Copy the content
	FullMCEoutput->m_readings = obs.m_readings;
	string sfObs = ObjectToString( FullMCEoutput.pointer() );
	//!  @moos_publish   ENOSE_MCE_OBS   The MCE e-nose gas observation as a CObservationGasSensorsMRPT parsed as a string through ObjectToString		
	m_Comms.Notify(FullMCEoutput->sensorLabel , sfObs );

		
	//---------------------------------
	//Publish content of Active Chamber
	//---------------------------------
	mrpt::slam::CObservationGasSensorsPtr MCEoutput = CObservationGasSensors::Create();
	MCEoutput->sensorLabel = "ENOSE_MCE_AC_OBS";
	MCEoutput->timestamp = obs.timestamp;
	MCEoutput->m_readings.push_back( obs.m_readings[acChamber] );				
	string sObs = ObjectToString( MCEoutput.pointer() );
	//!  @moos_publish ENOSE_MCE_AC_OBS The ActiveChamber observation of the MCE e-nose as a CObservationGasSensorsMRPT parsed as a string through ObjectToString		
	m_Comms.Notify(obs.sensorLabel, sObs );
	


	//-----------------------------------------------------
	// Update AC according to selected Switching strategy
	//-----------------------------------------------------
	switch (strategy)
	{
	case 0:	// No Switching Strategy
		// Similar to conventional Enose. The active chamber is always the same
		// Do nothing.
		break;

	case 1:	//Fixed: Periodic Switching given by minActiveTime(s)
		time =	mrpt::system::getCurrentTime();
		if ( mrpt::system::timeDifference(last_time,time) > minActiveTime) //seconds
		{
			//Select the next chaber as the AC
			acChamber++;
			if (acChamber>=num_chambers)
				acChamber=0;

			//Send nextActiveChamber to MCEnose
			eNoses.setActiveChamber(acChamber);
			last_time = mrpt::system::getCurrentTime();
		}
		break;

	case 2: //Adaptive Switching: Switchs according to the gas readings

		// To reduce the noise in the measurements and improve the switching process, use a moving average filter
		actualAverage = UpdateReadings ( activeChamberReading );

		if( actualAverage > maxAverage )
		// Rise Phase (No Switching)
		{
			//Update moving average (for max rise values)
			maxAverage = UpdateMaxReadings ( activeChamberReading );
		}
		else if( (maxAverage - actualAverage) > decayValue )
		// Decay phase
		{
			time =	mrpt::system::getCurrentTime();
			// Ensure minimum excitation time (seconds)
			if ( mrpt::system::timeDifference(last_time,time) > minActiveTime)
			{
				SwitchChamber(false,num_chambers);
			}
		}//end-if decaying
		break;

	case 3: //CALIBRATION: all chambers active
		eNoses.setActiveChamber(15); //send 1111 to MCE-nose to open all chambers
		break;

	case 4:	//Combined behaviour: Chamber 0 will always be opened but never as activeChamber, and we only swith between the other chambers 1, 2 , 3 ...
		
		if (acChamber == 0 ) //Ensure Activechamber != 0 (may happen at start up)
		{
			SwitchChamber(true,num_chambers-1);	//Force switching to other chamber
		}
		else
		{		
			// Update moving average
			actualAverage = UpdateReadings ( activeChamberReading );

			if( actualAverage > maxAverage )
			// Rise Phase (No Switching)
			{
				//Update moving average (for max rise values)
				maxAverage = UpdateMaxReadings ( activeChamberReading );
			}
			else if( (maxAverage - actualAverage) > decayValue )
			// Decay phase
			{
				time =	mrpt::system::getCurrentTime();
				// Ensure minimum excitation time (seconds)
				if ( mrpt::system::timeDifference(last_time,time) > minActiveTime)
				{
					SwitchChamber(false,num_chambers-1);
				}
			}//end-if decaying
		}
		break;
	}//end-case

	return true;
}


//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CMCEnoseApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CMCEnoseApp::DoRegistrations()
{
	//! @moos_subscribe ENOSE_MCE_SWITCH_CHAMBER
	//! @moos_var ENOSE_MCE_SWITCH_CHAMBER Variable to make the MCE-nose to switch chamber
	AddMOOSVariable( "ENOSE_MCE_SWITCH_CHAMBER", "ENOSE_MCE_SWITCH_CHAMBER", "ENOSE_MCE_SWITCH_CHAMBER", 0 );

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );	

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CMCEnoseApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;

	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
	//	MOOSTrace(format("New msg received: %s \n",i->GetKey()));
		if (i->GetName() == "ENOSE_MCE_SWITCH_CHAMBER")
		{
			MOOSTrace("\n ----------------------------------------------------\n An External Module Ordered a Switching.... \n ----------------------------------------------------\n");
			SwitchChamber(true,num_chambers);	//force switching
		}
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


//-------------------------------------
// UpdateReadings()
//-------------------------------------
float CMCEnoseApp::UpdateReadings(float activeChamberReading )
{
	//Keep the last 10 values to average them
	lastValues.push_back(activeChamberReading);
	if (lastValues.size() > 10)
		lastValues.erase( lastValues.begin() );

	//Estimate the new actual average
	float av = 0.0;
	for( unsigned int i=0 ; i<lastValues.size() ; i++ )
	{
		av = av + lastValues[i];
	}
	return av/(lastValues.size()) ;
}


//-------------------------------------
// UpdateMaxReadings()
//-------------------------------------
float CMCEnoseApp::UpdateMaxReadings(float activeChamberReading )
{
	maxValues.push_back(activeChamberReading);
	if (maxValues.size() > 10)
	{
		maxValues.erase( maxValues.begin() );
	}
	//Estimates the new maximum average
	float mav = 0.0;
	for( unsigned int i=0 ; i<maxValues.size() ; i++ )
	{
		mav = mav + maxValues[i];
	}
	return mav/(maxValues.size());
}


//-------------------------------------
// SwitchChamber()
//-------------------------------------
void CMCEnoseApp::SwitchChamber(bool forced, unsigned int chambers_available)
{

	try
	{
		//Check the best chamber to switch (the one with lower output)
		float min_val=100;
		size_t best_acChamber=0;

		if (chambers_available == num_chambers)	//All the chambers on the MCE-nose are available to be switched
		{
			if( (sensorID == 0) || (indexMonitoredSensor >= obs.m_readings[best_acChamber].sensorTypes.size()) )	//compute the mean
			{
				min_val = math::mean( obs.m_readings[0].readingsVoltage );
				for (unsigned char ch=1; ch<num_chambers; ch++)
				{
					if (math::mean( obs.m_readings[ch].readingsVoltage ) < min_val)
					{
						min_val = math::mean( obs.m_readings[ch].readingsVoltage );
						best_acChamber = ch;
					}
				}
			}
			else
			{
				for (unsigned char ch=0; ch<num_chambers; ch++)
				{
					for (size_t j=0; j< obs.m_readings[ch].sensorTypes.size(); j++)
					{
						if( (obs.m_readings[ch].sensorTypes[j] == vector_int::value_type(sensorID)) && (obs.m_readings[ch].readingsVoltage[j] < min_val) )
						{
							min_val = obs.m_readings[ch].readingsVoltage[j];
							best_acChamber = ch;
						}
					}
				}
				/*min_val = obs.m_readings[0].readingsVoltage[indexMonitoredSensor];
				for (unsigned char ch=1; ch<num_chambers; ch++)
				{
					if (obs.m_readings[ch].readingsVoltage[indexMonitoredSensor] < min_val)
					{
						min_val = obs.m_readings[ch].readingsVoltage[indexMonitoredSensor];
						best_acChamber = ch;
					}
				}*/
			}
		}
		else if (chambers_available == num_chambers-1)	//Chamber 0 can not be selected to be switched.
		{
			best_acChamber = 1;

			if( (sensorID == 0) )	//compute the mean
			{
				min_val = math::mean( obs.m_readings[1].readingsVoltage );
				for (unsigned char ch=2; ch<num_chambers; ch++)
				{
					if (math::mean( obs.m_readings[ch].readingsVoltage ) < min_val)
					{
						min_val = math::mean( obs.m_readings[ch].readingsVoltage );
						best_acChamber = ch;
					}
				}
			}
			else
			{
				for (unsigned char ch=1; ch<num_chambers; ch++)
				{
					for (size_t j=0; j< obs.m_readings[ch].sensorTypes.size(); j++)
					{
						if( (obs.m_readings[ch].sensorTypes[j] == vector_int::value_type(sensorID)) && (obs.m_readings[ch].readingsVoltage[j] < min_val) )
						{
							min_val = obs.m_readings[ch].readingsVoltage[j];
							best_acChamber = ch;
						}
					}
				}
			}
			// Modify the best_acChamber accordingly with the firmware (4=CH0+CH1, 5=CH0+CH2 ,6=CH0+CH3)
			best_acChamber+=3;
		}



		if (forced)
		{
			MOOSTrace(format("\n Switching to %i \n", (int)best_acChamber));
			//Apply changes
			eNoses.setActiveChamber(best_acChamber);
			last_time = mrpt::system::getCurrentTime();
			maxAverage = 0.0;
			maxValues.erase(maxValues.begin(),maxValues.end());
			//MOOSTrace(" - Salgo \n");
		}
		else if ((actualAverage - min_val) > min_diff)
		{
			//Ensure that the new candidate chamber's reading is at least min_diff volts lower than current AChamber
			//To avoid innecesary switchings when all chambers are clean
			MOOSTrace(format("\n Best candidate %i. Actual average is %f",(int)best_acChamber, actualAverage));
			MOOSTrace(format("\n Switching to %i \n", (int)best_acChamber));
			//Apply changes
			eNoses.setActiveChamber(best_acChamber);
			last_time = mrpt::system::getCurrentTime();
			maxAverage = 0.0;
			maxValues.erase(maxValues.begin(),maxValues.end());
			//MOOSTrace(" - Salgo \n");
		}
		else
		{
			//MOOSTrace(format("\n No Switching as max_difference is %f \n", (actualAverage - min_val)));
		}



	}
	catch(exception &e){
		printf("Exception when Switching: %s \n", e.what() );
	}
}
