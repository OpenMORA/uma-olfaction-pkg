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

/**  @moos_module Graphical Display for CObservationGasSensors observations comming from electronic-noses
  *  This module enables the display of the different chambers and sensors of an elecronic nose publishing a CObservationGasSensors type of observation 
  *  See Enose_MCE and CGenericSensor::EnoseModular modules for examples.
  *  Sensor IDs
  *				Temperature  = 0xFFFF
  *				CRaePID		 = 0x0001
*/

#include "CEnoseDisplayApp.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::gui;


CEnoseDisplayApp::CEnoseDisplayApp()	
{
}

CEnoseDisplayApp::~CEnoseDisplayApp()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CEnoseDisplayApp::OnStartUp()
{
	mrpt::system::sleep(1000);

	//!  @moos_param ENOSE_LABEL The name of the MOOS variable containing the Enose data to be displayed
	m_MissionReader.GetConfigurationParam( "enose_label", enose_label );
	
	//!  @moos_param graphics_min_x Minimum value on the X axis plot (time axis) to display
	m_MissionReader.GetConfigurationParam( "graphics_min_x", graphics_min_x );
	//!  @moos_param graphics_max_x Maximum value on the X axis plot (time axis) to display.
	m_MissionReader.GetConfigurationParam( "graphics_max_x", graphics_max_x );
	//!  @moos_param graphics_min_y Minimum value on the Y axis plot (Votls axis) to display
	m_MissionReader.GetConfigurationParam( "graphics_min_y", graphics_min_y );
	//!  @moos_param graphics_max_y Maximum value on the Y axis plot (Votls axis) to display.
	m_MissionReader.GetConfigurationParam( "graphics_max_y", graphics_max_y );

	//DEBUG
	//enose_label = "EnoseModular";
	//graphics_min_x = -20.0;			// min value on the X axis plot (time)
	//graphics_max_x = 220.0;			// max value on the X axis plot (time)
	//graphics_min_y = -0.1;				// min value on the Y axis plot (Volts)
	//graphics_max_y = 0.7;			// max value on the Y axis plot (Volts)

	//Init params
	winExists = false;
	timStart = 0;

	//Create color palet
	colors[0] = "b-4";
	colors[1] = "r-4";
	colors[2] = "k-4";
	colors[3] = "m-4";
	colors[4] = "g-4";
	colors[5] = "c-4";
	
	return DoRegistrations();
}



//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CEnoseDisplayApp::OnCommandMsg( CMOOSMsg Msg )
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
bool CEnoseDisplayApp::Iterate()
{
	try
	{
		cout << "-------------------------------------" << endl;

		// Get last observation from specified E-nose
		CMOOSVariable * pVarGas = GetMOOSVar( enose_label );
		if( pVarGas && pVarGas->IsFresh() )
		{
			pVarGas->SetFresh(false);
		
			CSerializablePtr obj;
			mrpt::slam::CObservationGasSensorsPtr gasObs;
			mrpt::utils::RawStringToObject(pVarGas->GetStringRef(), obj);
		
			if ( IS_CLASS(obj, CObservationGasSensors ))
			{
				gasObs = CObservationGasSensorsPtr(obj);
				cout << "New gasObs: ";
			}
			else
			{
				cout << "Error: Obs is not CObservationGasSensors type" << endl;
				return false;
			}
			
			// Display values on screen
			//-----------------------------------------			
			cout << gasObs->sensorLabel.c_str() << endl;
			//Timestamps
			if (timStart == 0)
				timStart = gasObs->timestamp;
			else
				cout << "\t Timestamp: " << mrpt::system::timeDifference(timStart,gasObs->timestamp) << endl;

			//For each chamber
			for( size_t ch=0; ch<gasObs->m_readings.size(); ch++ )
			{
				printf("\t Chamber: %u\n", ch);
				if( gasObs->m_readings[ch].hasTemperature )
					printf("\t\t Temperature --> %.2f\n", gasObs->m_readings[ch].temperature);
				//For each sensor
				for( size_t its=0; its<gasObs->m_readings[ch].sensorTypes.size(); its++ )
				{
					int32_t model = gasObs->m_readings[0].sensorTypes[its]%10000;
					printf("\t\t %u-> %.2f \n",model,gasObs->m_readings[0].readingsVoltage[its]);
				}
			}
			
			//Add obs to Graphical display
			//addToPlot(gasObs);					
			return true;
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
	
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CEnoseDisplayApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CEnoseDisplayApp::DoRegistrations()
{
	//DEBUG
	//AddMOOSVariable( "EnoseModular", "EnoseModular", "EnoseModular", 0 );

	//! @moos_subscribe <enose_label>
	AddMOOSVariable( enose_label, enose_label, enose_label, 0 );

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );	

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CEnoseDisplayApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;	
	for(MOOSMSG_LIST::iterator i=NewMail.end();i!=NewMail.begin();i--)
	{
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


/*-------------------------------------------------------------
					addToPlot
-------------------------------------------------------------*/
void CEnoseDisplayApp::addToPlot(mrpt::slam::CObservationGasSensorsPtr &obs)
{
	try
	{	
		//If this is the first observation, resize windows according to the number of chambers to display
		if ( sensorReadings.empty() )
		{
			printf("[CEnoseDisplay]: Creating Display = %u Windows\n",obs->m_readings.size()+1 );
			sensorReadings.resize( obs->m_readings.size() );			
		}
		else	
		{
			//Ensure both dimensions match
			if( sensorReadings.size() != obs->m_readings.size() )
			{
				cout  << "[CEnoseDisplay]: Error in GasObservation. Dimensions do not math previous observations" << endl;
				return;
			}
		}
				
		//Add new timeStamp (seconds)
		timeReadings.push_back( (float) mrpt::system::timeDifference(timStart,obs->timestamp) );	
		//Keep a max number of samples in Memory
		if (timeReadings.size() > 600)
			timeReadings.erase(timeReadings.begin());
				
		//For each Chamber
		for (size_t ch=0; ch<sensorReadings.size(); ch++)
		{
			//Add Temperature
			//---------------			
			if (obs->m_readings[ch].hasTemperature)
				sensorReadings[ch][0xFFFF].push_back(obs->m_readings[ch].temperature);
			else
				sensorReadings[ch][0xFFFF].push_back(0x0000);			
			//Keep a max number of samples in Memory
			if (sensorReadings[ch][0xFFFF].size() > 600)
				sensorReadings[ch][0xFFFF].erase( sensorReadings[ch][0xFFFF].begin() );
						
			//Add gas sensor readings
			//-------------------------
			for (size_t j=0; j<obs->m_readings[ch].sensorTypes.size(); j++)
			{
				sensorReadings[ch][(size_t)obs->m_readings[ch].sensorTypes[j]%10000].push_back( obs->m_readings[ch].readingsVoltage[j] );
				if (sensorReadings[ch][(size_t)obs->m_readings[ch].sensorTypes[j]%10000].size() > 600)
					sensorReadings[ch][(size_t)obs->m_readings[ch].sensorTypes[j]%10000].erase( sensorReadings[ch][(size_t)obs->m_readings[ch].sensorTypes[j]%10000].begin() );
			}
		
		}//end-for each chamber
		
		
		//Update display
		updatePlot();		
	}
	catch(exception &e)
	{
		cerr << "[CEnoseDisplayApp::Iterate] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return;
	}
	catch(...)
	{
		cerr << "[CEnoseDisplayApp::Iterate] Returning false due to unknown exception: " << endl;
		return;
	}
}



/*-------------------------------------------------------------
					updatePlot
-------------------------------------------------------------*/
void CEnoseDisplayApp::updatePlot()
{
	try
	{		
		//Create the windows if not already created and update the plot.
		if (!winExists)
		{
			//Windows 0 to N for the different Chambers on the e-nose
			//window N+1 for Temperature

			winExists = true;

			//Get screen resolution
			int horizontal = 0;
			int vertical = 0;
			GetDesktopResolution(horizontal, vertical);

			//Set Each window size
			int xSize = horizontal*0.9/sensorReadings.size();
			int ySize = vertical*0.7/2;

			int posx = 0;
			int posy = 0;

			//Create a window for each Chamber
			for( size_t ch=0; ch<sensorReadings.size(); ch++ )
			{
				/*winMap.push_back( new CDisplayWindowPlots( format("Chamber: %i",(int)ch) ) );
				winMap[ch]->resize(xSize,ySize);
				winMap[ch]->axis(graphics_min_x,graphics_max_x,graphics_min_y,graphics_max_y,false);
				winMap[ch]->axis_equal(false);
				winMap[ch]->setPos(posx*(xSize+20),posy*(ySize+60));
				winMap[ch]->hold_off();
				posx++;*/

				//Alternative
				CDisplayWindowPlots newPlot;
				mrpt::gui::CDisplayWindowPlotsPtr newWinPtr = newPlot.Create( format("Chamber: %i",(int)ch) );
				newWinPtr->resize(xSize,ySize);
				newWinPtr->axis(graphics_min_x,graphics_max_x,graphics_min_y,graphics_max_y,false);
				newWinPtr->axis_equal(false);
				newWinPtr->setPos(posx*(xSize+20),posy*(ySize+60));
				newWinPtr->hold_off();
				winMapPro.push_back(newWinPtr);
				posx++;
			}

			//Create a window for Temperature display
			/*winMap.push_back( new CDisplayWindowPlots( format("Temperature") ) );
			winMap[sensorReadings.size()]->resize(xSize,ySize);
			winMap[sensorReadings.size()]->axis(graphics_min_x,graphics_max_x,-5,50,false);
			winMap[sensorReadings.size()]->axis_equal(false);				
			winMap[sensorReadings.size()]->setPos(posx*(xSize+20),posy*(ySize+60));
			winMap[sensorReadings.size()]->hold_off();*/

			//Alternative
			CDisplayWindowPlots newPlot;
			mrpt::gui::CDisplayWindowPlotsPtr newWinPtr = newPlot.Create( format("Temperature") );
			newWinPtr->resize(xSize,ySize);
			newWinPtr->axis(graphics_min_x,graphics_max_x,-5,50,false);
			newWinPtr->axis_equal(false);
			newWinPtr->setPos(posx*(xSize+20),posy*(ySize+60));
			newWinPtr->hold_off();
			winMapPro.push_back(newWinPtr);

		}//end-if !winExists
		
		//Display Chambers
		for( size_t ch=0; ch<sensorReadings.size(); ch++ )
		{
			size_t color = 0;			
			for( map<size_t, std::vector<float> >::iterator iter = sensorReadings[ch].begin(); iter != sensorReadings[ch].end(); ++iter )
			{
				if (iter->first == (0xFFFF)) 	//Temperature to the last window plot
				{
					cout << "Plotting " << timeReadings.size() << " measurements." << endl;					
					//winMap[sensorReadings.size()]->plot(timeReadings, sensorReadings[ch][0xFFFF], colors[ch], "Temperature");
					if( timeReadings.size() == sensorReadings[ch][0Xffff].size() )
					{
						//winMapPro.back()->plot(timeReadings, sensorReadings[ch][0xFFFF], colors[ch], "Temperature");
					}
				}
				else
				{					
					//cout << "Ploting sensor: " << format("%i",int(iter->first)) << endl;
					if( timeReadings.size() == sensorReadings[ch][iter->first].size() )
					{
						//winMap[ch]->plot( timeReadings, sensorReadings[ch][iter->first], colors[color], format("%i",int(iter->first)) );
						//winMapPro[ch]->plot( timeReadings, sensorReadings[ch][iter->first], colors[color], format("%i",int(iter->first)) );
						color++;
						if( color > 5)
							color = 0;
					}
					else
						cout << "Error: measurements have incorrect length." << endl;
				}
			}
		}//End-for
		
		//Display vertical line on current time
		//--------------------------------------
		/*std::vector<float> time_line, line;
		time_line.resize(10);
		line.resize(10);
		for (size_t i=0;i<time_line.size();i++)
		{
			time_line[i]= timeReadings.back();
			line[i]=i/10.0;
		}
		winMap[0]->plot(time_line, line ,colors[0x0000], format("%i",int(0x0000)));*/

	}
	catch(exception &e)
	{
		cerr << "[CEnoseDisplayApp::updatePlot] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return;
	}
	catch(...)
	{
		cerr << "[CEnoseDisplayApp::updatePlot] Returning false due to unknown exception: " << endl;
		return;
	}

}

/*-------------------------------------------------------------
					GetDesktopResolution (windows)
-------------------------------------------------------------*/
void CEnoseDisplayApp::GetDesktopResolution(int& horizontal, int& vertical)
{	
	#ifdef MRPT_OS_WINDOWS
		RECT desktop;
		// Get a handle to the desktop window
		const HWND hDesktop = GetDesktopWindow();
		// Get the size of screen to the variable desktop
		GetWindowRect(hDesktop, &desktop);
		// The top left corner will have coordinates (0,0)
		// and the bottom right corner will have coordinates
		// (horizontal, vertical)
		horizontal = desktop.right;
		vertical = desktop.bottom;
	#else
		horizontal = 800;
		vertical = 600;
		/*
		GdkScreen screen*;
		screen = gdk_screen_get_default();
		gdouble resolution = gdk_screen_get_resolution (screen);
		*/
	#endif

}