/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Javier Gonzalez Monroy  <jgmonroy@isa.uma.es>                 |
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

#ifndef CMOX_SensorModelApp_H
#define CMOX_SensorModelApp_H

#include <COpenMORAMOOSApp.h>
#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/slam/CObservationGasSensors.h>

#define INVALID_SENSOR_INDEX size_t(-1)

class CMOX_SensorModelApp : public COpenMORAApp
{
public:
    CMOX_SensorModelApp();
    virtual ~CMOX_SensorModelApp();

protected:
	
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
	/** state our interest in variables from other modules (registration for mail)*/
	bool DoRegistrations();

	
	// DATA. Your local variables here...
	//---------------------------------------------

	std::string																		enose_label;	// The name of the MOOS variable containing the e-nose observations to be model
	std::vector< std::vector<mrpt::slam::CObservationGasSensors::CMOSmodel> >		MOXmodels;
	
	//MOX model parameters
	size_t        winNoise_size;              // The size of the mobile average window used to reduce noise on sensor reagings.
	int           decimate_value;             // The decimate frecuency applied after noise filtering
						
	float         a_rise;                     // tau = a*AMPLITUDE +b (linear relationship)
	float         b_rise;                     // tau = a*AMPLITUDE +b (linear relationship)
	float         a_decay;                    // tau = a*AMPLITUDE +b (linear relationship)
	float         b_decay;                    // tau = a*AMPLITUDE +b (linear relationship)
												
	bool          save_maplog;                // If true save generated gas map as a log file
	bool		  initialization_done;
};
#endif