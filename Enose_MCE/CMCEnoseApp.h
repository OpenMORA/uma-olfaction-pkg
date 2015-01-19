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

#ifndef CMCEnoseApp_H
#define CMCEnoseApp_H

#include <COpenMORAMOOSApp.h>
#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/math.h>
#include <mrpt/utils.h>

#define INVALID_SENSOR_INDEX size_t(-1)

class CMCEnoseApp : public COpenMORAApp
{
public:
    CMCEnoseApp();
    virtual ~CMCEnoseApp();

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

	// Local funtions
	//--------------------------------------------

	/** parse text to vector. Used to read vector from mission file */
	bool parseTextToVector(std::string &aux, mrpt::math::CVectorFloat &outValues);
	
	/** Updates a mobile average window and returns the actual average reading */
	float UpdateReadings(float activeChamberReading );
	
	/** Adds a new reading if it is > actual average, and returns the average of the last maximum values*/
	float UpdateMaxReadings(float activeChamberReading );
	
	/** Indicates the MCEnose to switch the Active Chamber to a new one. If forced = true no verifications are made. */
	void SwitchChamber(bool forced, unsigned int chambers_available);

	// DATA. Your local variables here...
	//---------------------------------------------

	/** The MCEnose board class */
	mrpt::hwdrivers::CBoardENoses			eNoses;

	/** The MCEnose observation class */
	mrpt::obs::CObservationGasSensors		obs;
	
	bool						m_initialized_ok;
	unsigned int				num_chambers;			/** MCEnose chambers number */
	unsigned int				num_sensors_chamber;	/** Number of sensors on each cahmber */
	unsigned int				strategy;				/** Switching strategy 0->Periodic, 1->Adaptive */
	size_t						sensorID;				/** Sensor ID to be procesed between all sensors (only one sensor) */
	size_t						indexMonitoredSensor;	/** Index of the array that contains the Reading of the monitored sensors, or INVALID_SENSOR_INDEX */
	double						minActiveTime;			/** Minimun time [sec] a chamber must be active (odor flooded). In periodic Switching this is the Period */
	float						decayValue;				/** [Adaptive switching] Decay in the active chamber readings (%) to consider a decay phase --> Switch to other chamber (maxValue - maxValue*decayPercent/100.0 ) */
	float						min_diff;				/** Minimum reading diference between the actual AC and the candidate AC, to avoid switching */
	float						delay;					/** Aproximate delay introduced by the pneumatic circuit.*/

	float						maxAverage;
	float						actualAverage;

	std::vector<float>			maxValues;				/** [Adaptive switching] VectorMax reading of the active chamber for the current excitation. Used to detect the start of the decay phase (maxValue - maxValue*0.08) */
	std::vector<float>			lastValues;				/** Vector containing the last readed values for the active chamber (used to estimate the start of a decaying phase) */
	unsigned char				acChamber;				/** chamber being odor flooded (Active Chumber) */
	mrpt::system::TTimeStamp	time,last_time,start_time;
	
};
#endif

/** @moos_TODO
  * Separate the MOXmodel in a different module, reading the variables published by this module and generating the MOXmodel output.

*/