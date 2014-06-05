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

#ifndef CEnoseSimulApp_H
#define CEnoseSimulApp_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <COpenMORAMOOSApp.h>

// All the types of gas map simulators:
#include "GasMapModels.h"


class CEnoseSimulApp : public COpenMORAApp
{
public:
    CEnoseSimulApp();
    virtual ~CEnoseSimulApp();

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


	void SendGasMap3DToGUI();

	// DATA. Your local variables here...

	/** A pointer to the ground truth (GT) gas map(s).
	  *  It's initialized in \a OnStartUp() from the .moos file params.
	  *  Existing implementations are in file "GasMapModels.h"
	  */
	GasMapModelBase  *m_gt_gas_maps;

	mrpt::system::TTimeStamp m_last_simul_time;		//!< Last simulation
	mrpt::system::TTimeStamp m_last_read_time;		//!< Last reading
	mrpt::system::TTimeStamp m_last_3D_GUI_time;	//!< Last time \a SendGasMap3DToGUI() was called.

	double  m_rate_sensor;       //!< In Hz: rate of sensor measurements
	double  m_rate_refresh_gui;  //!< In Hz: rate of resent gas map to GUI

	std::string m_sensorLabel;   //!< Sensor label to publish

	std::vector<InfoPerENose>  m_eNoses; //!< All the eNoses. Loaded at StartUp()

};
#endif

/** @moos_TODO 
  * Consider the robot movement to modify the gas concentration arround it.
  * Use a Computational Fluid Dynamics software to simulate the state of the gas at each instant, instead of use the basic image squence.
*/

/** @moos_changelog
  * 161-03-2012 Added Dynamic GasMap simulation, loading the state from a squence of images.
*/