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

#ifndef CDistributionMappingApp_H
#define CDistributionMappingApp_H

#include <COpenMORAMOOSApp.h>
#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/utils.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/threads.h>
#include <mrpt/slam/CRandomFieldGridMap2D.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CObservationWindSensor.h>


#define INVALID_SENSOR_INDEX  size_t(-1)

class CDistributionMappingApp : public COpenMORAApp
{
public:
    CDistributionMappingApp();
    virtual ~CDistributionMappingApp();

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
	void SaveGasMap();

	// DATA. Your local variables here...
	mrpt::slam::CRandomFieldGridMap2D::TMapRepresentation mapType;				/** The type of map to generate (kalman, kernel, etc) */
	mrpt::slam::CGasConcentrationGridMap2D	*my_map;							/** The Online Gas Distribution Map being generated */
	mrpt::slam::COccupancyGridMap2D	m_gridmap;									/** The occupancy_grid map (optional) */

	bool use_occupancy_information;												/** [GMRF] Wheter consider metric map to build the MRF*/
	double  m_rate_refresh_gui,STD_increase_value,std_windNoise_phi,std_windNoise_mod;
	bool SAVE_MAP_SCENE;
	std::string OUT_DIR;
	int advectionTickFreq;
	float LOG_FREQUENCY;
	bool insert_from_rawlog;
	std::string rawlog_path, sSimplemapFil, sGridmapFil;
	mrpt::system::TTimeStamp timeLastSimulated;		 /** The timestamp of the last time the advection simulation was executed */
	mrpt::system::TTimeStamp timeLastSaved;
	mrpt::poses::CPose3D sensorPose3D;
	mrpt::poses::CPose3D robotPose3D;
	double local_width, local_height;
	size_t step;
	mrpt::utils::CFileGZInputStream rawlog_input;
	int indexMonitoredSensor;
	bool end_of_rawlog;
};
#endif


/** @moos_TODO
  * At the moment this modules has been implemented only for GasConcentration maps. Modify to work with other gridmaps.
  * Add a new mapping algorithm the "InformationFilter" to avoid the instability of KalmanFilters.
  * Solve a problem when the reference map needs to be resized. (Runtime Error)
*/

/** @moos_changelog
  * 21-02-2012 Added a function that increase the STD of all cells in the m_map, to simulate loss of confidence
  * 19-03-2012 Fixed problem in the GUI visualization of the maps generated in this module. Now only the main map is sent to the GUI.
  * 19-03-2012 Added new parameters to set the initial Local map dimensions.
  * 20-0.-2012 New variable published to draw in the GUI a black square arround the robot, corresponding to the local map limits.
  * 08-04-2013 Added workflow to load data from an existing rawlog-file
*/