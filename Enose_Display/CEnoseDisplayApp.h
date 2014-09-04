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

#ifndef CEnoseModularApp_H
#define CEnoseModularApp_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <COpenMORAMOOSApp.h>
#include <mrpt/hwdrivers/CEnoseModular.h>
#include <mrpt/gui/CDisplayWindowPlots.h>


class CEnoseDisplayApp : public COpenMORAApp
{
public:
    CEnoseDisplayApp();
    virtual ~CEnoseDisplayApp();

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


	//------------------
	// Local funtions
	//------------------
	std::string					enose_label;			// The name of the MOOS variable containing the e-nose observations to be displayed
	
	float						graphics_min_x;			/** min value on the X axis plot (time) */
	float						graphics_max_x;			/** max value on the X axis plot (time) */
	float						graphics_min_y;			/** min value on the Y axis plot (Volts) */
	float						graphics_max_y;			/** max value on the Y axis plot (Volts) */

	/** A map to store all the readings of each chamber of each sensor to be plotted*/
	std::vector< std::map<size_t, std::vector<float> > > sensorReadings;

	/** Time vector for plots */
	std::vector<float>  timeReadings;

	/** Timestamp of the first observation received */
	mrpt::system::TTimeStamp				timStart;
		
	/** Define when the GUI windows are already created or not (only false before first call to Plot)*/
	bool winExists;

	/** Define colors */
	std::map<size_t, std::string> colors;	

	/** Map with all the windows created to plot the data */
	std::vector<mrpt::gui::CDisplayWindowPlots*> winMap;
	std::vector<mrpt::gui::CDisplayWindowPlotsPtr> winMapPro;

	/** Get the resolution of the display screen to resize windows */
	void GetDesktopResolution(int& horizontal, int& vertical);

	/** Updates the display for each chamber, ActiveChamber and temperature */
	void updatePlot();
	void addToPlot(mrpt::slam::CObservationGasSensorsPtr &obs);

};
#endif

/** @moos_TODO
*/