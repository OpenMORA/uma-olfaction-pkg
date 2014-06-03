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

#pragma once

#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/utils/CConfigFileBase.h>

/** Parameters & state of each eNose device */
struct InfoPerENose
{
	InfoPerENose() :
		x(0),y(0),
		noise_offset(0), noise_std(0.1),
		tau_rise(0.2),tau_decay(5),
		last_real_concentrations(),
		last_outputs()
	{}

	double  x,y; //!< Position wrt the robot (2D)
	double  noise_offset;  //!< Offset to add to all readings (typ=0)
	double  noise_std;     //!< Additive gaussian noise (standard deviation)  (readings saturate to [0,1])

	double  tau_rise, tau_decay;  //!< First order time constants
	std::vector<double> last_real_concentrations,last_outputs;          //!< (one per gas map) Values needed to implement the 1st order model.
};


/** The base virtual class for all ground truth gas map models.
  *  Several 2D grids can be stored simultaneously, one for each type of gas.
  *
  *   The class is also a class factory thru the method: GasMapModelBase::createFromName()
  *
  *  How to derive a new class:
  *    - In your new constructor, after the ":" call this base constructor, so parameters can be loaded from .moos file, etc.
  *    - Implement all virtual methods.
  *    - Add an #include at the end of the "GasMapModels.h" file.
  *    - Update the class factory \a createFromName()
  */
class GasMapModelBase
{
public:
	/** Class factory: construct an object from a textual name of its class (e.g. "GasMapModel_Static")
	  *  String comps. are case insensitive.
	  *  Return NULL if class name is unknown.
	  *  \a config_source is the \a m_ini member of any CMapirMOOSApp, which is used
	  *  to automatically load all the params from the .moos file and create the maps.
	  */
	static GasMapModelBase * createFromName(
		const std::string &className,
		const mrpt::utils::CConfigFileBase &config_source
		);

	// Dtor required in polymorphic classes to assure clean delete!
	~GasMapModelBase() {}

	/** Simulate the passage of time in all the gas maps.
	  * \param[in] At Time increment, in seconds.
	  *  This method internally call impl_simulateTimeInterval() for each individual map.
	  * \exception std::exception Upon any error.
	  */
	void simulateTimeInterval(const double At);

	/** Simulate all the noisy & low-passed readings from a set of enoses.
	  */
	void simulateReadings(
		const mrpt::poses::CPose2D &robotPose2D,
		std::vector<InfoPerENose> &eNoses,
		const mrpt::system::TTimeStamp &timestamp,
		const double  At_since_last_seconds,
		const std::string  &sensorLabel,
		mrpt::slam::CObservationGasSensors  &out_obs
		);

	/** Obtain a 3D view of the current state of all gas maps:
	  */
	void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &glObj) const;

	/** Obtain the current state of all gas maps as Matrices:
	  */
	void getAsMatrix(mrpt::math::CMatrix &mat) const;

	/** Static method that returns the REAL instantaneous concentration at any point of a map (or 0 if it's out of the map)
	  */
	static double getRealConcentration(
		const mrpt::slam::CGasConcentrationGridMap2D & gasmap,
		const mrpt::math::TPoint2D &pt
		);


protected:
	struct InfoPerMap
	{
		/**  0: doesn't matter.
		  *  0xABCD : The gas corresponding to a given sensor code, following the convention of \a sensorTypes in:
		  *   http://reference.mrpt.org/svn/structmrpt_1_1slam_1_1_c_observation_gas_sensors_1_1_t_observation_e_nose.html
		  */
		unsigned int gas_type;

		/** [GasMapModel_Dynamic] The frame rate at which the sequence of gasmap images should be loaded */
		double frameRate;
		/** [GasMapModel_Dynamic] The directory path containing the sequence of gridmaps. */
		std::string sequence_folder;
		/** [GasMapModel_Dynamic] The prefix of the frame squence */
		std::string filename_prefix;
		/** [GasMapModel_Dynamic] The total simulation time (seconds), used to calculate the frame to display*/
		double simulated_time;
	};


	/** Constructor: call with \a m_ini (member of any CMapirMOOSApp) as parameter from your MOOS module. */
	GasMapModelBase(const mrpt::utils::CConfigFileBase &config_source);


	/** Implementation of GasMapModelBase::simulateTimeInterval() for one individual map.
	  * \exception std::exception Upon any error.
	  */
	virtual void impl_simulateTimeInterval(
		const double     At,
		InfoPerMap & info,
		mrpt::slam::CGasConcentrationGridMap2D & gasMap
		) = 0;


	std::vector<std::pair<InfoPerMap,mrpt::slam::CGasConcentrationGridMap2DPtr> > m_all_gas_grid_maps;
private:


	double m_voltage_max; //!< Sensor output are between 0 and this value.

}; // end of GasMapModelBase

