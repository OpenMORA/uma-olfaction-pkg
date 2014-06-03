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


#include "GasMapModel_Static.h"


GasMapModel_Static::GasMapModel_Static(const mrpt::utils::CConfigFileBase &config_source)
	: GasMapModelBase(config_source)  // This will automatically create all maps.
{
	// We only need to load here specific parameters of this derived class, not loaded in the base
	// m_var = config_source.read_double(...)

}

/** Implementation of GasMapModelBase::simulateTimeInterval() for one individual map.
  * \exception std::exception Upon any error.
  */
void GasMapModel_Static::impl_simulateTimeInterval(
	const double     At,
	InfoPerMap & info,
	mrpt::slam::CGasConcentrationGridMap2D & gasMap
	)
{
	// Static model: Nothing to do, since the map doesn't change.
}

