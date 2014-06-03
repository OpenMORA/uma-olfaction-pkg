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


#include "GasMapModel_Dynamic.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;


GasMapModel_Dynamic::GasMapModel_Dynamic(const mrpt::utils::CConfigFileBase &config_source)
	: GasMapModelBase(config_source)  // This will automatically create all maps and load common parameters.
{
	// We only need to load here specific parameters of this derived class, not loaded in the base
	// Load specific parameters for each map:
	const int num_maps = config_source.read_int("","num_gas_maps",0,true);

	for (int i=0;i<num_maps;i++)
	{
		//! @moos_param map_sequence_folder [GasMapModel_Dynamic] The path to the folder containing the bitmap sequence.
		const std::string sDir = config_source.read_string("",format("map%i_sequence_folder",i),"",true);
		//! @moos_param map_filename_prefix [GasMapModel_Dynamic] The prefix used for the filnames
		const std::string sPrefix = config_source.read_string("",format("map%i_filename_prefix",i),"",true);
		//! @moos_param map_frameRate [GasMapModel_Dynamic] The frame rate at wich process the bitmaps (frames/seconds)
		double fR = config_source.read_double("",format("map%i_frameRate",i),0,true);
		
		printf("[GasMapModel_Dynamic] map%i_sequence_folder = %s\n",i,sDir.c_str());
		printf("[GasMapModel_Dynamic] map%i_filename_prefix = %s\n",i,sPrefix.c_str());
		printf("[GasMapModel_Dynamic] map%i_frameRate = %f\n",i,fR);

		m_all_gas_grid_maps[i].first.sequence_folder = sDir;
		m_all_gas_grid_maps[i].first.filename_prefix = sPrefix;
		m_all_gas_grid_maps[i].first.frameRate = fR;
		m_all_gas_grid_maps[i].first.simulated_time = 0;		//At start-up
	}
}

/** Implementation of GasMapModelBase::simulateTimeInterval() for one individual map.
  * \exception std::exception Upon any error.
  */
void GasMapModel_Dynamic::impl_simulateTimeInterval(
	const double     At,
	InfoPerMap & info,
	mrpt::slam::CGasConcentrationGridMap2D & gasMap
	)
{
	//printf("[GasMapModel_Dynamic] At = %.2f sec\n",At);
	//printf("[GasMapModel_Dynamic] fR = %.2f\n",info.frameRate);
	//printf("[GasMapModel_Dynamic] actual_frame = %u\n",info.actual_frame);

	// Update the simulation time
	info.simulated_time = info.simulated_time + (At); //sec
	//printf("[GasMapModel_Dynamic] simulation_time = %.2f sec\n",info.simulated_time);

	unsigned int new_frame = 1+(unsigned int)(floor(info.simulated_time * info.frameRate));
	//printf("[GasMapModel_Dynamic] Candidate new_frame = %u\n",new_frame);

	//Candidate next frame
	std::string sFil = format("%s/%s%u.png",info.sequence_folder.c_str(), info.filename_prefix.c_str(), new_frame);
	//printf("[GasMapModel_Dynamic] Candidate new_frame_path = %s\n",sFil.c_str());

	CImage img;
	if (!img.loadFromFile(sFil, 0 /* force grayscale */))
	{
		cout << "[GasMapModel_Dynamic] Error loading bitmap image for gas map: " << sFil.c_str() << endl; 
		cout << "[GasMapModel_Dynamic] STOPPING DYNAMIC GAS MAP" << endl;
		//throw std::runtime_error(mrpt::format("Error loading bitmap image for gas map: '%s'",sFil.c_str() ));
	}
	else
	{
		ASSERT_(!img.isColor())

		//flip gasmap to fit the gridmap
		img.flipVertical();
		//fill map from image
		const double K = 1./255.;
		const unsigned int w=img.getWidth();
		const unsigned int h=img.getHeight();
		for (unsigned int y=0;y<h;y++)
		{
			unsigned int yy=h-1-y;
			const unsigned char * ptr_img = img.get_unsafe(0,yy,0);
			for (unsigned int xx=0;xx<w;xx++)
			{
				mrpt::slam::TRandomFieldCell * cell = gasMap.cellByIndex(xx,yy);
				cell->kf_mean = (*ptr_img++) * K;
			}
		}		
	}
}

