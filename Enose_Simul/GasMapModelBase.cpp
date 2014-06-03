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


#include "GasMapModels.h" // Include all, for the class factory

#include <mrpt/system/string_utils.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/opengl.h>
#include <mrpt/random.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::random;


// Class factory:
GasMapModelBase * GasMapModelBase::createFromName(
	const std::string &className,
	const mrpt::utils::CConfigFileBase &config_source
	)
{
	if (strCmpI(className, "GasMapModel_Static"))
		return new GasMapModel_Static(config_source);
	else if (strCmpI(className, "GasMapModel_Dynamic"))
		return new GasMapModel_Dynamic(config_source);
	else return NULL;
}


/** Constructor: call with \a m_ini (member of any CMapirMOOSApp) as parameter from your MOOS module.
*/
GasMapModelBase::GasMapModelBase(const mrpt::utils::CConfigFileBase &config_source)
{
	// Load # of maps:
	//! @moos_param num_gas_maps Number of gasMaps to simulate
	const int num_maps = config_source.read_int("","num_gas_maps",0,true);

	//! @moos_param voltage_max All sensor output concentrations data are normalized scaled by this value
	m_voltage_max = config_source.read_double("","voltage_max",5.0);

	// Load & create each map: 
	m_all_gas_grid_maps.resize(num_maps);
	printf("[GasMapModelBase] Creating %i GasMap(s)\n",num_maps);	

	for (int i=0;i<num_maps;i++)
	{
		// Create map:
		mrpt::slam::CGasConcentrationGridMap2DPtr &gasmap_ptr = m_all_gas_grid_maps[i].second;
		gasmap_ptr = mrpt::slam::CGasConcentrationGridMap2D::Create();
		mrpt::slam::CGasConcentrationGridMap2D & gasmap = *gasmap_ptr;

		// Extra info field:
		InfoPerMap &info = m_all_gas_grid_maps[i].first;

		// Set all params from .moos file:
		// ------------------------------------------------------------
		std::stringstream ss;
		//! @moos_param map_gastype	The ID of the gasMap. Usually is the model of a gas sensor (0x2620)
		const std::string sType = trim( config_source.read_string("",format("map%i_gastype",i),"",true) );
		if (sType.size()>=2 && sType[0]=='0' && sType[1]=='x')
				sscanf(sType.c_str(),"%X",&info.gas_type);
		else	sscanf(sType.c_str(),"%u",&info.gas_type);		
		printf("[GasMapModelBase] map%i_gastype = %X\n",i,info.gas_type);	

		// Initialization type:
		//! @moos_param map_init_from Possible values: "bitmap", "uniform"
		const std::string sInitType = trim( config_source.read_string("",format("map%i_init_from",i),"",true) );
		printf("[GasMapModelBase] map%i_init_from = %s\n",i,sInitType.c_str());

		if (strCmpI("bitmap",sInitType))
		{
			//! @moos_param map_x0 [bitmap] Position on the simulation reference system to position the gasmap (m)
			const double x0 = config_source.read_double("",format("map%i_x0",i),0,true);
			//! @moos_param map_y0 [bitmap] Position on the simulation reference system to position the gasmap (m)
			const double y0 = config_source.read_double("",format("map%i_y0",i),0,true);
			//! @moos_param map_resolution [bitmap] (meters/pixel)
			const double res = config_source.read_double("",format("map%i_resolution",i),0,true);
			//! @moos_param map_bitmap [bitmap] Path to the initial bitmap image used to initialize the gas concentration
			const string sFil = config_source.read_string("",format("map%i_bitmap",i),"",true);
			printf("[GasMapModelBase] map%i_initial_bitmap = %s\n",i,sFil.c_str());

			CImage img;
			if (!img.loadFromFile(sFil, 0 /* force grayscale */))
			{
				cout << "Error loading bitmap image for gas map: " << sFil.c_str() << endl; 
				throw std::runtime_error(mrpt::format("Error loading bitmap image for gas map: '%s'",sFil.c_str() ));
			}
			ASSERT_(!img.isColor())

			//flip gasmap to fit the gridmap
			img.flipVertical();

			const unsigned int w=img.getWidth();
			const unsigned int h=img.getHeight();

			gasmap.setSize(x0,x0+res*w, y0,y0+res*h, res);
			const double K = 1./255.;
			for (unsigned int y=0;y<h;y++)
			{
				unsigned int yy=h-1-y;
				const unsigned char * ptr_img = img.get_unsafe(0,yy,0);
				for (unsigned int xx=0;xx<w;xx++)
				{
					mrpt::slam::TRandomFieldCell * cell = gasmap.cellByIndex(xx,yy);
					cell->kf_mean = (*ptr_img++) * K;
				}
			}
		}
		else if (strCmpI("uniform",sInitType))
		{
			//! @moos_param map_x0 [uniform] Coordinates of the initial point (meters)
			const double x0 = config_source.read_double("",format("map%i_x0",i),0,true);
			//! @moos_param map_y0 [uniform] Coordinates of the initial point (meters)
			const double y0 = config_source.read_double("",format("map%i_y0",i),0,true);
			//! @moos_param map_x1 [uniform] Coordinates of the final point (meters)
			const double x1 = config_source.read_double("",format("map%i_x1",i),0,true);
			//! @moos_param map_y1 [uniform] Coordinates of the final point (meters)
			const double y1 = config_source.read_double("",format("map%i_y1",i),0,true);
			//! @moos_param map_resolution [uniform] meters/pixel
			const double res = config_source.read_double("",format("map%i_resolution",i),0,true);
			//! @moos_param map_value [uniform] Value to asign to the uniform gasMap
			const double value = config_source.read_double("",format("map%i_value",i),0,true);

			// Set size:
			gasmap.setSize(x0,x1,y0,y1,res);

			// Fill all cells with default value:
			mrpt::slam::TRandomFieldCell  defCell;
			defCell.kf_mean = value;
			gasmap.fill(defCell);
		}
		else throw std::runtime_error("Invalid value found in 'map%i_init_from'");


		// Derived classes will continue reading from the .moos file any other extra params at their contructors.
	}
}

/** Simulate the passage of time in all the gas maps.
  * \param[in] At Time increment, in seconds.
  *  This method internally call impl_simulateTimeInterval() for each individual map.
  * \exception std::exception Upon any error.
  */
void GasMapModelBase::simulateTimeInterval(const double At)
{
	ASSERT_ABOVEEQ_(At,0)

	if (At==0 || m_all_gas_grid_maps.empty())
		return; // Nothing to do.

	for (size_t i=0;i<m_all_gas_grid_maps.size();i++)
	{
		InfoPerMap &info = m_all_gas_grid_maps[i].first;
		CGasConcentrationGridMap2D & gasMap = *(m_all_gas_grid_maps[i].second);

		this->impl_simulateTimeInterval(At,info,gasMap);
	}

}


/** Obtain a 3D view of the current state of all gas maps:
  */
void GasMapModelBase::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &glObj) const
{
	glObj->clear();

	// TODO Javi: represent maps as a plan gridmap with transparency & some special colormap,
	//   each map (each "chemical") can be in a different color, colormaps, etc.
	//   New options can be created and loaded from the .moos file at start up.
	std::string m_3D_map_mode = "default";
	TColormap color_map_type = mrpt::utils::cmJET;

	for (size_t i=0;i<m_all_gas_grid_maps.size();i++)
	{
		const CGasConcentrationGridMap2D & gasmap = *(m_all_gas_grid_maps[i].second);
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::CSetOfObjects::Create();

		if (strCmpI(m_3D_map_mode,"default"))
		{
			const size_t w = gasmap.getSizeX();
			const size_t h = gasmap.getSizeY();
			CImage img(w,h, CH_RGB);
			CImage img_transparency(w,h, CH_GRAY);


			for (size_t y=0;y<h;y++)
			{
				const size_t yy=h-1-y;
				unsigned char * ptr_img = img.get_unsafe(0,yy,0);
				unsigned char * ptr_img_transp = img_transparency.get_unsafe(0,yy,0);
				for (size_t xx=0;xx<w;xx++)
				{
					const mrpt::slam::TRandomFieldCell * cell = gasmap.cellByIndex(xx,yy);
					const float in_gray = cell->kf_mean;

					// RGB color:
					mrpt::utils::TColorf col;
					mrpt::utils::colormap(color_map_type, in_gray, col.R,col.G,col.B);
					(*ptr_img++) = static_cast<unsigned char>(col.R * 255.f);
					(*ptr_img++) = static_cast<unsigned char>(col.G * 255.f);
					(*ptr_img++) = static_cast<unsigned char>(col.B * 255.f);

					// Transparency: (There're many possibilities here...)
					(*ptr_img_transp++) = static_cast<unsigned char>(col.R * 255.f);
				}
			}

			mrpt::opengl::CTexturedPlanePtr gl_pl = mrpt::opengl::CTexturedPlane::Create();
			gl_pl->setPlaneCorners( gasmap.getXMin(), gasmap.getXMax(), gasmap.getYMin(), gasmap.getYMax() );
			gl_pl->assignImage_fast(img,img_transparency);  // _fast: move semantics

			obj->insert(gl_pl);

		}
//		else if (mrpt::utils::StrCmpI(m_3D_map_mode,"XXX"))
//		{
//			// obj...
//		}

		// Append to output:
		glObj->insert(obj);
	}

}


/** Simulate all the noisy & low-passed readings from a set of enoses.
  */
void GasMapModelBase::simulateReadings(
	const mrpt::poses::CPose2D &robotPose2D,
	std::vector<InfoPerENose> &eNoses,
	const mrpt::system::TTimeStamp &timestamp,
	const double  At_since_last_seconds,
	const std::string  &sensorLabel,
	mrpt::slam::CObservationGasSensors  &obs
	)
{	
	obs.timestamp = timestamp;
	obs.sensorLabel = sensorLabel;

	for (size_t nEnose=0;nEnose<eNoses.size();nEnose++)
	{
		InfoPerENose & enose = eNoses[nEnose];

		mrpt::slam::CObservationGasSensors::TObservationENose eNoseData;
		eNoseData.hasTemperature = false;
		eNoseData.isActive = false;
		eNoseData.eNosePoseOnTheRobot.x = enose.x;
		eNoseData.eNosePoseOnTheRobot.y = enose.y;
		eNoseData.eNosePoseOnTheRobot.z = 0.0;

		// Position of this eNose:
		mrpt::math::TPoint2D  sensorPose2D;
		robotPose2D.composePoint(
			enose.x,enose.y,  // local
			sensorPose2D.x,sensorPose2D.y // global
			);

		// For the first iteration:
		const size_t nMaps = m_all_gas_grid_maps.size();
		if (enose.last_real_concentrations.size()!=nMaps) enose.last_real_concentrations.resize(nMaps);
		if (enose.last_outputs.size()!=nMaps) enose.last_outputs.resize(nMaps);

		// Simulate readings in each gas map:
		for (size_t nMap=0; nMap<nMaps;nMap++)
		{
			// get map data:
			const mrpt::slam::CGasConcentrationGridMap2DPtr &gasmap_ptr = m_all_gas_grid_maps[nMap].second;
			const mrpt::slam::CGasConcentrationGridMap2D & gasmap = *gasmap_ptr;
			InfoPerMap &mapinfo = m_all_gas_grid_maps[nMap].first;

			// Simulate noise-free reading:
			const double real_gas_concentration  = getRealConcentration(gasmap,sensorPose2D);
			const double real_last_concentration = enose.last_real_concentrations[nMap];

			// Rise or decay?
			//const double tau = (real_gas_concentration>=real_last_concentration) ? enose.tau_rise : enose.tau_decay;
			const double tau = (real_gas_concentration >= enose.last_outputs[nMap]) ? enose.tau_rise : enose.tau_decay;
			
			// 1st order model:
			//            B(z)             1
			// H(z) = ------------ = ---------------
			//            A(Z)         1 - a*z^{-1}
			//
			// Time impulse response: ...
			//
			// 1st order implementation!

			const double last_output = enose.last_outputs[nMap];
			const double no_noise_reading = last_output+(1.-std::exp(-At_since_last_seconds / tau))*(real_gas_concentration-last_output);

			// Add noises:
			const double final_reading = no_noise_reading + randomGenerator.drawGaussian1D(enose.noise_offset, enose.noise_std );
			const double final_reading_volt = std::max(0.,std::min(1.,final_reading)) *m_voltage_max;

			// save data to eNose observation:
			eNoseData.sensorTypes.push_back(mapinfo.gas_type);
			eNoseData.readingsVoltage.push_back(final_reading_volt);

#if 1
			cout << "eNose #"<< nEnose << " Map #"<< nMap << "  -> Real: " << real_gas_concentration << " Sensed [0-1]: " << final_reading << endl;
#endif
			// Write values for next iter:
			enose.last_real_concentrations[nMap] = real_gas_concentration;
			enose.last_outputs[nMap] = final_reading;
		}

		// Done with this eNose
		obs.m_readings.push_back( eNoseData );
	}
}


/** Static method that returns the REAL instantaneous concentration at any point of a map (or 0 if it's out of the map)
  */
double GasMapModelBase::getRealConcentration(
	const mrpt::slam::CGasConcentrationGridMap2D & gasmap,
	const mrpt::math::TPoint2D &pt
	)
{
	// TODO JAVI: This can be done better with linear interpolation of neighbors, etc.
	//  See methods: cellByIndex(), x2idx(), y2idx()...

	const mrpt::slam::TRandomFieldCell * cell_xy = gasmap.cellByPos(pt.x,pt.y);
	if (cell_xy)
	{
		return cell_xy->kf_mean;
	}
	else
	{
		return 0;
	}
}


/** save the current state of all gas maps:
  */
void GasMapModelBase::getAsMatrix(mrpt::math::CMatrix &mat) const
{
	bool first = true;
	

	for (size_t i=0;i<m_all_gas_grid_maps.size();i++)
	{
		const CGasConcentrationGridMap2D & gasmap = *(m_all_gas_grid_maps[i].second);
		
		const size_t w = gasmap.getSizeX();
		const size_t h = gasmap.getSizeY();
		mrpt::math::CMatrix M(h,w);
		try
		{
			for (size_t i=0; i<h; i++)
			{
				for (size_t j=0; j<w; j++)
				{
					M(i,j) = gasmap.cellByIndex(j,i)->kf_mean;
				}
			}

			// Append to output:
			if (first)
			{	
				mat.resize(h,w);
				mat.fill(0.0);
				first = false;
			}
			mat += M;
		}
		catch (mrpt::utils::exception e)
		{
			cout << "exception: " << e.what() << endl;
		}
	}

}
