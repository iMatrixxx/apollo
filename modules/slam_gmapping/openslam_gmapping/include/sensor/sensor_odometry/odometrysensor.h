#ifndef ODOMETRYSENSOR_H
#define ODOMETRYSENSOR_H

#include <string>
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_base/sensor.h"
namespace GMapping{

class OdometrySensor: public Sensor{
	public:
		OdometrySensor(const std::string& name, bool ideal=false);
		inline bool isIdeal() const { return m_ideal; }
	protected:
		bool m_ideal;	
};

};

#endif

