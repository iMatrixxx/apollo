#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <istream>
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_base/sensor.h"

namespace GMapping {

class Configuration{
	public:
		virtual ~Configuration();
		virtual SensorMap computeSensorMap() const=0;
};

};
#endif

