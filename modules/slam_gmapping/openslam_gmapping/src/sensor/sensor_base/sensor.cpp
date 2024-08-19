#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_base/sensor.h"

namespace GMapping{

Sensor::Sensor(const std::string& name){
	m_name=name;
}

Sensor::~Sensor(){
}

};// end namespace
