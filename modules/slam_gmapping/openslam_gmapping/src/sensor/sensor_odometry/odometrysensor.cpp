//#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_odometry/odometrysensor.h"

namespace GMapping{

OdometrySensor::OdometrySensor(const std::string& name, bool ideal): Sensor(name){ m_ideal=ideal;}


};

