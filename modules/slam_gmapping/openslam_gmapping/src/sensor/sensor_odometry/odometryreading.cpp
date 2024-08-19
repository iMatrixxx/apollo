// #include <gmapping/sensor/sensor_odometry/odometryreading.h>
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_odometry/odometryreading.h"

namespace GMapping{

OdometryReading::OdometryReading(const OdometrySensor* odo, double time):
	SensorReading(odo,time){}

};

