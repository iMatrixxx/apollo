#ifndef SENSORLOG_H
#define SENSORLOG_H

#include <list>
#include <istream>
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_base/sensorreading.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_odometry/odometrysensor.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_range/rangesensor.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_odometry/odometryreading.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_range/rangereading.h"
#include "modules/slam_gmapping/openslam_gmapping/include/log/configuration.h"

namespace GMapping {

class SensorLog : public std::list<SensorReading*>{
	public:
		SensorLog(const SensorMap&);
		~SensorLog();
		std::istream& load(std::istream& is);
		OrientedPoint boundingBox(double& xmin, double& ymin, double& xmax, double& ymax) const;
	protected:
		const SensorMap& m_sensorMap;
		OdometryReading* parseOdometry(std::istream& is, const OdometrySensor* ) const;
		RangeReading* parseRange(std::istream& is, const RangeSensor* ) const;
};

};

#endif
