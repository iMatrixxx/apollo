#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "modules/slam_gmapping/openslam_gmapping/include/utils/point.h"
#include "modules/slam_gmapping/openslam_gmapping/include/utils/stat.h"
#include "modules/slam_gmapping/openslam_gmapping/include/utils/macro_params.h"

namespace  GMapping { 

struct MotionModel{
	OrientedPoint drawFromMotion(const OrientedPoint& p, double linearMove, double angularMove) const;
	OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
	Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const;
	double srr, str, srt, stt;
};

};

#endif
