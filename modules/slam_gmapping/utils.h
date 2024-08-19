#pragma once

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/common_msgs/akman_msgs/pose_stamped.pb.h"
#include "modules/common_msgs/akman_msgs/entropy.pb.h"
#include "modules/common_msgs/akman_msgs/occupancy_grid.pb.h"
#include "modules/common_msgs/akman_msgs/map_meta_data.pb.h"
#include "modules/transform/buffer.h"
#include "third_party/tf2/include/tf2/LinearMath/Transform.h"



namespace apollo {
namespace slamg_mapping {
void TransformPointStamped(
  const apollo::akman::PointStamped & t_in,
  apollo::akman::PointStamped & t_out,
  const apollo::transform::TransformStamped & transform);
void TransformPoseStamped(
  const apollo::akman::PoseStamped & t_in,
  apollo::akman::PoseStamped & t_out,
  const apollo::transform::TransformStamped & transform);

double getYaw(const apollo::common::Quaternion& q);
}
}