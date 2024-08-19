
#include "modules/slam_gmapping/utils.h"

namespace apollo {
namespace slamg_mapping {
double getYaw(const apollo::common::Quaternion& q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.qx() * q.qx();
  sqy = q.qy() * q.qy();
  sqz = q.qz() * q.qz();
  sqw = q.qw() * q.qw();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  // normalization added from urdfom_headers
  double sarg = -2 * (q.qx() * q.qz() - q.qw() * q.qy()) / (sqx + sqy + sqz + sqw);

  if (sarg <= -0.99999) {
    yaw = -2 * std::atan2(q.qy(), q.qx());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q.qy(), q.qx());
  } else {
    yaw = atan2(2 * (q.qx() * q.qy() + q.qw() * q.qz()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}


void TransformPoseStamped(
  const apollo::akman::PoseStamped & t_in,
  apollo::akman::PoseStamped & t_out,
  const apollo::transform::TransformStamped & transform)
{
    // 从 PoseStamped 中提取位置和方向，并转换为 Eigen 类型
    Eigen::Vector3d position_in(t_in.pose().position().x(),     
                                t_in.pose().position().y(), 
                                t_in.pose().position().z());
    Eigen::Quaterniond orientation_in(t_in.pose().orientation().qw(), t_in.pose().orientation().qx(),
                                       t_in.pose().orientation().qy(), t_in.pose().orientation().qz());

    // 从 TransformStamped 中提取变换矩阵
    Eigen::Matrix4d transform_matrix;
    Eigen::Quaterniond rotation(transform.transform().rotation().qw(), 
                                transform.transform().rotation().qx(),
                                transform.transform().rotation().qy(), 
                                transform.transform().rotation().qz());
    Eigen::Vector3d translation(transform.transform().translation().x(), 
                                transform.transform().translation().y(),
                                transform.transform().translation().z());

    transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform_matrix.block<3, 1>(0, 3) = translation;
    transform_matrix.row(3) << 0, 0, 0, 1;

    // 应用变换
    Eigen::Vector4d homogeneous_position_in(position_in.x(), position_in.y(), position_in.z(), 1.0);
    Eigen::Vector4d homogeneous_position_out = transform_matrix * homogeneous_position_in;

    Eigen::Vector3d position_out(homogeneous_position_out.x(), homogeneous_position_out.y(), homogeneous_position_out.z());
    Eigen::Quaterniond orientation_out = rotation * orientation_in;

    // 更新 PoseStamped 输出
    t_out.mutable_pose()->mutable_position()->set_x(position_out.x());
    t_out.mutable_pose()->mutable_position()->set_y(position_out.y());
    t_out.mutable_pose()->mutable_position()->set_z(position_out.z());
    t_out.mutable_pose()->mutable_orientation()->set_qx(orientation_out.x());
    t_out.mutable_pose()->mutable_orientation()->set_qy(orientation_out.y());
    t_out.mutable_pose()->mutable_orientation()->set_qz(orientation_out.z());
    t_out.mutable_pose()->mutable_orientation()->set_qw(orientation_out.w());
    t_out.mutable_header()->set_timestamp_sec(transform.header().timestamp_sec());
    t_out.mutable_header()->set_frame_id(transform.header().frame_id());
}

void TransformPointStamped(
  const apollo::akman::PointStamped & t_in,
  apollo::akman::PointStamped & t_out,
  const apollo::transform::TransformStamped & transform)
{
    // 提取并转换输入点
    Eigen::Vector3d point_in(t_in.point().x(), t_in.point().y(), t_in.point().z());

    // 提取并转换变换矩阵
    Eigen::Quaterniond rotation(transform.transform().rotation().qw(), 
                                transform.transform().rotation().qx(),
                                transform.transform().rotation().qy(), 
                                transform.transform().rotation().qz());
    Eigen::Vector3d translation(transform.transform().translation().x(), 
                                transform.transform().translation().y(),
                                transform.transform().translation().z());

    // 应用旋转和平移
    Eigen::Vector3d point_out = rotation * point_in + translation;

    // 设置输出点
    t_out.mutable_point()->set_x(point_out.x());
    t_out.mutable_point()->set_y(point_out.y());
    t_out.mutable_point()->set_z(point_out.z());
    t_out.mutable_header()->set_timestamp_sec(transform.header().timestamp_sec());
    t_out.mutable_header()->set_frame_id(transform.header().frame_id());
}

}
}