#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <thread>
#include <time.h>
#include <functional>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Dense"


#include "modules/transform/buffer.h"
#include "third_party/tf2/include/tf2/LinearMath/Transform.h"
#include "modules/slam_gmapping/utils.h"


#include "cyber/component/component.h"
#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"

#include "modules/common_msgs/akman_msgs/entropy.pb.h"
#include "modules/common_msgs/akman_msgs/occupancy_grid.pb.h"
#include "modules/common_msgs/akman_msgs/map_meta_data.pb.h"
// #include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"
#include "modules/common_msgs/akman_msgs/pose_stamped.pb.h"
#include "modules/common_msgs/akman_msgs/laser_scan.pb.h"
#include "modules/common_msgs/monitor_msgs/monitor_log.pb.h"
#include "modules/slam_gmapping/proto/slam_gmapping_conf.pb.h"

#include "modules/slam_gmapping/openslam_gmapping/include/gridfastslam/gridslamprocessor.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_base/sensor.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_range/rangesensor.h"
#include "modules/slam_gmapping/openslam_gmapping/include/sensor/sensor_odometry/odometrysensor.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace slamg_mapping {
                                                                              
using apollo::transform::Buffer;

struct StampedTransform {
  double timestamp = 0.0;  // in second
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SlamGmappingComponent final : public cyber::Component<> {
 public:
  SlamGmappingComponent();
  ~SlamGmappingComponent();

  std::string Name() const;

  void startLiveSlam();
  void publishTransform();
  void laserCallback(const std::shared_ptr<apollo::akman::LaserScan>& scan);
  void publishLoop(double transform_publish_period);
  bool initMapper(const apollo::akman::LaserScan& scan);

  apollo::slam_gmapping::SlamGmappingConfig config_;

 private:

  bool Init() override;

  Buffer* tf2_buffer_ = Buffer::Instance();
  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;
  

  ::apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  std::shared_ptr<cyber::Writer<apollo::akman::Etropy>> entropy_writer_;
  std::shared_ptr<cyber::Writer<apollo::akman::OccupancyGrid>> sst_writer_;
  std::shared_ptr<cyber::Writer<apollo::akman::MapMetaData>> sstm_writer_;
  std::shared_ptr<cyber::Reader<apollo::akman::LaserScan>> laser_reader_;

  // std::shared_ptr<apollo::transform::Buffer> buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tfl_;

  GMapping::GridSlamProcessor* gsp_;
  GMapping::RangeSensor* gsp_laser_;

  // The angles in the laser, going from -x to x (adjustment is made to get the laser between
  // symmetrical bounds as that's what gmapping expects)
  std::vector<double> laser_angles_;
  // The pose, in the original laser frame, of the corresponding centered laser with z facing up
  apollo::akman::PoseStamped centered_laser_pose_;

  // Depending on the order of the elements in the scan and the orientation of the scan frame,
  // We might need to change the order of the scan
  bool do_reverse_range_;
  unsigned int gsp_laser_beam_count_;
  GMapping::OdometrySensor* gsp_odom_;

  bool got_first_scan_;

  bool got_map_;
  apollo::akman::OccupancyGrid map_;

  double map_update_interval_;
  tf2::Transform map_to_odom_;
  std::mutex map_to_odom_mutex_;
  std::mutex map_mutex_;

  int laser_count_;
  int throttle_scans_;

  std::shared_ptr<std::thread> transform_thread_;

  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;

    void updateMap(const std::shared_ptr<apollo::akman::LaserScan>& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const double& t);
    bool initMapper(const std::shared_ptr<apollo::akman::LaserScan>& scan);
    bool addScan(const std::shared_ptr<apollo::akman::LaserScan>& scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;

    double last_map_update_;

};

CYBER_REGISTER_COMPONENT(SlamGmappingComponent)

}  // namespace canbus
}  // namespace apollo