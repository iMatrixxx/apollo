#ifndef __MAP_SERVER__MAP_SERVER_HPP__
#define __MAP_SERVER__MAP_SERVER_HPP__


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
#include "cyber/component/component.h"
#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"
#include "cyber/common/util.h"
#include "cyber/service/service.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common_msgs/akman_msgs/entropy.pb.h"
#include "modules/common_msgs/akman_msgs/occupancy_grid.pb.h"
#include "modules/common_msgs/akman_msgs/map_meta_data.pb.h"
#include "modules/transform/transform_broadcaster.h"
#include "modules/common/util/util.h"

#include "modules/air_map_server/proto/save_msg.pb.h" //message middleware
#include "modules/air_map_server/proto/map_service_msg.pb.h" //message middleware

#define GET_MAP_REQUEST apollo::modules::air_map_server::map_service_msg::proto::get_map_request
#define GET_MAP_RESPONSE apollo::modules::air_map_server::map_service_msg::proto::get_map_response
#define LOAD_MAP_REQUEST apollo::modules::air_map_server::map_service_msg::proto::load_map_request
#define LOAD_MAP_RESPONSE apollo::modules::air_map_server::map_service_msg::proto::load_map_response
                

namespace apollo {
namespace MapServer {
              
                                                

class MapServerComponent final : public cyber::Component<> {

public:
    ~MapServerComponent();

    bool Init();


protected:
  void RoutineTask(); 
  
  bool loadMapResponseFromYaml(
    const std::string & yaml_file,
    const std::shared_ptr<LOAD_MAP_RESPONSE>& response);

  bool onGetMapRequestReceived(const std::shared_ptr<GET_MAP_REQUEST>& request,
                               const std::shared_ptr<GET_MAP_RESPONSE>& response);
                     

  bool onLoadMapRequestReceived(const std::shared_ptr<LOAD_MAP_REQUEST>& request,
                               const std::shared_ptr<LOAD_MAP_RESPONSE>& response); 

  const std::string get_map_service_name_{"apollo/air_map_server/get_map_service"};
  const std::string load_map_service_name_{"apollo/air_map_server/load_map_service"};
  const std::string map_channel_name_{"apollo/air_map_server/map"};


  std::shared_ptr<cyber::Service<GET_MAP_REQUEST,GET_MAP_RESPONSE>> occ_service_;
  std::shared_ptr<cyber::Service<LOAD_MAP_REQUEST, LOAD_MAP_RESPONSE>> load_map_service_;
  std::shared_ptr<cyber::Writer<apollo::akman::OccupancyGrid>> occ_writer_;

  std::string frame_id_;
  apollo::akman::OccupancyGrid map_;
  std::atomic<bool> map_available_ ={false};
  std::shared_ptr<std::thread> map_pub_thread_;
  std::mutex map_mutex_;
  double map_pub_freq_;
}; 






CYBER_REGISTER_COMPONENT(MapServerComponent)
}  // namespace MapServer
}  // namespace apollo
#endif
