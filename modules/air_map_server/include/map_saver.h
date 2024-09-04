#ifndef __MAP_SERVER__MAP_SAVER_HPP__
#define __MAP_SERVER__MAP_SAVER_HPP__


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
#include "modules/air_map_server/proto/save_msg.pb.h" //message middleware
#include "modules/common/util/util.h"
#include "modules/air_map_server/include/map_io.h"
#include "modules/air_map_server/include/map_mode.h"
#define SAVE_REQUEST apollo::modules::air_map_server::save_msg::proto::save_request
#define SAVE_RESPONSE apollo::modules::air_map_server::save_msg::proto::save_response


namespace apollo {
namespace MapSaver {
                                                                              

class MapSaverComponent final : public cyber::Component<> {
 public:
    ~MapSaverComponent() override = default;

    bool Init();


 protected:

    cyber::Duration save_map_timeout_ = cyber::Duration(2.0);
    cyber::Timer timer_;
    
    std::shared_ptr<cyber::Service<SAVE_REQUEST, SAVE_RESPONSE>> save_service_;
    std::shared_ptr<cyber::Reader<apollo::akman::OccupancyGrid>> map_reader_;
   

    void onSaveRequestReceived(const std::shared_ptr<SAVE_REQUEST>&,
                               const std::shared_ptr<SAVE_RESPONSE>&);
    
    void onMapMsgRead(const std::shared_ptr<apollo::akman::OccupancyGrid>&);

    


    bool is_need_to_save_ = false;
    SAVE_REQUEST save_params_;


    std::string map_topic_ = "/OccupancyGrid";
    std::string save_path_ = "./";
    std::string img_format_ = "png";
    std::string map_mode_ = "trinary";
    double free_thresh_ = 0.65;
    double occupied_thresh_ = 0.25;

}; 






CYBER_REGISTER_COMPONENT(MapSaverComponent)
}  // namespace MapSaver
}  // namespace apollo
#endif
