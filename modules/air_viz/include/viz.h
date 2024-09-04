#ifndef __AIR_VIZ_HPP__
#define __AIR_VIZ_HPP__

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
#include <atomic>

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
#include "modules/air_map_server/proto/save_msg.pb.h"
#include "modules/air_map_server/proto/map_service_msg.pb.h"
#include <cstdlib>  // 用于设置和获取环境变量
#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <vector>
#define LOAD_MAP_REQUEST apollo::modules::air_map_server::map_service_msg::proto::load_map_request
#define LOAD_MAP_RESPONSE apollo::modules::air_map_server::map_service_msg::proto::load_map_response

namespace apollo {
namespace AirViz {

class VizComponent final : public cyber::Component<> {
public:
    VizComponent();
    ~VizComponent();

    bool Init() override;

protected:

    void RoutineTask();


    void InitializeOpenGL();
    void RenderLoop();
    static void DrawElementCallBack();
    void DrawElements();
    void DrawMap();
    void DrawVehicle();
    // void DrawWalls(); 
    void SetupGlobalFrame();
    void DrawCoordinateAxes();
    // void ProcessMap(const vector<vector<int8_t>>& map_mat, int erosion_size = 1);
    
    void KeyboardDown(unsigned char key, int x, int y);
    void KeyboardUp(unsigned char key, int x, int y);

    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);
    void MouseWheel(int button, int dir, int x, int y);

    bool RequestMapData();
    void OnMapDataReceived(const std::shared_ptr<LOAD_MAP_RESPONSE>& response);
    void CreateMapTexture();
    
    std::shared_ptr<std::thread> render_thread_;  // 渲染线程成员变量
    std::shared_ptr<std::thread> routine_task_thread_;
    std::shared_ptr<cyber::Client<LOAD_MAP_REQUEST, LOAD_MAP_RESPONSE>> map_service_client_;
    std::shared_ptr<apollo::akman::OccupancyGrid> map_;  

    int width_;
    int height_;
    double resolution_;
    
    const double free_thresh_ = 0.25;
    const double occupied_thresh_ = 0.35;

    std::vector<std::vector<int8_t>> map_mat_;
    GLuint map_display_list_ = 0;
    GLuint texture_id_ = 0;  // 纹理ID

    


    cyber::Duration map_request_period_ = cyber::Duration(5.0);
    std::atomic<bool> is_map_requested_{false};
    std::atomic<bool> is_map_cached_{false};
    

//---Pose Init GUI---------------------------------------------------------------//
    std::atomic<bool> is_space_pressed_{false};

    float original_eye_x_;
    float original_eye_y_;
    float original_eye_z_;
    float original_center_x_;
    float original_center_y_;
    float original_center_z_;
    float original_up_x_;
    float original_up_y_;
    float original_up_z_;
//-------------------------------------------------------------------------------//











    std::mutex map_access_mutex_;  



    

    static VizComponent* g_instance_;  // 全局实例指针
    Eigen::Matrix4f global_frame_; // 全局坐标系

    float rotation_x_ = 0.0f;
    float rotation_y_ = 0.0f;
    float zoom_ = 1.0f;
    float pan_x_ = 0.0f;  // 新增平移变量
    float pan_y_ = 0.0f;  // 新增平移变量

    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    bool is_dragging_ = false;
    bool is_panning_ = false;  // 新增平移状态变量

    char* old_env_;

    const std::string get_map_service_name_{"apollo/air_map_server/get_map_service"};
    const std::string load_map_service_name_{"apollo/air_map_server/load_map_service"};
    const std::string map_channel_name_{"apollo/air_map_server/map"};
    const std::string map_file_path_{"my_map.map.yaml"};

};

CYBER_REGISTER_COMPONENT(VizComponent);

}  // namespace AirViz
}  // namespace apollo

#endif  // __AIR_VIZ_HPP__
