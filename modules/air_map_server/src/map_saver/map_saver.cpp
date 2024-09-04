#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "cyber/time/duration.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/air_map_server/include/map_saver.h"
#include <iostream>
#include "modules/air_map_server/include/map_io.h"
#include "modules/air_map_server/include/map_mode.h"
#include <string>
using namespace apollo::MapSaver;



// /* Map output part */
// struct SaveParameters
// {
//   std::string map_file_name{""};
//   std::string image_format{""};
//   double free_thresh{0.0};
//   double occupied_thresh{0.0};
//   MapMode mode{MapMode::Trinary};
// };



// bool saveMapToFile(
//   const apollo::akman::OccupancyGrid& map,
//   const SaveParameters & save_parameters);



void PrintSaveRequest(SAVE_REQUEST& request) {
    AINFO << "---- Save Request Details ----";

    // 打印 map_topic
    if (request.has_map_topic()) {
        AINFO << "Map Topic: " << request.map_topic();
    } else {
        AINFO << "Map Topic: Not specified (default will be used)";
    }

    // 打印 save_path
    if (request.has_save_path()) {
        AINFO << "Save Path: " << request.save_path();
    } else {
        AINFO << "Save Path: Not specified (default will be used)";
    }

    // 打印 img_format
    if (request.has_img_format()) {
        AINFO << "Image Format: " << request.img_format();
    } else {
        AINFO << "Image Format: Not specified (default will be used)";
    }

    // 打印 map_mode
    if (request.has_map_mode()) {
        AINFO << "Map Mode: " << request.map_mode();
    } else {
        AINFO << "Map Mode: Not specified (default will be used)";
    }

    // 打印 free_thresh
    if (request.has_free_thresh()) {
        AINFO << "Free Threshold: " << request.free_thresh();
    } else {
        AINFO << "Free Threshold: Not specified (default will be used)";
    }

    // 打印 occupied_thresh
    if (request.has_occupied_thresh()) {
        AINFO << "Occupied Threshold: " << request.occupied_thresh();
    } else {
        AINFO << "Occupied Threshold: Not specified (default will be used)";
    }

    AINFO << "---- End of Save Request Details ----";
}



void MapSaverComponent::onMapMsgRead(
    const std::shared_ptr<apollo::akman::OccupancyGrid>& map_msg)
{
    if(is_need_to_save_){

        
        apollo::akman::OccupancyGrid map_copy = *map_msg;
        
        
        apollo::MapServer::SaveParameters save_params_fomater;

        save_params_fomater.map_file_name = save_params_.save_path();
        save_params_fomater.image_format = save_params_.img_format();
        save_params_fomater.mode = apollo::MapServer::map_mode_from_string(save_params_.map_mode());
        save_params_fomater.free_thresh = save_params_.free_thresh();
        save_params_fomater.occupied_thresh = save_params_.occupied_thresh();

        if(apollo::MapServer::saveMapToFile(map_copy,save_params_fomater)){
            is_need_to_save_ = false;
            
        }
        
    }
}




void MapSaverComponent::onSaveRequestReceived(const std::shared_ptr<SAVE_REQUEST>& request,
                   const std::shared_ptr<SAVE_RESPONSE>& response)
{

        save_params_ = *request;
        PrintSaveRequest(save_params_);

        // Check and set default map topic
        if (save_params_.map_topic().empty()) {
            save_params_.set_map_topic("map");  // 设置默认的地图话题
            AWARN << "Map channel unspecified. Map messages will be read from 'map' channel";
        }

        // Check and set default save path
        if (save_params_.save_path().empty()) {
            save_params_.set_save_path("/tmp/map");
            AWARN << "Save path unspecified. Setting it to default path: '/tmp/map'";
        }

        // Check and set default image format
        if (save_params_.img_format().empty()) {
            save_params_.set_img_format("pgm"); 
            AWARN << "Image format unspecified. Setting it to default format: 'pgm'";
        } else {
            // Check if the provided image format is valid (pgm, png, bmp)
            std::string format = save_params_.img_format();
            if (format != "pgm" && format != "png" && format != "bmp") {
                save_params_.set_img_format("pgm"); 
                AWARN << "Invalid image format. Setting it to default format: 'pgm'";
            }
        }

        // Check and set default map mode
        if (save_params_.map_mode().empty()) {
            save_params_.set_map_mode("trinary");  // 设置默认的地图模式
            AWARN << "Map mode unspecified. Setting it to default mode: 'trinary'";
        } else {
            // Check if the provided map mode is valid (trinary, scaled, raw)
            std::string mode = save_params_.map_mode();
            if (mode != "trinary" && mode != "scaled" && mode != "raw") {
                save_params_.set_map_mode("trinary");  // 不支持的模式时，设置为默认的 trinary
                AWARN << "Invalid map mode. Setting it to default mode: 'trinary'";
            }
        }

        // Check and set default free threshold
        if (save_params_.free_thresh() == 0.0) {
            save_params_.set_free_thresh(free_thresh_);  // 使用预定义的默认值
            AWARN << "Free threshold unspecified. Setting it to default value: " 
                << free_thresh_;
        }

        // Check and set default occupied threshold
        if (save_params_.occupied_thresh() == 0.0) {
            save_params_.set_occupied_thresh(occupied_thresh_);  // 使用预定义的默认值
            AWARN << "Occupied threshold unspecified. Setting it to default value: "
                << occupied_thresh_;
        }


        if (!map_reader_) {
            map_reader_ = node_->CreateReader<apollo::akman::OccupancyGrid>(
                save_params_.map_topic(),
                std::bind(&MapSaverComponent::onMapMsgRead, this, std::placeholders::_1));
        }    
        
        is_need_to_save_ = true;

}
    





bool  MapSaverComponent::Init(){


    node_ = apollo::cyber::CreateNode("@@@ map_saver_node");


    save_service_ = node_->CreateService<SAVE_REQUEST, SAVE_RESPONSE>(
        "map_save_service",  // 服务名称
        std::bind(&MapSaverComponent::onSaveRequestReceived, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    AINFO << "Service save_map_service created successfully.";
    return true;
}
    
CYBER_REGISTER_COMPONENT(MapSaverComponent);
