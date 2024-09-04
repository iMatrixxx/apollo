#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "cyber/time/duration.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/air_map_server/include/map_server.h"
#include "modules/air_map_server/include/map_io.h"
#include <iostream>


using namespace apollo::MapServer;

MapServerComponent::~MapServerComponent(){

    if(map_pub_thread_){
        map_pub_thread_->join();
    }
}

void MapServerComponent::RoutineTask() {

    // cyber::Rate r(1.0 / map_pub_freq_);
    while (cyber::OK()) {
        {
            std::lock_guard<std::mutex> guard(map_mutex_);
            if(!map_available_)continue;
            map_.mutable_header()->set_timestamp_sec(cyber::Clock::Now().ToSecond());
            map_.mutable_header()->set_frame_id(frame_id_);   
            occ_writer_->Write(map_);
        }
        cyber::Duration(0.1).Sleep();
        // r.Sleep();
    }
}



bool  MapServerComponent::onGetMapRequestReceived(
    const std::shared_ptr<GET_MAP_REQUEST>& request,
    const std::shared_ptr<GET_MAP_RESPONSE>& response){
    
    std::lock_guard<std::mutex> guard(map_mutex_);
    if(!map_available_){
        response->set_get_result(-1);
        return false;
    }
    response->set_get_result(1);
    map_.mutable_header()->set_timestamp_sec(cyber::Clock::Now().ToSecond());
    map_.mutable_header()->set_frame_id(frame_id_);    
    response->mutable_map()->CopyFrom(map_);
    return true;                          
}


bool MapServerComponent::onLoadMapRequestReceived(
    const std::shared_ptr<LOAD_MAP_REQUEST>& request,
    const std::shared_ptr<LOAD_MAP_RESPONSE>& response){
    AINFO<< "LoadMap request received, Handling LoadMap request";
    return MapServerComponent::loadMapResponseFromYaml(request->map_url(),response);
}


bool  MapServerComponent::Init(){

    node_ = apollo::cyber::CreateNode("###map_server_node");

    // occ_service_ = node_->CreateService<GET_MAP_REQUEST, GET_MAP_RESPONSE>(
    //     get_map_service_name_, 
    //     [this](const std::shared_ptr<GET_MAP_REQUEST>& request,
    //            std::shared_ptr<GET_MAP_RESPONSE>& response) {
    //         onGetMapRequestReceived(request, response);
    //     });

    load_map_service_ = node_->CreateService<LOAD_MAP_REQUEST, LOAD_MAP_RESPONSE>(
        load_map_service_name_, 
        [this](const std::shared_ptr<LOAD_MAP_REQUEST>& request,
               std::shared_ptr<LOAD_MAP_RESPONSE>& response) {
            try {
                if(onLoadMapRequestReceived(request, response)){
                    map_available_ = true;
                } 
            } catch (const std::exception& e) {
                AERROR << "Exception in onLoadMapRequestReceived: " << e.what();
            }
        });

    occ_writer_ = node_->CreateWriter<apollo::akman::OccupancyGrid>(map_channel_name_);


    map_pub_thread_ = std::make_shared<std::thread>
            (std::bind(&MapServerComponent::RoutineTask, this));

    map_pub_freq_ = 10.;
    frame_id_ = "map";
    return true;
}
    

bool MapServerComponent::loadMapResponseFromYaml(
    const std::string & yaml_file,
    const std::shared_ptr<LOAD_MAP_RESPONSE>& response){
    

    switch (loadMapFromYaml(yaml_file, map_)) {
        case MAP_DOES_NOT_EXIST:
            response->set_load_status(LOAD_MAP_RESPONSE::RESULT_MAP_DOES_NOT_EXIST);
            return false;
        case INVALID_MAP_METADATA:
            response->set_load_status(LOAD_MAP_RESPONSE::RESULT_INVALID_MAP_METADATA);
            return false;
        case INVALID_MAP_DATA:
            response->set_load_status(LOAD_MAP_RESPONSE::RESULT_INVALID_MAP_DATA);
            return false;
        case LOAD_MAP_SUCCESS:
            response->mutable_map()->CopyFrom(map_);
            response->set_load_status(LOAD_MAP_RESPONSE::RESULT_SUCCESS);
            return true;
    }
    return true;    
}





CYBER_REGISTER_COMPONENT(MapServerComponent)
