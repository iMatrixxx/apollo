#include "cyber/cyber.h"
#include "modules/air_map_server/proto/save_msg.pb.h" //message middleware
#define SAVE_REQUEST apollo::modules::air_map_server::save_msg::proto::save_request
#define SAVE_RESPONSE apollo::modules::air_map_server::save_msg::proto::save_response



#include <iostream>
int main(int argc,char* argv[]){

    apollo::cyber::Init(argv[0]);
    std::string map_topic = "apollo/slam_gmapping/map";
    std::string save_path = "./my_map.map";
    std::string img_format = "pgm";
    std::string map_mode = "trinary";
    double free_thresh = 0.25;
    double occupied_thresh = 0.65;
    auto client_node = apollo::cyber::CreateNode("map_saver_client_node");
    auto client = client_node->CreateClient<SAVE_REQUEST,SAVE_RESPONSE> ("map_save_service");
    auto request = std::make_shared<SAVE_REQUEST>();

    request->set_map_topic(map_topic);
    request->set_save_path(save_path);
    request->set_img_format(img_format);
    request->set_map_mode(map_mode);
    request->set_free_thresh(free_thresh);
    request->set_occupied_thresh(occupied_thresh);


    auto response = client->SendRequest(request);   
    auto res = response->result();;

    
    
    apollo::cyber::WaitForShutdown();
    return 0;
}