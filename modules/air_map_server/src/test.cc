#include "cyber/cyber.h"

#include <iostream>

#include "modules/air_map_server/include/map_io.h"
#include "modules/air_map_server/include/map_mode.h"
#include "modules/air_map_server/proto/save_msg.pb.h" //message middleware
#include "modules/air_map_server/proto/map_service_msg.pb.h" //message middleware

#define GET_MAP_REQUEST apollo::modules::air_map_server::map_service_msg::proto::get_map_request
#define GET_MAP_RESPONSE apollo::modules::air_map_server::map_service_msg::proto::get_map_response
#define LOAD_MAP_REQUEST apollo::modules::air_map_server::map_service_msg::proto::load_map_request
#define LOAD_MAP_RESPONSE apollo::modules::air_map_server::map_service_msg::proto::load_map_response
                
int main(int argc,char* argv[]){

    apollo::cyber::Init(argv[0]);

    std::string path = "my_map.map.yaml";

//==============IO TEST =============================================//
    // apollo::akman::OccupancyGrid map;
    // apollo::MapServer::loadMapFromYaml(path,map);

    // apollo::MapServer::SaveParameters params;
    // params.map_file_name = "helloWorld";
    // params.image_format = "pgm";
    // apollo::MapServer::saveMapToFile(map,params);


//==============LOAD SERVICE TEST =============================================//

    const std::string load_map_service_name_{"apollo/air_map_server/load_map_service"};
    auto client_node = apollo::cyber::CreateNode("testNode");
    auto client = client_node->CreateClient<LOAD_MAP_REQUEST,LOAD_MAP_RESPONSE> (load_map_service_name_);
    auto request = std::make_shared<LOAD_MAP_REQUEST>();
    request->set_map_url(path);
    auto response = client->SendRequest(request);   
    AINFO<< response->map().meta_data().resolution();


//==============GET SERVICE TEST =============================================//



    // const std::string get_map_service_name_{"apollo/air_map_server/get_map_service"};
    // // auto client_node = apollo::cyber::CreateNode("testNode");
    // auto client2 = client_node->CreateClient<GET_MAP_REQUEST,GET_MAP_RESPONSE> (get_map_service_name_);
    // auto request2 = std::make_shared<GET_MAP_REQUEST>();


    // auto response2 = client2->SendRequest(request2);

    // int ans = response2->get_result();
    // AINFO<<"#####"<<response2->get_result();
    
    // if(ans==1){                         
    //     AINFO<< response2->map().meta_data().resolution();
    // }


//==============SUBSCRIPTION TEST =============================================//

    // const std::string get_map_service_name_{"apollo/air_map_server/get_map_service"};
    // const std::string load_map_service_name_{"apollo/air_map_server/load_map_service"};
    // const std::string map_channel_name_{"apollo/air_map_server/map"};
    // auto client_node = apollo::cyber::CreateNode("testNode");
    // auto client = client_node->CreateClient<LOAD_MAP_REQUEST,LOAD_MAP_RESPONSE> (load_map_service_name_);
    // auto request = std::make_shared<LOAD_MAP_REQUEST>();
    // request->set_map_url("/apollo/modules/air_map_server/map/tmp.yaml");
    // auto response = client->SendRequest(request);   
    // AINFO<< response->load_status();






    apollo::cyber::WaitForShutdown();
    return 0;
}