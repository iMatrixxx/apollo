#ifndef __MAP_SERVER__MAP_IO_HPP__
#define __MAP_SERVER__MAP_IO_HPP__

#include <string>
#include <vector>
#include "modules/common_msgs/akman_msgs/occupancy_grid.pb.h"
#include "modules/common_msgs/akman_msgs/map_meta_data.pb.h"
#include "yaml-cpp/yaml.h"
#include <libgen.h> 
#include "modules/air_map_server/include/map_mode.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "third_party/tf2/include/tf2/LinearMath/Transform.h"


/* Map input part */
namespace apollo {
namespace MapServer {
                              


static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;


struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum
{
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;




LoadParameters loadMapYaml(const std::string & yaml_filename);


void loadMapFromFile(
  const LoadParameters & load_parameters,
  apollo::akman::OccupancyGrid & map);




LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  apollo::akman::OccupancyGrid  & map);




/* Map output part */
struct SaveParameters
{
  std::string map_file_name{""};
  std::string image_format{""};
  double free_thresh{0.0};
  double occupied_thresh{0.0};
  MapMode mode{MapMode::Trinary};
};



bool saveMapToFile(
  const apollo::akman::OccupancyGrid& map,
  const SaveParameters & save_parameters);





}  // namespace MapServer
}  // namespace apollo
#endif