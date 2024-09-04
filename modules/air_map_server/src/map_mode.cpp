#include "modules/air_map_server/include/map_mode.h"
#include <stdexcept>
#include <string>

namespace apollo::MapServer
{

const char * map_mode_to_string(MapMode map_mode)
{
  switch (map_mode) {
    case MapMode::Trinary:
      return "trinary";
    case MapMode::Scale:
      return "scale";
    case MapMode::Raw:
      return "raw";
    default:
      throw std::invalid_argument("map_mode");
  }
}

MapMode map_mode_from_string(std::string map_mode_name)
{
  for (auto & c : map_mode_name) {
    c = tolower(c);
  }

  if (map_mode_name == "scale") {
    return MapMode::Scale;
  } else if (map_mode_name == "raw") {
    return MapMode::Raw;
  } else if (map_mode_name == "trinary") {
    return MapMode::Trinary;
  } else {
    throw std::invalid_argument("map_mode_name");
  }
}


}  // namespace nav2_map_server
