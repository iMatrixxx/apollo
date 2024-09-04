
#ifndef __MAP_SERVER__MAP_MODE_HPP__
#define __MAP_SERVER__MAP_MODE_HPP__

#include <string>
#include <vector>

namespace apollo {
namespace MapServer {
/**
 * @enum nav2_map_server::MapMode
 * @brief Describes the relation between image pixel values and map occupancy
 * status (0-100; -1). Lightness refers to the mean of a given pixel's RGB
 * channels on a scale from 0 to 255.
 */
enum class MapMode
{
  /**
   * Together with associated threshold values (occupied and free):
   *   lightness >= occupied threshold - Occupied (100)
   *             ... (anything in between) - Unknown (-1)
   *    lightness <= free threshold - Free (0)
   */
  Trinary,
  /**
   * Together with associated threshold values (occupied and free):
   *   alpha < 1.0 - Unknown (-1)
   *   lightness >= occ_th - Occupied (100)
   *             ... (linearly interpolate to)
   *   lightness <= free_th - Free (0)
   */
  Scale,
  /**
   * Lightness = 0 - Free (0)
   *          ... (linearly interpolate to)
   * Lightness = 100 - Occupied (100)
   * Lightness >= 101 - Unknown
   */
  Raw,
};


const char * map_mode_to_string(MapMode map_mode);
MapMode map_mode_from_string(std::string map_mode_name);

}  // namespace MapServer
}  // namespace apollo


#endif  // __MAP_SERVER__MAP_MODE_HPP__
