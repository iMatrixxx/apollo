#include "modules/air_map_server/include/map_io.h"


namespace apollo::MapServer
{


// // === Map input part ===
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}



LoadParameters loadMapYaml(const std::string & yaml_filename)
{

  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;
  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }

  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }

  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");


  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = apollo::MapServer::map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  std::cout << "[DEBUG] [map_io]: resolution: " << load_parameters.resolution << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[0]: " << load_parameters.origin[0] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[1]: " << load_parameters.origin[1] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[2]: " << load_parameters.origin[2] << std::endl;
  std::cout << "[DEBUG] [map_io]: free_thresh: " << load_parameters.free_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: occupied_thresh: " << load_parameters.occupied_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: mode: " << map_mode_to_string(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_io]: negate: " << load_parameters.negate << std::endl;  //NOLINT

  return load_parameters;
}



void loadMapFromFile(
  const LoadParameters & load_parameters,
  apollo::akman::OccupancyGrid& msg)
{

  std::cout << "[INFO] [map_io]: Loading image_file: " << load_parameters.image_file_name << std::endl;
  cv::Mat img = cv::imread(load_parameters.image_file_name, cv::IMREAD_UNCHANGED);
  if (img.empty()) {
    throw std::runtime_error("Failed to load image file: " + load_parameters.image_file_name);
  }
  // Copy the image data into the map structure
  /*
  message OccupancyGrid {
      optional apollo.common.Header header = 1;
      optional apollo.akman.MapMetaData meta_data = 2;
      repeated int32 data = 3;  //地图从左上角[0,0]按行存储, data表示每个点被占用的概率[0,100], -1表示未知
  }
  message MapMetaData {
      optional double map_load_time = 1; // time when map was loaded
      optional double resolution    = 2; // resolution of the map [m/cell]
      optional uint32 width = 3;  // width of the map [cells]
      optional uint32 height = 4; // height of the map [cells]
      optional Pose   origin = 5; // 地图的原点，[m, m, rad],即网格[0,0]在真实世界的位置和朝向
  }
  message Pose {
      optional apollo.common.Point3D position = 1;
      optional apollo.common.Quaternion orientation = 2;
  }
  */
  msg.mutable_meta_data()->mutable_origin()->mutable_position()->set_x(load_parameters.origin[0]);
  msg.mutable_meta_data()->mutable_origin()->mutable_position()->set_y(load_parameters.origin[1]);
  msg.mutable_meta_data()->mutable_origin()->mutable_position()->set_z(0);

  tf2::Quaternion q;
  q.setRPY(0,0,load_parameters.origin[2]);
  msg.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qx(q.getX());
  msg.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qy(q.getY());
  msg.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qz(q.getZ());
  msg.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qw(q.getW());


  msg.mutable_meta_data()->set_width(img.cols);
  msg.mutable_meta_data()->set_height(img.rows);
  msg.mutable_meta_data()->set_resolution(load_parameters.resolution);
  msg.mutable_data()->Resize(img.cols * img.rows, 0);

  // Allocate space to hold the data
  // Copy pixel data into the map structure

  int channels = img.channels();
  std::function<double(int, int)> computeShade;

  if (channels == 1) {
      computeShade = [&img](int y, int x) {
          uint8_t pixel = img.at<uint8_t>(y, x);
          return pixel / 255.0;
      };
  } else if (channels == 3) {
      computeShade = [&img](int y, int x) {
          cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
          return (pixel[0] + pixel[1] + pixel[2]) / 3.0 / 255.0;
      };
  } else if (channels == 4) {
      computeShade = [&img, &load_parameters](int y, int x) {
          cv::Vec4b pixel = img.at<cv::Vec4b>(y, x);
          double shade = (pixel[0] + pixel[1] + pixel[2]) / 3.0 / 255.0;
          if (load_parameters.mode == MapMode::Trinary) {
              shade = (shade * 3.0 + (255 - pixel[3]) / 255.0) / 4.0;
          }
          return shade;
      };
  } else {
      throw std::runtime_error("Unsupported number of image channels");
  }
            /*
                col0  col1  col2
                ----  ----  ---- X
            row0| 0     1     2  
            row1| 3     4     5                       
            row2| 6    (7)    8
                Y
            */
  for (int y = 0; y < img.rows; y++) {
    for (int x = 0; x < img.cols; x++) {

            double shade = computeShade(y, x);
            double occ = (load_parameters.negate ? shade : 1.0 - shade);

            int8_t map_cell;
            switch (load_parameters.mode) {
                case MapMode::Trinary:
                    if (load_parameters.occupied_thresh < occ) {
                        map_cell = apollo::MapServer::OCC_GRID_OCCUPIED;
                    } else if (occ < load_parameters.free_thresh) {
                        map_cell = apollo::MapServer::OCC_GRID_FREE;
                    } else {
                        map_cell = apollo::MapServer::OCC_GRID_UNKNOWN;
                    }
                    break;
                case MapMode::Scale:
                    if (channels == 4 && img.at<cv::Vec4b>(y, x)[3] != 255) { // 检查透明度
                        map_cell = apollo::MapServer::OCC_GRID_UNKNOWN;
                    } else if (load_parameters.occupied_thresh < occ) {
                        map_cell = apollo::MapServer::OCC_GRID_OCCUPIED;
                    } else if (occ < load_parameters.free_thresh) {
                        map_cell = apollo::MapServer::OCC_GRID_FREE;
                    } else {
                        map_cell = std::rint(
                          (occ - load_parameters.free_thresh) /
                          (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
                    }
                    break;
                case MapMode::Raw: {
                    double occ_percent = std::round(shade * 255);
                    if (apollo::MapServer::OCC_GRID_FREE <= occ_percent &&
                      occ_percent <= apollo::MapServer::OCC_GRID_OCCUPIED)
                    {
                      map_cell = static_cast<int8_t>(occ_percent);
                    } else {
                      map_cell = apollo::MapServer::OCC_GRID_UNKNOWN;
                    }
                    break;
                }
                default:
                    throw std::runtime_error("Invalid map mode");
            }

            int index = y*img.cols + x;
            if (index >= msg.data_size()) {
                std::cerr << "Index out of bounds: " << index << " >= " << msg.data_size() << std::endl;
                throw std::out_of_range("Index out of bounds");
            }
            msg.mutable_data()->Set(index, map_cell);

    }
  }
  msg.mutable_meta_data()->set_map_load_time(cyber::Time::Now().ToSecond());
  msg.mutable_header()->set_frame_id("map");
  msg.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToNanosecond());

  std::cout << "[DEBUG] [map_io]: Read map " << load_parameters.image_file_name 
            << ": " << msg.meta_data().width() << " X " << msg.meta_data().height() 
            << " map @ " << msg.meta_data().resolution() << " m/cell" << std::endl;
}


LOAD_MAP_STATUS loadMapFromYaml(
  const std::string&  yaml_file,
  apollo::akman::OccupancyGrid&  map)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io]: YAML file name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }
  std::cout << "[INFO] [map_io]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed processing YAML file " << yaml_file << " at position (" <<
      e.mark.line << ":" << e.mark.column << ") for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to parse map YAML loaded from file " << yaml_file <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  }

  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to load image file " << load_parameters.image_file_name <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}







// === Map output part ===
void checkSaveParameters(SaveParameters & save_parameters)
{
  // // Magick must me initialized before any activity with images
  // Magick::InitializeMagick(nullptr);

  if (save_parameters.map_file_name.empty()) {
    auto now = cyber::Time::Now();
    save_parameters.map_file_name = "map_" + std::to_string(static_cast<int>(now.ToSecond()));
    std::cout << "[map_io]: Map file unspecified. Map will be saved to "
          << save_parameters.map_file_name << " file"<<std::endl;
  }
  // Checking thresholds
  if (save_parameters.occupied_thresh == 0.0) {
    save_parameters.occupied_thresh = 0.65;
    std::cout << "[WARN] [map_io]: Occupied threshold unspecified. Setting it to default value: " <<
      save_parameters.occupied_thresh << std::endl;
  }
  if (save_parameters.free_thresh == 0.0) {
    save_parameters.free_thresh = 0.25;
    std::cout << "[WARN] [map_io]: Free threshold unspecified. Setting it to default value: " <<
      save_parameters.free_thresh << std::endl;
  }
  if (1.0 < save_parameters.occupied_thresh) {
    std::cerr << "[ERROR] [map_io]: Threshold_occupied must be 1.0 or less" << std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.free_thresh < 0.0) {
    std::cerr << "[ERROR] [map_io]: Free threshold must be 0.0 or greater" << std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
    std::cerr << "[ERROR] [map_io]: Threshold_free must be smaller than threshold_occupied" <<
      std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }

  // Checking image format
  if (save_parameters.image_format == "") {
    save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
    std::cout << "[WARN] [map_io]: Image format unspecified. Setting it to: " <<
      save_parameters.image_format << std::endl;
  }

  std::transform(
    save_parameters.image_format.begin(),
    save_parameters.image_format.end(),
    save_parameters.image_format.begin(),
    [](unsigned char c) {return std::tolower(c);});

  const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
  if (
    std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
    BLESSED_FORMATS.end())
  {
    std::stringstream ss;
    bool first = true;
    for (auto & format_name : BLESSED_FORMATS) {
      if (!first) {
        ss << ", ";
      }
      ss << "'" << format_name << "'";
      first = false;
    }
    std::cout <<
      "[WARN] [map_io]: Requested image format '" << save_parameters.image_format <<
      "' is not one of the recommended formats: " << ss.str() << std::endl;
  }
  const std::string FALLBACK_FORMAT = "png";


  // 尝试保存一个小的测试图像以检查格式是否可写
  try {
      std::string test_file_name = "test." + save_parameters.image_format;
      cv::Mat test_img = cv::Mat::zeros(1, 1, CV_8UC1); // 创建一个1x1的黑色图像

      // 使用指定格式保存测试图像
      if (!cv::imwrite(test_file_name, test_img)) {
          AWARN << "[map_io]: Format '" << save_parameters.image_format
                << "' is not writable. Using '" << FALLBACK_FORMAT << "' instead.";
          save_parameters.image_format = FALLBACK_FORMAT;
      } else {
          // 删除测试文件
          std::remove(test_file_name.c_str());
      }
  } catch (const cv::Exception &e) {
      AWARN << "[map_io]: Format '" << save_parameters.image_format 
            << "' is not usable. Using '" << FALLBACK_FORMAT << "' instead: " 
            << e.what();
      save_parameters.image_format = FALLBACK_FORMAT;
  }
}

  // Copy the image data into the map structure
  /*
  message OccupancyGrid {
      optional apollo.common.Header header = 1;
      optional apollo.akman.MapMetaData meta_data = 2;
      repeated int32 data = 3;  //地图从左上角[0,0]按行存储, data表示每个点被占用的概率[0,100], -1表示未知
  }
  message MapMetaData {
      optional double map_load_time = 1; // time when map was loaded
      optional double resolution    = 2; // resolution of the map [m/cell]
      optional uint32 width = 3;  // width of the map [cells]
      optional uint32 height = 4; // height of the map [cells]
      optional Pose   origin = 5; // 地图的原点，[m, m, rad],即网格[0,0]在真实世界的位置和朝向
  }
  message Pose {
      optional apollo.common.Point3D position = 1;
      optional apollo.common.Quaternion orientation = 2;
  }
  */
void tryWriteMapToFile(
  const apollo::akman::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{


  std::cout <<
    "[INFO] [map_io]: Received a " << map.meta_data().width()  << " X " 
                                   << map.meta_data().height() << " map @ " 
                                   << map.meta_data().resolution() << " m/pix" 
                                   << std::endl;

  std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
  
  {
    cv::Mat image(map.meta_data().height(), map.meta_data().width(), CV_8UC1, cv::Scalar(127)); // 初始化为灰色 (127)

    // 根据模式设置阈值
    int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
    int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);


    for (int y = 0; y < map.meta_data().height(); y++) {
      for (int x = 0; x < map.meta_data().width(); x++) {

          int8_t map_cell = map.data(map.meta_data().width() * (map.meta_data().height() - y - 1) + x);
          uint8_t pixel_value = 127; // 默认灰色

          switch (save_parameters.mode) {
                case MapMode::Trinary:
                      if (map_cell < 0 || 100 < map_cell) {
                          pixel_value = 205; // 未知区域
                      } else if (map_cell <= free_thresh_int) {
                          pixel_value = 254; // 空闲区域
                      } else if (occupied_thresh_int <= map_cell) {
                          pixel_value = 0;   // 占用区域
                      } else {
                          pixel_value = 205; // 未知区域
                      }
                      break;
                  case MapMode::Scale:
                      if (map_cell < 0 || 100 < map_cell) {
                          pixel_value = 127; // 未知区域，设为中性灰色
                      } else {
                          pixel_value = static_cast<uint8_t>((100.0 - map_cell) * 2.55); // 0-100 映射到 0-255
                      }
                      break;
                  case MapMode::Raw:
                      if (map_cell < 0 || 100 < map_cell) {
                          pixel_value = 255; // 未知区域
                      } else {
                          pixel_value = static_cast<uint8_t>(map_cell / 100.0 * 255.0); // 直接映射
                      }
                      break;
                  default:
                      std::cerr << "[ERROR] [map_io]: Map mode should be Trinary, Scale or Raw" << std::endl;
                      throw std::runtime_error("Invalid map mode");
              }

              image.at<uint8_t>(y, x) = pixel_value;
          }
      }

      std::cout << "[INFO] [map_io]: Writing map occupancy data to " << mapdatafile << std::endl;
      // image.write(mapdatafile);
      cv::imwrite(mapdatafile,image);
  }

  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    apollo::common::Quaternion orientation = map.meta_data().origin().orientation();
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.qx(), orientation.qy(), orientation.qz(), orientation.qw()));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    const int file_name_index = mapdatafile.find_last_of("/\\");
    std::string image_name = mapdatafile.substr(file_name_index + 1);

    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << image_name;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
    e << YAML::Key << "resolution" << YAML::Value << map.meta_data().resolution();
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq 
      << map.meta_data().origin().position().x() 
      << map.meta_data().origin().position().x()
      << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
    e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

    if (!e.good()) {
      std::cout <<
        "[WARN] [map_io]: YAML writer failed with an error " << e.GetLastError() <<
        ". The map metadata may be invalid." << std::endl;
    }

    std::cout << "[INFO] [map_io]: Writing map metadata to " << mapmetadatafile << std::endl;
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  std::cout << "[INFO] [map_io]: Map saved" << std::endl;
}

bool saveMapToFile(
  const apollo::akman::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  // Local copy of SaveParameters that might be modified by checkSaveParameters()
  SaveParameters save_parameters_loc = save_parameters;


  try {
    // Checking map parameters for consistency
    checkSaveParameters(save_parameters_loc);
    tryWriteMapToFile(map, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  //apollo::MapServer