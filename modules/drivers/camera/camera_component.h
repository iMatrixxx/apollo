/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

//---------------------------
#include <thread>
#include <fcntl.h>
#include <opencv2/opencv.hpp>

#include "cyber/cyber.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

//#include "modules/drivers/camera/usb_cam.h"

//zhxf 20240806 阿克曼小车
#include "third_party/camera_library/astracamera/include/OpenNI.h"
#include "modules/drivers/camera/astracamera/device_listener.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/camera/astracamera/ob_frame_listener.h"


namespace apollo {
namespace drivers {
namespace camera {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Image;
using apollo::drivers::camera::config::Config;

class CameraComponent : public Component<> {
 public:
  bool Init() override;
  ~CameraComponent();

 private:
  void run();
  

  /*********************************************************************** 
  std::shared_ptr<Writer<Image>> writer_ = nullptr;
  std::shared_ptr<Writer<Image>> raw_writer_ = nullptr;
  std::unique_ptr<UsbCam> camera_device_;
  std::shared_ptr<Config> camera_config_;
  CameraImagePtr raw_image_ = nullptr;
  CameraImagePtr raw_image_for_compress_ = nullptr;
  std::vector<std::shared_ptr<Image>> pb_image_buffer_;
  std::vector<std::shared_ptr<Image>> raw_image_buffer_;
  uint32_t spin_rate_ = 200;
  uint32_t device_wait_ = 2000;
  int index_ = 0;
  int buffer_size_ = 16;
  const int32_t MAX_IMAGE_SIZE = 20 * 1024 * 1024;
  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
  *************************************************************************/

  //---------------------------------------------------------------------------
  void onDeviceConnected(const openni::DeviceInfo* device_info);
  void onDeviceDisconnected(const openni::DeviceInfo* device_info);
  void setupConfig();
  void setupFrameCallback();
  void setupPublishers();
  void setupVideoMode();
  void setupDevices();
  void startStreams();
  void setDepthColorSync(bool data);
  void setImageRegistrationMode(bool data);
  void stopStreams();
  void clean();
  void onNewFrameCallback(const openni::VideoFrameRef& frame,
                                      const stream_index_pair& stream_index); 
  uint8_t* matToBytes(cv::Mat image);
  std::future<void> async_result_;
  std::shared_ptr<Config> camera_config_;
  std::atomic<bool> running_ = {false};
  
  std::atomic_bool is_alive_{false};
  std::shared_ptr<openni::Device> device_ = nullptr;
  openni::DeviceInfo device_info_;
  std::unique_ptr<DeviceListener> device_listener_ = nullptr;
  std::string serial_number_;
  std::string device_type_;
  std::string device_uri_;
  std::shared_ptr<cyber::Timer> check_connection_timer_;
  std::atomic_bool device_connected_{false};
  size_t number_of_devices_;
  std::unordered_map<std::string, openni::DeviceInfo> connected_devices_;
  long reconnection_delay_ = 0;
  bool is_first_connection_ = true;

  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, int> unit_step_size_;
  std::map<stream_index_pair, openni::PixelFormat> format_;
  std::map<stream_index_pair, int> image_format_;
  std::map<stream_index_pair, std::string> encoding_;

  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::string camera_link_frame_id_;

  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, int> fps_;
  std::map<stream_index_pair, bool> enable_;
  std::map<stream_index_pair, bool> stream_started_;
  std::map<stream_index_pair, FrameCallbackFunction> stream_frame_callback_;
  std::map<stream_index_pair, cv::Mat> images_;
  std::map<stream_index_pair, std::shared_ptr<Writer<Image>>> image_writer_;

  std::map<stream_index_pair, std::vector<openni::VideoMode>> supported_video_modes_;
  std::map<stream_index_pair, openni::VideoMode> stream_video_mode_;
  std::map<stream_index_pair, std::shared_ptr<OBFrameListener>> stream_frame_listener_;
  std::map<stream_index_pair, std::shared_ptr<openni::VideoStream>> streams_;

};

CYBER_REGISTER_COMPONENT(CameraComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
