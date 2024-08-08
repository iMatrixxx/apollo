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

#include "modules/drivers/camera/camera_component.h"

namespace apollo {
namespace drivers {
namespace camera {

/************************************************************************** 
bool CameraComponent::Init() {
  camera_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "UsbCam config: " << camera_config_->DebugString();

  camera_device_.reset(new UsbCam());
  camera_device_->init(camera_config_);
  raw_image_.reset(new CameraImage);
  raw_image_for_compress_.reset(new CameraImage);

  raw_image_->width = camera_config_->width();
  raw_image_->height = camera_config_->height();
  raw_image_->bytes_per_pixel = camera_config_->bytes_per_pixel();

  raw_image_for_compress_->width = camera_config_->width();
  raw_image_for_compress_->height = camera_config_->height();
  raw_image_for_compress_->bytes_per_pixel = camera_config_->bytes_per_pixel();

  if (camera_config_->pixel_format() == "yuyv" ||
      camera_config_->pixel_format() == "uyvy" ||
      camera_config_->pixel_format() == "yuvmono10") {
    raw_image_for_compress_->image_size =
      raw_image_for_compress_->width * raw_image_for_compress_->height * 2;
  } else if (camera_config_->pixel_format() == "mjpeg") {
    AINFO << "Disable sensor raw camera output with format mjpeg";
    raw_image_for_compress_->image_size = 0;
  } else if (camera_config_->pixel_format() == "rgb24") {
    raw_image_for_compress_->image_size =
        raw_image_for_compress_->width * raw_image_for_compress_->height * 3;
  } else {
    AERROR << "Wrong pixel fromat:" << camera_config_->pixel_format()
          << ",must be yuyv | uyvy | mjpeg | yuvmono10 | rgb24";
    return false;
  }

  if (raw_image_for_compress_->image_size == 0) {
    raw_image_for_compress_->image = nullptr;
  } else {
    raw_image_for_compress_->image = reinterpret_cast<char*>(
      calloc(raw_image_for_compress_->image_size, sizeof(char)));
  }

  device_wait_ = camera_config_->device_wait_ms();
  spin_rate_ = static_cast<uint32_t>((1.0 / camera_config_->spin_rate()) * 1e6);

  if (camera_config_->output_type() == YUYV) {
    raw_image_->image_size = raw_image_->width * raw_image_->height * 2;
  } else if (camera_config_->output_type() == RGB) {
    raw_image_->image_size = raw_image_->width * raw_image_->height * 3;
  }
  if (raw_image_->image_size > MAX_IMAGE_SIZE) {
    AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE
           << " bytes.";
    return false;
  }
  raw_image_->is_new = 0;
  // free memory in this struct desturctor
  raw_image_->image =
      reinterpret_cast<char*>(calloc(raw_image_->image_size, sizeof(char)));
  if (raw_image_->image == nullptr) {
    AERROR << "system calloc memory error, size:" << raw_image_->image_size;
    return false;
  }

  for (int i = 0; i < buffer_size_; ++i) {
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(camera_config_->frame_id());
    pb_image->set_width(raw_image_->width);
    pb_image->set_height(raw_image_->height);
    pb_image->mutable_data()->reserve(raw_image_->image_size);

    if (camera_config_->output_type() == YUYV) {
      pb_image->set_encoding("yuyv");
      pb_image->set_step(2 * raw_image_->width);
    } else if (camera_config_->output_type() == RGB) {
      pb_image->set_encoding("rgb8");
      pb_image->set_step(3 * raw_image_->width);
    }

    pb_image_buffer_.push_back(pb_image);

    auto raw_image = std::make_shared<Image>();
    raw_image->mutable_header()->set_frame_id(camera_config_->frame_id());
    raw_image->set_width(raw_image_for_compress_->width);
    raw_image->set_height(raw_image_for_compress_->height);
    raw_image->mutable_data()->reserve(raw_image_for_compress_->image_size);
    raw_image->set_encoding(camera_config_->pixel_format());

    raw_image_buffer_.push_back(raw_image);
  }

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());
  raw_writer_ = node_->CreateWriter<Image>(camera_config_->raw_channel_name());
  async_result_ = cyber::Async(&CameraComponent::run, this);
  return true;
}
**************************************************************************/

bool CameraComponent::Init() {
  
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  
  camera_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "UsbCam config: " << camera_config_->DebugString();

    auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    AERROR<<"Initialize failed\n%s\n"<<openni::OpenNI::getExtendedError();
    return false;
  }

  serial_number_ = camera_config_->serial_number();
  number_of_devices_ = camera_config_->number_of_devices();
  reconnection_delay_ = camera_config_->reconnection_delay();

  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  device_listener_ = std::make_unique<DeviceListener>(connected_cb, disconnected_cb);
  //async_result_ = cyber::Async(&CameraComponent::run, this);


  setupConfig();
  setupFrameCallback();
  setupDevices();
  setupPublishers();
  setupVideoMode();
  startStreams();
  return true;
}

void CameraComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    if (!device_connected_) {
      AERROR << "wait for device connect... ";
    }
  }
  return;
}

//--------------------------------------------------------------------------------------------------

void CameraComponent::onDeviceConnected(const openni::DeviceInfo* device_info) {
  AINFO << "Device connected: " << device_info->getName();
  if (device_info->getUri() == nullptr) {
    AERROR<<"Device connected: " << device_info->getName() << " uri is null";
    return;
  }
  auto device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
  if (device_sem == (void*)SEM_FAILED) {
    AERROR << "Failed to create semaphore";
    return;
  }
  AINFO << "Waiting for device to be ready";
  int ret = 0;
  ret = sem_wait(device_sem);
  if (ret != 0) {
    AERROR << "sem_wait failed";
    sem_close(device_sem);
    return;
  }
  if (!connected_devices_.count(device_info->getUri())) {
    auto device = std::make_shared<openni::Device>();
    AINFO << "Trying to open device: " << device_info->getUri();
    auto rc = device->open(device_info->getUri());
    if (rc != openni::STATUS_OK) {
      AERROR << "Failed to open device: " << device_info->getUri() << " error: "
                                                             << openni::OpenNI::getExtendedError();
      if (errno == EBUSY) {
        AERROR << "Device is already opened OR device is in use";
        connected_devices_[device_info->getUri()] = *device_info;
      }
    } else {
      char serial_number[64];
      int data_size = sizeof(serial_number);
      rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
      if (rc != openni::STATUS_OK) {
        AERROR << "Failed to get serial number: " << openni::OpenNI::getExtendedError();
      } else if (serial_number_.empty() || serial_number == serial_number_) {
        AINFO << "Device connected: " << device_info->getName() << " serial number: " << serial_number;
        device_uri_ = device_info->getUri();
        connected_devices_[device_uri_] = *device_info;
        device_ = device;
        if (!is_first_connection_) {
          std::this_thread::sleep_for(std::chrono::seconds(reconnection_delay_));
        }
        device_connected_ = true;
        //startDevice();
      }
    }
    if (!device_connected_) {
      device->close();
    }
    
  }
  AINFO << "Release device semaphore";
  sem_post(device_sem);
  AINFO << "Release device semaphore done";
  if (connected_devices_.size() == number_of_devices_) {
    AINFO << "All devices connected";
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void CameraComponent::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  if (device_uri_ == device_info->getUri()) {
    device_uri_.clear();
    if (device_) {
      device_->close();
      device_.reset();
    }

    AINFO <<"Device disconnected: " << device_info->getUri();
    device_connected_ = false;
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void CameraComponent::setupConfig() {
  stream_name_[DEPTH] = "depth";
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[DEPTH] = openni::PIXEL_FORMAT_DEPTH_1_MM;
  image_format_[DEPTH] = CV_16UC1;
  encoding_[DEPTH] = camera_config_->pixel_format_depth();
  width_[DEPTH] = camera_config_->depth_camera().width();
  height_[DEPTH] = camera_config_->depth_camera().height();
  fps_[DEPTH] = camera_config_->depth_camera().fps();
  enable_[DEPTH] = camera_config_->depth_camera().enable();

  stream_name_[COLOR] = "color";
  unit_step_size_[COLOR] = 3*sizeof(uint8_t);
  format_[COLOR] = openni::PIXEL_FORMAT_RGB888;
  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = camera_config_->pixel_format_color();;
  width_[COLOR] = camera_config_->color_camera().width();
  height_[COLOR] = camera_config_->color_camera().height();
  fps_[COLOR] = camera_config_->color_camera().fps();
  enable_[COLOR] = camera_config_->color_camera().enable();

  stream_name_[INFRA1] = "ir";
  unit_step_size_[INFRA1] = sizeof(uint8_t);
  format_[INFRA1] = openni::PIXEL_FORMAT_GRAY16;
  image_format_[INFRA1] = CV_16UC1;
  encoding_[INFRA1] = camera_config_->pixel_format_ir();;
  width_[INFRA1] = camera_config_->ir_camera().width();
  height_[INFRA1] = camera_config_->ir_camera().height();
  fps_[INFRA1] = camera_config_->ir_camera().fps();
  enable_[INFRA1] = camera_config_->ir_camera().enable();


  camera_link_frame_id_ = camera_config_->camera_name()+"_link";
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
    frame_id_[stream_index] = camera_config_->camera_name() + "_" + stream_name_[stream_index]+"_frame";
    optical_frame_id_[stream_index] =
        camera_config_->camera_name() + "_" + stream_name_[stream_index] + "_optical_frame";
    depth_aligned_frame_id_[stream_index] = camera_config_->camera_name() + "_" +"color"+"_optical_frame";

  }
}

void CameraComponent::setupFrameCallback() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    auto frame_callback = [this, stream_index = stream_index](const openni::VideoFrameRef& frame) {
      onNewFrameCallback(frame, stream_index);
    };
    stream_frame_callback_[stream_index] = frame_callback;
  }
}

void CameraComponent::onNewFrameCallback(const openni::VideoFrameRef& frame,
                                      const stream_index_pair& stream_index) {
  int width = frame.getWidth();
  int height = frame.getHeight();
  CHECK(images_.count(stream_index));
  auto& image = images_.at(stream_index);
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame.getData();
  // auto& camera_info_publisher = camera_info_publishers_.at(stream_index);
  auto& image_publisher = image_writer_.at(stream_index);
  cv::Mat scaled_image;
  if (stream_index == DEPTH) {
    cv::resize(image, scaled_image, cv::Size(width * camera_config_->depth_scale(), height * camera_config_->depth_scale()), 0, 0,
               cv::INTER_NEAREST);
  }

  auto pb_image = std::make_shared<Image>();
  auto header_time = cyber::Time::Now().ToSecond();
  auto measurement_time = frame.getTimestamp()/1000;
  pb_image->mutable_header()->set_timestamp_sec(header_time);
  pb_image->set_measurement_time(measurement_time);

  if (stream_index == DEPTH) {
    // 将cv::Mat数据转换为std::string
    std::string mat_data(reinterpret_cast<const char*>(scaled_image.data), scaled_image.total() * scaled_image.elemSize());
    pb_image->set_data(mat_data);
    pb_image->set_width(width * camera_config_->depth_scale());
    pb_image->set_height(height * camera_config_->depth_scale());
  } else {
    // 将cv::Mat数据转换为std::string
    std::string mat_data(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
    pb_image->set_data(mat_data);
    pb_image->set_width(width);
    pb_image->set_height(height);
  }
  pb_image->mutable_header()->set_frame_id(frame_id_[stream_index]);
  pb_image->set_encoding(encoding_[stream_index]);
  pb_image->set_step(width * unit_step_size_[stream_index]);
  image_publisher->Write(pb_image);
  // auto camera_info = stream_index == COLOR ? getColorCameraInfo() : getDepthCameraInfo();
  // if (camera_info->width != static_cast<uint32_t>(image_msg->width) ||
  //     camera_info->height != static_cast<uint32_t>(image_msg->height)) {
  //   camera_info->width = image_msg->width;
  //   camera_info->height = image_msg->height;
  // }
  // camera_info->header.stamp = timestamp;
  // camera_info->header.frame_id =
  //     depth_align_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];

  // camera_info_publisher->publish(std::move(camera_info));
}


uint8_t* CameraComponent::matToBytes(cv::Mat image) {
    int size = image.total() * image.elemSize();
    uint8_t* bytes = new uint8_t[size];
    std::memcpy(bytes, image.data, size * sizeof(uint8_t));
    return bytes;
}

void CameraComponent::setupPublishers() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_[stream_index]) {
      std::string name = stream_name_[stream_index];
      if (name == "color") {
        image_writer_[stream_index] = node_->CreateWriter<Image>(camera_config_->color_channel_name());
      } else if (name == "depth") {
        image_writer_[stream_index] = node_->CreateWriter<Image>(camera_config_->depth_channel_name());
      } else if (name == "ir") {
        image_writer_[stream_index] = node_->CreateWriter<Image>(camera_config_->ir_channel_name());
      }
    }
  }
}

void CameraComponent::setupVideoMode() {
  if (enable_[INFRA1] && enable_[COLOR]) {
    AWARN<<"Infrared and Color streams are enabled. Infrared stream will be disabled.";
    enable_[INFRA1] = false;
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    supported_video_modes_[stream_index] = std::vector<openni::VideoMode>();
    if (device_->hasSensor(stream_index.first) && enable_[stream_index]) {
      auto stream = streams_[stream_index];
      const auto& sensor_info = stream->getSensorInfo();
      const auto& supported_video_modes = sensor_info.getSupportedVideoModes();
      int size = supported_video_modes.getSize();
      for (int i = 0; i < size; i++) {
        supported_video_modes_[stream_index].emplace_back(supported_video_modes[i]);
      }
      openni::VideoMode video_mode, default_video_mode;
      video_mode.setResolution(width_[stream_index], height_[stream_index]);
      default_video_mode.setResolution(width_[stream_index], height_[stream_index]);
      video_mode.setFps(fps_[stream_index]);
      video_mode.setPixelFormat(format_[stream_index]);
      default_video_mode.setPixelFormat(format_[stream_index]);
      bool is_supported_mode = false;
      bool is_default_mode_supported = false;
     for (const auto& item : supported_video_modes_[stream_index]) {
       if (video_mode.getResolutionX() == item.getResolutionX() &&
           video_mode.getResolutionY() == item.getResolutionY() &&
           video_mode.getPixelFormat() == item.getPixelFormat()) {
          is_supported_mode = true;
          stream_video_mode_[stream_index] = video_mode;
          break;
        }
        if (default_video_mode.getResolutionX() == item.getResolutionX() &&
            default_video_mode.getResolutionY() == item.getResolutionY() &&
            default_video_mode.getPixelFormat() == item.getPixelFormat()) {
          default_video_mode.setFps(item.getFps());
          is_default_mode_supported = true;
        }
      }
      if (!is_supported_mode) {
        AWARN << "Video mode is not supported.";
        if (is_default_mode_supported) {
          AWARN << "Default video mode is supported. Stream will be enabled.";
          stream_video_mode_[stream_index] = default_video_mode;
          video_mode = default_video_mode;
          is_supported_mode = true;
        } else {
          AWARN << "Default video mode is not supported. Stream will be disabled.";
          enable_[stream_index] = false;
          for (const auto& item : supported_video_modes_[stream_index]) {
            AINFO << "Supported video modes: ";
          }
        }
      }
      if (is_supported_mode) {
        AINFO << "set " << stream_name_[stream_index] << " video mode ";
        images_[stream_index] = cv::Mat(height_[stream_index], width_[stream_index],
                                        image_format_[stream_index], cv::Scalar(0, 0, 0));
      }
    }
  }
}

void CameraComponent::startStreams() {
  setupVideoMode();
  // int color_width = 0;
  // int color_height = 0;

  // color_width = stream_video_mode_[COLOR].getResolutionX();
  // color_height = stream_video_mode_[COLOR].getResolutionY();
  
  setImageRegistrationMode(camera_config_->depth_align());
  setDepthColorSync(camera_config_->color_depth_synchronization());  //同步深度图像和彩色图像
  // if (camera_config_->depth_align()) {
  //   setDepthToColorResolution(color_width, color_height);  //设置图像分变率
  // }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_[stream_index] && !stream_started_[stream_index]) {
      CHECK(stream_video_mode_.count(stream_index));
      auto video_mode = stream_video_mode_.at(stream_index);
      CHECK(streams_.count(stream_index));
      streams_[stream_index]->setVideoMode(video_mode);
      streams_[stream_index]->setMirroringEnabled(false);
      CHECK(stream_frame_listener_.count(stream_index));
      CHECK_NOTNULL(stream_frame_listener_[stream_index]);
      streams_[stream_index]->addNewFrameListener(stream_frame_listener_[stream_index].get());
      CHECK_EQ(streams_[stream_index]->start(), openni::STATUS_OK);
      stream_started_[stream_index] = true;
      
    }
  }
}

void CameraComponent::setDepthColorSync(bool data) {
  auto rc = device_->setDepthColorSyncEnabled(data); 
  if (rc != openni::STATUS_OK) {
    AERROR<<"Enabling depth color synchronization failed: "<< openni::OpenNI::getExtendedError();
  }
}

void CameraComponent::setImageRegistrationMode(bool data) {
  if (!device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    AWARN << "Current do not support " << openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
    return;
  }
  auto mode = data ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF;
  auto rc = device_->setImageRegistrationMode(mode);
  if (rc != openni::STATUS_OK) {
    AERROR << "Enabling image registration mode failed: "<<openni::OpenNI::getExtendedError();
  }
}

void CameraComponent::setupDevices() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
    if (device_ == nullptr) {
      AERROR << "Device is not connected";
    }

    //if (enable_[stream_index] && device_->hasSensor(stream_index.first)) {
    if (enable_[stream_index] && (device_->getSensorInfo(stream_index.first)!=nullptr)) {
      auto stream = std::make_shared<openni::VideoStream>();
      auto status = stream->create(*device_, stream_index.first);
      if (status != openni::STATUS_OK) {
        AERROR << "Couldn't create depth video stream: " << openni::OpenNI::getExtendedError();
      }
      CHECK_EQ(status, openni::STATUS_OK);
      streams_[stream_index] = stream;
      auto frame_listener = std::make_shared<OBFrameListener>();
      frame_listener->setCallback(stream_frame_callback_[stream_index]);
      stream_frame_listener_[stream_index] = frame_listener;
    } else {
      if (streams_[stream_index]) {
        streams_[stream_index].reset();
      }
      enable_[stream_index] = false;
      if (stream_frame_listener_[stream_index]) {
        stream_frame_listener_[stream_index].reset();
      }
    }
  }
  device_info_ = device_->getDeviceInfo();
}

void CameraComponent::stopStreams() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (stream_started_[stream_index]) {
      streams_[stream_index]->stop();
      auto listener = stream_frame_listener_[stream_index];
      streams_[stream_index]->removeNewFrameListener(listener.get());
      AINFO << "Stopped stream " << stream_name_[stream_index];
      stream_started_[stream_index] = false;
    }
  }
}

void CameraComponent::clean() {
  if (running_.load()) {
    running_.exchange(false);
    //async_result_.wait();
  }
  stopStreams();
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (streams_[stream_index]) {
      streams_[stream_index]->destroy();
      streams_[stream_index].reset();
    }
  }
}

/*
void CameraComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    if (!camera_device_->wait_for_device()) {
      // sleep for next check
      cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      continue;
    }

    if (!camera_device_->poll(raw_image_, raw_image_for_compress_)) {
      AERROR << "camera device poll failed";
      continue;
    }

    cyber::Time image_time(raw_image_->tv_sec, 1000 * raw_image_->tv_usec);
    if (index_ >= buffer_size_) {
      index_ = 0;
    }
    auto pb_image = pb_image_buffer_.at(index_);
    auto header_time = cyber::Time::Now().ToSecond();
    auto measurement_time = image_time.ToSecond();
    pb_image->mutable_header()->set_timestamp_sec(header_time);
    pb_image->set_measurement_time(measurement_time);
    pb_image->set_data(raw_image_->image, raw_image_->image_size);
    writer_->Write(pb_image);

    auto raw_image_for_compress = raw_image_buffer_.at(index_++);
    raw_image_for_compress->mutable_header()->set_timestamp_sec(
        header_time);
    raw_image_for_compress->set_measurement_time(measurement_time);
    raw_image_for_compress->set_data(raw_image_for_compress_->image,
                                      raw_image_for_compress_->image_size);
    raw_writer_->Write(raw_image_for_compress);

    cyber::SleepFor(std::chrono::microseconds(spin_rate_));
  }
}

*/

CameraComponent::~CameraComponent() {
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  clean();
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
