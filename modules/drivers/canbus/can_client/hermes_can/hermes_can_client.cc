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

#include "modules/drivers/canbus/can_client/hermes_can/hermes_can_client.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

/*------------  初始化帧参数  -------------------*/
auto ACC_CODE = 0x80000000;  // # 过滤验收码
auto ACC_MASK = 0xFFFFFFFF;  // # 过滤屏蔽码
auto FILTER = 1;             // # 滤波模式 0/1=接收所有类型
auto TIMING_0 = 0x00;        // # 波特率 T0   0x00: 对应1000Kbps
auto TIMING_1 = 0x14;        // # 波特率 T1   0x14: 对应1000Kbps
auto MODE = 0;               // # 工作模式 0=正常工作

/*-----------   发送帧参数  --------------------*/
UINT TIME_STAMP = 10;  // 时间标识，仅在接收帧时有意义
BYTE TIME_FLAG = 1;    // 是否使用时间标识,仅在接收帧时有意义
BYTE TRANSMIT_SEND_TYPE =
    1;  // 发送帧类型，0：正常发送，发送失败重发，4秒内未发送则取消；1：单次发送
BYTE REMOTE_FLAG = 0;  // 是否是远程帧， 0：数据帧； 1：远程帧
BYTE EXTERN_FLAG = 0;  // 是否是扩展帧，0：标准帧(11位ID) 1：扩展帧（29位ID）
BYTE DATA_LEN = 8;  // 数据长度

using apollo::common::ErrorCode;

HermesCanClient::~HermesCanClient() {
  if (dev_handler_) {
    Stop();
  }
}

void HermesCanClient::SetCANObjStdConfig(VCI_CAN_OBJ &obj) {
  obj.TimeStamp = TIME_STAMP;
  obj.TimeFlag = TIME_FLAG;
  obj.SendType = TRANSMIT_SEND_TYPE;
  obj.RemoteFlag = REMOTE_FLAG;
  obj.ExternFlag = EXTERN_FLAG;
  obj.DataLen = DATA_LEN;
  for (int i = 0; i < 3; ++i) {
    obj.Reserved[i] = 0;
  }
}

bool HermesCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();
  auto num_ports = parameter.num_ports();
  if (port_ > static_cast<int32_t>(num_ports) || port_ < 0) {
    AERROR << "Can port number [" << port_ << "] is out of bound [0,"
           << num_ports << ")";
    return false;
  }

  return true;
}

ErrorCode HermesCanClient::Start() {
  if (is_init_) {
    return ErrorCode::OK;
  }

  // open device
  if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1) {
    AERROR << "Open device error";
    printf("打开设备失败\n");
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  printf("打开设备1成功\n");
  AWARN << "Open device success, channel id: " << port_;
  // init
  VCI_INIT_CONFIG init_config;
  init_config.AccCode = ACC_CODE;
  init_config.AccMask = ACC_MASK;
  init_config.Filter = FILTER;
  init_config.Timing0 = TIMING_0;
  init_config.Timing1 = TIMING_1;
  init_config.Mode = MODE;
  if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &init_config) != 1) {
    AERROR << ">> Init CAN1 Error!";
     printf("初始化CAN1失败\n");
    VCI_CloseDevice(0, 0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  printf("初始化CAN1成功\n");
  AWARN << ">> Init CAN1 Success!";

  // 2. start receive
  if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
    AERROR << ">> Start CAN1 error!";
     printf("开启CAN1失败\n");
    VCI_CloseDevice(0, 0);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  printf("开启CAN1成功\n");
  AWARN << ">> Start CAN1 Success!";
  
  is_init_ = true;
  return ErrorCode::OK;
}

void HermesCanClient::Stop() {
  if (is_init_) {
    is_init_ = false;
    //int32_t ret = bcan_close(dev_handler_);  //apollo source code
    // if (ret != ErrorCode::OK) {
    //   AERROR << "close error code: " << ret;
    // }
    int32_t ret = VCI_CloseDevice(VCI_USBCAN2, VCI_DEVICE_ID_0);
    if (ret != 1) {
      AERROR << "close error code: " << ret;
    }
  }
}

// Synchronous transmission of CAN messages
apollo::common::ErrorCode HermesCanClient::Send(
    const std::vector<CanFrame> &frames, int32_t *const frame_num) {
  /*
  typedef struct bcan_msg {
      uint32_t bcan_msg_id;        // source CAN node id
      uint8_t  bcan_msg_datalen;   // message data length
      uint8_t  bcan_msg_rsv[3];    // reserved
      uint8_t  bcan_msg_data[8];   // message data
      uint64_t bcan_msg_timestamp; // TBD
  } bcan_msg_t;
  */
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_init_) {
    AERROR << "Hermes can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  if (*frame_num > MAX_CAN_SEND_FRAME_LEN || *frame_num < 0) {
    AERROR << "send can frame num not in range[0, "
      << MAX_CAN_SEND_FRAME_LEN << "], frame_num:" << *frame_num;
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  for (int i = 0; i < *frame_num; ++i) {
    /*  apollo source code
    _send_frames[i].bcan_msg_id = frames[i].id;
    _send_frames[i].bcan_msg_datalen = frames[i].len;
    memcpy(_send_frames[i].bcan_msg_data, frames[i].data, frames[i].len);  
    */

    //------------- zhxf 20240725 -----------------------------------
    SetCANObjStdConfig(_send_frames[i]);
    _send_frames[i].ID = frames[i].id;
    _send_frames[i].DataLen = frames[i].len;
    memcpy(_send_frames[i].Data, frames[i].data, frames[i].len);
    //-----------------------------------------------------------------
  }

  // Synchronous transmission of CAN messages
  int32_t send_num = *frame_num;
  //int32_t ret = bcan_send(dev_handler_, _send_frames, send_num); //apollo source code
  int32_t ret = VCI_Transmit(VCI_USBCAN2, VCI_DEVICE_ID_0, VCI_CAN_CHANNEL_0,_send_frames, send_num);
  if (ret < 0) {
    //int ret_send_error = bcan_get_status(dev_handler_);  //apollo source code
    AERROR << "send message failed, error code: " << ret;
          // << ", send error: " << ret_send_error;
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  *frame_num = ret;
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
const int RX_TIMEOUT = -7;

apollo::common::ErrorCode HermesCanClient::Receive(
    std::vector<CanFrame> *const frames, int32_t *const frame_num) {
  if (!is_init_) {
    AERROR << "Hermes can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  //int32_t ret = bcan_recv(dev_handler_, _recv_frames, *frame_num);   //apollo source code
  int32_t ret = VCI_Receive(VCI_USBCAN2, VCI_DEVICE_ID_0, VCI_CAN_CHANNEL_0, _recv_frames, *frame_num, 1000);

  // don't log timeout
  if (ret == RX_TIMEOUT) {
    *frame_num = 0;
    return ErrorCode::OK;
  }

  if (ret < 0) {
    //int ret_rece_error = bcan_get_status(dev_handler_); //apollo source code
    //AERROR << "receive message failed, error code:" << ret
          // << "receive error:" << ret_rece_error;
    AERROR << "Can receive message failed, received number of data:"<< ret;
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
  *frame_num = ret;

  // is ret num is equal *frame_num?
  for (int i = 0; i < *frame_num; ++i) {
    CanFrame cf;
    // cf.id = _recv_frames[i].bcan_msg_id;
    // cf.len = _recv_frames[i].bcan_msg_datalen;
    // cf.timestamp.tv_sec = _recv_frames[i].bcan_msg_timestamp.tv_sec;
    // cf.timestamp.tv_usec = _recv_frames[i].bcan_msg_timestamp.tv_usec;
    // memcpy(cf.data, _recv_frames[i].bcan_msg_data, cf.len);  
    cf.id = _recv_frames[i].ID;
    cf.len = _recv_frames[i].DataLen;
    cf.timestamp.tv_sec = _recv_frames[i].TimeStamp*10*1000;
    cf.timestamp.tv_usec = _recv_frames[i].TimeStamp/100.0;
    memcpy(cf.data, _recv_frames[i].Data, cf.len); 
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

std::string HermesCanClient::GetErrorString(int32_t ntstatus) { return ""; }

void HermesCanClient::SetInited(bool init) { is_init_ = init; }

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
