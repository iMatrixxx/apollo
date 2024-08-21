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

/**
 * @file
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "cyber/class_loader/class_loader_register_macro.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

//zhxf add 阿克曼小车
#include "modules/common_msgs/akman_msgs/adometry.pb.h"
#include "modules/common_msgs/akman_msgs/akeman_imu.pb.h"
#include "tf2/LinearMath/Quaternion.h"
#include "modules/canbus/tools/Quaternion_Solution.h"

using apollo::control::ControlCommand;

//--------- zhxf add 阿克曼小车 ------
using apollo::akman::Adometry;
using apollo::akman::AkmanImu;
//-----------------------------------

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

  const double odom_pose_covariance1[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

  const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };
  const double odom_twist_covariance1[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
 const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	double X = 0.0;
	double Y = 0.0;
	double Z = 0.0;
}Vel_Pos_Data;

void cpy_odom_pose_covariance(int index, Adometry& odometry);
void cpy_odom_twist_covariance(int index, Adometry& odometry);

/**
 * @class AbstractVehicleFactory
 *
 * @brief this class is the abstract factory following the AbstractFactory
 * design pattern. It can create VehicleController and MessageManager based on
 * a given VehicleParameter.
 */
class AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  virtual ~AbstractVehicleFactory() = default;

  /**
   * @brief set VehicleParameter.
   */
  void SetVehicleParameter(const VehicleParameter &vehicle_paramter);

  /**
   * @brief init vehicle factory
   * @returns true if successfully initialized
   */
  virtual bool Init(const CanbusConf *canbus_conf) = 0;

  /**
   * @brief start canclient, cansender, canreceiver, vehicle controller
   * @returns true if successfully started
   */
  virtual bool Start() = 0;

  /**
   * @brief stop canclient, cansender, canreceiver, vehicle controller
   */
  virtual void Stop() = 0;

  /**
   * @brief update control command
   */
  virtual void UpdateCommand(const ControlCommand *control_command) = 0;

  /**
   * @brief update chassis command
   */
  virtual void UpdateCommand(const ChassisCommand *chassis_command) = 0;

  /**
   * @brief publish chassis messages
   */
  virtual Chassis publish_chassis() = 0;

  /**
   * @brief publish chassis for vehicle messages
   */
  virtual void PublishChassisDetail() = 0;

  virtual bool publish_odometry(Adometry& odometry);
  virtual bool publish_imu_sensor(AkmanImu& akman_imu);

  /**
   * @brief create cansender heartbeat
   */
  virtual void UpdateHeartbeat();

 private:
  VehicleParameter vehicle_parameter_;
};

#define CYBER_REGISTER_VEHICLEFACTORY(name) \
  CLASS_LOADER_REGISTER_CLASS(name, AbstractVehicleFactory)

}  // namespace canbus
}  // namespace apollo
