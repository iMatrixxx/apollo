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

#include "modules/canbus_vehicle/lincoln/lincoln_vehicle_factory.h"

#include "cyber/common/log.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus_vehicle/lincoln/lincoln_controller.h"
#include "modules/canbus_vehicle/lincoln/lincoln_message_manager.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using apollo::drivers::canbus::CanClientFactory;

namespace apollo {
namespace canbus {

bool LincolnVehicleFactory::Init(const CanbusConf *canbus_conf) {
  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf->can_card_parameter());
  if (!can_client_) {
    AERROR << "Failed to create can client.";
    return false;
  }
  AINFO << "Can client is successfully created.";

  message_manager_ = this->CreateMessageManager();
  if (message_manager_ == nullptr) {
    AERROR << "Failed to create message manager.";
    return false;
  }
  AINFO << "Message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         canbus_conf->enable_receiver_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can receiver.";
    return false;
  }
  AINFO << "The can receiver is successfully initialized.";

  if (can_sender_.Init(can_client_.get(), message_manager_.get(),
                       canbus_conf->enable_sender_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can sender.";
    return false;
  }
  AINFO << "The can sender is successfully initialized.";

  vehicle_controller_ = CreateVehicleController();
  if (vehicle_controller_ == nullptr) {
    AERROR << "Failed to create vehicle controller.";
    return false;
  }
  AINFO << "The vehicle controller is successfully created.";

  if (vehicle_controller_->Init(canbus_conf->vehicle_parameter(), &can_sender_,
                                message_manager_.get()) != ErrorCode::OK) {
    AERROR << "Failed to init vehicle controller.";
    return false;
  }

  AINFO << "The vehicle controller is successfully"
        << " initialized with canbus conf as : "
        << canbus_conf->vehicle_parameter().ShortDebugString();

  node_ = ::apollo::cyber::CreateNode("chassis_detail");

  chassis_detail_writer_ = node_->CreateWriter<::apollo::canbus::Lincoln>(
      FLAGS_chassis_detail_topic);

  odometry_frame_id_ = canbus_conf->odom_frame_id();
  robot_frame_id_ = canbus_conf->robot_frame_id();
  
  return true;
}

bool LincolnVehicleFactory::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    AERROR << "Failed to start can client";
    return false;
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can receiver.";
    return false;
  }
  AINFO << "Can receiver is started.";

  // 3. start send
  if (can_sender_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can sender.";
    return false;
  }

  // 4. start controller
  if (!vehicle_controller_->Start()) {
    AERROR << "Failed to start vehicle controller.";
    return false;
  }

  _Last_Time = cyber::Time::Now();

  return true;
}

void LincolnVehicleFactory::Stop() {
  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
  AINFO << "Cleanup cansender, canreceiver, canclient, vehicle controller.";
}

void LincolnVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand *control_command) {
  if (vehicle_controller_->Update(*control_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  can_sender_.Update();
}

void LincolnVehicleFactory::UpdateCommand(
    const apollo::external_command::ChassisCommand *chassis_command) {
  if (vehicle_controller_->Update(*chassis_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  can_sender_.Update();
}

Chassis LincolnVehicleFactory::publish_chassis() {
  Chassis chassis = vehicle_controller_->chassis();
  ADEBUG << chassis.ShortDebugString();
  return chassis;
}

void LincolnVehicleFactory::PublishChassisDetail() {
  Lincoln chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(chassis_detail);
}

bool LincolnVehicleFactory::publish_odometry(Adometry& odometry) {
  Lincoln chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.is_akman101()) {
    return false;
  }

  _Now = cyber::Time::Now();
  Sampling_Time = (_Now - _Last_Time).ToSecond();
  if (chassis_detail.has_robot_vel())

  //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
  Robot_Pos.X+=(chassis_detail.robot_vel().x() * cos(Robot_Pos.Z) - chassis_detail.robot_vel().y()* sin(Robot_Pos.Z)) * Sampling_Time; 
  //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
  Robot_Pos.Y+=(chassis_detail.robot_vel().x() * sin(Robot_Pos.Z) + chassis_detail.robot_vel().y() * cos(Robot_Pos.Z)) * Sampling_Time; 
  //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
  Robot_Pos.Z+=chassis_detail.robot_vel().z() * Sampling_Time; 

  AWARN << "------------ odometry position --------------";
  AWARN <<"x: "<< Robot_Pos.X <<"y: "<< Robot_Pos.Y <<"z: "<< Robot_Pos.Z;
  AWARN << "Sampling_Time "<<Sampling_Time;

  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z);

  odometry.mutable_header()->set_frame_id(odometry_frame_id_);
  odometry.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_x(Robot_Pos.X);
  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_y(Robot_Pos.Y);
  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_z(Robot_Pos.Z);

  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_qx(q.x());
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_qy(q.y());
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_qz(q.z());
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_qw(q.w());

  odometry.set_child_frame_id(robot_frame_id_);
  odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(chassis_detail.robot_vel().x());
  odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_y(chassis_detail.robot_vel().y());
  odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(chassis_detail.robot_vel().z());
  //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
  //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
  if(chassis_detail.robot_vel().x()== 0&&
     chassis_detail.robot_vel().y()== 0&&
     chassis_detail.robot_vel().z()== 0) {
    //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    cpy_odom_pose_covariance(2, odometry);
    cpy_odom_twist_covariance(2, odometry);
  } else {
    //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    cpy_odom_pose_covariance(1, odometry);
    cpy_odom_twist_covariance(1, odometry);
  }
  _Last_Time = _Now;
  return true;
}


bool LincolnVehicleFactory::publish_imu_sensor(AkmanImu& akman_imu) {
  Lincoln chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  if (!(chassis_detail.is_akman102()&&chassis_detail.is_akman102())) {
    return false;
  }

  akman_imu.mutable_header()->set_frame_id(gyro_frame_id_);
  akman_imu.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

  //四元数表达三轴姿态
  apollo::common::Quaternion q;
  Robot_Quat.Quaternion_Solution(chassis_detail.mpu6050().angular_velocity().x(),
                      chassis_detail.mpu6050().angular_velocity().y(),
                      chassis_detail.mpu6050().angular_velocity().z(),
                      chassis_detail.mpu6050().linear_acceleration().x(),
                      chassis_detail.mpu6050().linear_acceleration().y(),
                      chassis_detail.mpu6050().linear_acceleration().z(),
                      q);
  AWARN << "debug imu orientation-------------------------";
  AWARN <<"qx: "<< q.qx(); 
  AWARN <<"qy: "<< q.qy();
  AWARN <<"qz: "<< q.qz();
  AWARN <<"qw: "<< q.qw();
  AWARN << "debug angular_velocity-------------------------";
  AWARN <<"ax: "<< chassis_detail.mpu6050().angular_velocity().x();
  AWARN <<"ay: "<< chassis_detail.mpu6050().angular_velocity().y();
  AWARN <<"az: "<< chassis_detail.mpu6050().angular_velocity().z();
  AWARN << "debug linear_acceleration-------------------------";
  AWARN <<"lx: "<< chassis_detail.mpu6050().linear_acceleration().x();
  AWARN <<"ly: "<< chassis_detail.mpu6050().linear_acceleration().y();
  AWARN <<"lz: "<< chassis_detail.mpu6050().linear_acceleration().z();

  akman_imu.mutable_orientation()->set_qx(q.qx());
  akman_imu.mutable_orientation()->set_qy(q.qy());
  akman_imu.mutable_orientation()->set_qz(q.qz());
  akman_imu.mutable_orientation()->set_qw(q.qw());

  //三轴姿态协方差矩阵
  //三轴角速度协方差矩阵
  akman_imu.mutable_orientation_covariance()->Reserve(9);
  akman_imu.mutable_angular_velocity_covariance()->Reserve(9);
  for (int i = 0; i < 9; i++) {
    if (i == 0 || i == 4) {
      akman_imu.add_orientation_covariance(1e6);
      akman_imu.add_angular_velocity_covariance(1e6);
    } else if ( i == 8 ){
      akman_imu.add_orientation_covariance(1e-6);
      akman_imu.add_angular_velocity_covariance(1e-6);
    } else {
      akman_imu.add_orientation_covariance(0);
      akman_imu.add_angular_velocity_covariance(0);
    }
  }

  //三轴角速度
  akman_imu.mutable_angular_velocity()->set_x(chassis_detail.mpu6050().angular_velocity().x());
  akman_imu.mutable_angular_velocity()->set_y(chassis_detail.mpu6050().angular_velocity().y());
  akman_imu.mutable_angular_velocity()->set_z(chassis_detail.mpu6050().angular_velocity().z());

  //三轴线性加速度
  akman_imu.mutable_linear_acceleration()->set_x(chassis_detail.mpu6050().linear_acceleration().x());
  akman_imu.mutable_linear_acceleration()->set_y(chassis_detail.mpu6050().linear_acceleration().y());
  akman_imu.mutable_linear_acceleration()->set_z(chassis_detail.mpu6050().linear_acceleration().z());
  return true;
}

std::unique_ptr<VehicleController<::apollo::canbus::Lincoln>>
LincolnVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController<::apollo::canbus::Lincoln>>(
      new lincoln::LincolnController());
}

std::unique_ptr<MessageManager<::apollo::canbus::Lincoln>>
LincolnVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::Lincoln>>(
      new lincoln::LincolnMessageManager());
}

}  // namespace canbus
}  // namespace apollo
