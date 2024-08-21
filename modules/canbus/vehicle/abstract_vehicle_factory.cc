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

#include "modules/canbus/vehicle/abstract_vehicle_factory.h"

namespace apollo {
namespace canbus {

void cpy_odom_pose_covariance(int index, Adometry& odometry) {
  odometry.mutable_pose()->mutable_covariance()->Reserve(36);
  if (index == 1) {
    for (int i = 0; i < 36; i++) {
      odometry.mutable_pose()->add_covariance(odom_pose_covariance1[i]);
    }
  } else {
    for (int i = 0; i < 36; i++) {
      odometry.mutable_pose()->add_covariance(odom_pose_covariance2[i]);
    }
  }
}

void cpy_odom_twist_covariance(int index, Adometry& odometry) {
  odometry.mutable_twist()->mutable_covariance()->Reserve(36);
  if (index == 1) {
    for (int i = 0; i < 36; i++) {
      odometry.mutable_twist()->add_covariance(odom_twist_covariance1[i]);
    }
  } else {
    for (int i = 0; i < 36; i++) {
      odometry.mutable_twist()->add_covariance(odom_twist_covariance2[i]);
    }
  }
}

void AbstractVehicleFactory::UpdateHeartbeat() {}

bool AbstractVehicleFactory::publish_odometry(Adometry& odometry) {return false;};
bool AbstractVehicleFactory::publish_imu_sensor(AkmanImu& akman_imu) {return false;};

void AbstractVehicleFactory::SetVehicleParameter(
    const VehicleParameter &vehicle_parameter) {
  vehicle_parameter_ = vehicle_parameter;
}

}  // namespace canbus
}  // namespace apollo
