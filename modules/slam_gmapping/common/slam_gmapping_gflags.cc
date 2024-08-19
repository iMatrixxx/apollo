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

#include "modules/slam_gmapping/common/slam_gmapping_gflags.h"

// System gflags
DEFINE_string(slam_gmapping_node_name, "SlamGmapping", "The chassis module name in proto");
DEFINE_string(slam_gmapping_module_name, "slam_gmapping_component", "Module name");
DEFINE_string(laser2footprtint_frame_id, "laser_scan", "laser2footprtint_frame_id");
DEFINE_string(footprtint2laser_frame_id, "base_footprint", "footprtint2laser_frame_id");

// data file
// DEFINE_string(canbus_conf_file,
//               "/apollo/modules/canbus/conf/canbus_conf.pb.txt",
//               "Default canbus conf file");


