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

#ifndef MODULES_COMMON_ADAPTERS_ADAPTER_GFLAGS_H_
#define MODULES_COMMON_ADAPTERS_ADAPTER_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_bool(enable_adapter_dump);
DECLARE_string(monitor_topic);
DECLARE_string(gps_topic);
DECLARE_string(imu_topic);
DECLARE_string(chassis_topic);
DECLARE_string(chassis_detail_topic);
DECLARE_string(localization_topic);
DECLARE_string(planning_trajectory_topic);
DECLARE_string(monitor_topic);
DECLARE_string(pad_topic);
DECLARE_string(control_command_topic);
DECLARE_string(pointcloud_topic);
DECLARE_string(radar_topic);
DECLARE_string(prediction_topic);
DECLARE_string(perception_obstacle_topic);
DECLARE_string(traffic_light_detection_topic);
DECLARE_string(routing_request_topic);
DECLARE_string(routing_response_topic);
DECLARE_string(relative_odometry_topic);
DECLARE_string(ins_stat_topic);
DECLARE_string(ins_status_topic);
DECLARE_string(gnss_status_topic);
DECLARE_string(system_status_topic);
// TODO(xiaoxq): Retire hmi_command topic after integration with dreamview.
DECLARE_string(hmi_command_topic);
DECLARE_string(mobileye_topic);
DECLARE_string(delphi_esr_topic);
DECLARE_string(compressed_image_topic);

#endif  // MODULES_COMMON_ADAPTERS_ADAPTER_GFLAGS_H_
