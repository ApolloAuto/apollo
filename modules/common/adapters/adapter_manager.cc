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

#include "modules/common/adapters/adapter_manager.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace common {
namespace adapter {

AdapterManager::AdapterManager() {}

void AdapterManager::Observe() {
  for (const auto observe : instance()->observers_) {
    observe();
  }
}

bool AdapterManager::Initialized() {
  return instance()->initialized_;
}

void AdapterManager::Reset() {
  instance()->initialized_ = false;
  instance()->observers_.clear();
}

void AdapterManager::Init(const std::string &adapter_config_filename) {
  // Parse config file
  AdapterManagerConfig configs;
  CHECK(util::GetProtoFromFile(adapter_config_filename, &configs))
      << "Unable to parse adapter config file " << adapter_config_filename;
  AINFO << "Init AdapterManger config:" << configs.DebugString();
  Init(configs);
}

void AdapterManager::Init(const AdapterManagerConfig &configs) {
  if (Initialized()) {
    return;
  }

  instance()->initialized_ = true;
  if (configs.is_ros()) {
    instance()->node_handle_.reset(new ros::NodeHandle());
  }

  for (const auto &config : configs.config()) {
    switch (config.type()) {
      case AdapterConfig::POINT_CLOUD:
        EnablePointCloud(FLAGS_pointcloud_topic, config.mode(),
                         config.message_history_limit());
        break;
      case AdapterConfig::GPS:
        EnableGps(FLAGS_gps_topic, config.mode(),
                  config.message_history_limit());
        break;
      case AdapterConfig::IMU:
        EnableImu(FLAGS_imu_topic, config.mode(),
                  config.message_history_limit());
        break;
      case AdapterConfig::CHASSIS:
        EnableChassis(FLAGS_chassis_topic, config.mode(),
                      config.message_history_limit());
        break;
      case AdapterConfig::LOCALIZATION:
        EnableLocalization(FLAGS_localization_topic, config.mode(),
                           config.message_history_limit());
        break;
      case AdapterConfig::PERCEPTION_OBSTACLES:
        EnablePerceptionObstacles(FLAGS_perception_obstacle_topic,
                                  config.mode(),
                                  config.message_history_limit());
        break;
      case AdapterConfig::TRAFFIC_LIGHT_DETECTION:
        EnableTrafficLightDetection(FLAGS_traffic_light_detection_topic,
                                    config.mode(),
                                    config.message_history_limit());
        break;
      case AdapterConfig::PAD:
        EnablePad(FLAGS_pad_topic, config.mode(),
                  config.message_history_limit());
        break;
      case AdapterConfig::CONTROL_COMMAND:
        EnableControlCommand(FLAGS_control_command_topic, config.mode(),
                             config.message_history_limit());
        break;
      case AdapterConfig::ROUTING_REQUEST:
        EnableRoutingRequest(FLAGS_routing_request_topic, config.mode(),
                             config.message_history_limit());
        break;
      case AdapterConfig::ROUTING_RESPONSE:
        EnableRoutingResponse(FLAGS_routing_response_topic, config.mode(),
                              config.message_history_limit());
        break;
      case AdapterConfig::PLANNING_TRAJECTORY:
        EnablePlanning(FLAGS_planning_trajectory_topic, config.mode(),
                       config.message_history_limit());
        break;
      case AdapterConfig::PREDICTION:
        EnablePrediction(FLAGS_prediction_topic, config.mode(),
                         config.message_history_limit());
        break;
      case AdapterConfig::MONITOR:
        EnableMonitor(FLAGS_monitor_topic, config.mode(),
                      config.message_history_limit());
        break;
      case AdapterConfig::CHASSIS_DETAIL:
        EnableChassisDetail(FLAGS_chassis_detail_topic, config.mode(),
                            config.message_history_limit());
        break;
      case AdapterConfig::RELATIVE_ODOMETRY:
        EnableRelativeOdometry(FLAGS_relative_odometry_topic, config.mode(),
                               config.message_history_limit());
        break;
      case AdapterConfig::INS_STAT:
        EnableInsStat(FLAGS_ins_stat_topic, config.mode(),
                      config.message_history_limit());
        break;
      case AdapterConfig::INS_STATUS:
        EnableInsStatus(FLAGS_ins_status_topic, config.mode(),
                        config.message_history_limit());
        break;
      case AdapterConfig::GNSS_STATUS:
        EnableGnssStatus(FLAGS_gnss_status_topic, config.mode(),
                         config.message_history_limit());
        break;
      case AdapterConfig::SYSTEM_STATUS:
        EnableSystemStatus(FLAGS_system_status_topic, config.mode(),
                           config.message_history_limit());
        break;
      case AdapterConfig::HMI_COMMAND:
        EnableHMICommand(FLAGS_hmi_command_topic, config.mode(),
                         config.message_history_limit());
        break;
      case AdapterConfig::MOBILEYE:
        EnableMobileye(FLAGS_mobileye_topic, config.mode(),
                       config.message_history_limit());
        break;
      case AdapterConfig::DELPHIESR:
        EnableDelphiESR(FLAGS_delphi_esr_topic, config.mode(),
                        config.message_history_limit());
        break;
      case AdapterConfig::COMPRESSED_IMAGE:
        EnableCompressedImage(FLAGS_compressed_image_topic, config.mode(),
                              config.message_history_limit());
        break;
      case AdapterConfig::RADAR:
        EnablePointCloud(FLAGS_radar_topic, config.mode(),
                         config.message_history_limit());
        break;
      default:
        AERROR << "Unknown adapter config type!";
        break;
    }
  }
}

}  // namespace adapter
}  // namespace common
}  // namespace apollo
