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
 * @file online_local_visualizer.h
 * @brief
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "cyber/cyber.h"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_manager.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class OnlineVisualizerComponent final
    : public cyber::Component<drivers::PointCloud> {
 public:
  OnlineVisualizerComponent();
  ~OnlineVisualizerComponent();

  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::PointCloud> &msg) override;

 private:
  bool InitConfig();
  bool InitIO();
  void OnLidarLocalization(
      const std::shared_ptr<LocalizationEstimate> &message);
  void OnGNSSLocalization(const std::shared_ptr<LocalizationEstimate> &message);
  void OnFusionLocalization(
      const std::shared_ptr<LocalizationEstimate> &message);

  void ParsePointCloudMessage(
      const std::shared_ptr<drivers::PointCloud> &message,
      ::apollo::common::EigenVector3dVec *pt3ds,
      std::vector<unsigned char> *intensities);

 private:
  std::string lidar_extrinsic_file_;
  std::string map_folder_;
  std::string map_visual_folder_;

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> lidar_local_listener_ =
      nullptr;
  std::string lidar_local_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> gnss_local_listener_ =
      nullptr;
  std::string gnss_local_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> fusion_local_listener_ =
      nullptr;
  std::string fusion_local_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> ndt_local_listener_ =
      nullptr;
  std::string ndt_local_topic_ = "";
};

CYBER_REGISTER_COMPONENT(OnlineVisualizerComponent);

}  // namespace msf
}  // namespace localization
}  // namespace apollo
