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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ONLINE_LOCAL_VISUALIZER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ONLINE_LOCAL_VISUALIZER_H

#include <Eigen/Geometry>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "cybertron/cybertron.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/common/status/status.h"
#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_manager.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class OnlineVisualizerComponent final
    : public cybertron::Component<drivers::PointCloud> {
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
      std::vector<Eigen::Vector3d> *pt3ds,
      std::vector<unsigned char> *intensities);

 private:
  std::string lidar_extrinsic_file_;
  std::string map_folder_;
  std::string map_visual_folder_;

  std::shared_ptr<cybertron::Reader<LocalizationEstimate>>
      lidar_local_listener_ = nullptr;
  std::string lidar_local_topic_ = "";

  std::shared_ptr<cybertron::Reader<LocalizationEstimate>>
      gnss_local_listener_ = nullptr;
  std::string gnss_local_topic_ = "";

  std::shared_ptr<cybertron::Reader<LocalizationEstimate>>
      fusion_local_listener_ = nullptr;
  std::string fusion_local_topic_ = "";
};

CYBERTRON_REGISTER_COMPONENT(OnlineVisualizerComponent);

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ONLINE_LOCAL_VISUALIZER_H
