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
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// #include "modules/drivers/gnss/proto/imu.pb.h"
// #include "modules/localization/proto/gps.pb.h"
// #include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
// #include "modules/localization/proto/measure.pb.h"
// #include "modules/localization/proto/sins_pva.pb.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/common/apollo_app.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_manager.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class OnlineLocalVisualizer : public apollo::common::ApolloApp {
 public:
  OnlineLocalVisualizer();
  ~OnlineLocalVisualizer();

  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  void Stop() override;

 private:
  void InitParams();
  void OnPointCloud(const sensor_msgs::PointCloud2 &message);
  void OnLidarLocalization(const LocalizationEstimate &message);
  void OnGNSSLocalization(const LocalizationEstimate &message);
  void OnFusionLocalization(const LocalizationEstimate &message);

  void ParsePointCloudMessage(const sensor_msgs::PointCloud2 &message,
                              std::vector<Eigen::Vector3d> *pt3ds,
                              std::vector<unsigned char> *intensities);

 private:
  apollo::common::monitor::MonitorLogger monitor_logger_;
  std::string lidar_extrinsic_file_;
  std::string map_folder_;
  std::string map_visual_folder_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ONLINE_LOCAL_VISUALIZER_H
