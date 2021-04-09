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

#ifndef MODEULES_CALIBRATION_LIDAR_EX_CHECKER_H_
#define MODEULES_CALIBRATION_LIDAR_EX_CHECKER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/perception/common/pcl_types.h"

/**
 * @namespace apollo::calibration
 * @brief apollo::calibration
 */
namespace apollo {
namespace calibration {

using apollo::perception::pcl_util::PointXYZIT;

class LidarExChecker : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  // get extrinsics which are calibrated before
  bool GetExtrinsics();
  // visualize the checking result
  void VisualizeClouds();

  // Upon receiving point cloud data
  void OnPointCloud(const sensor_msgs::PointCloud2& message);
  // Upon receiving GPS data
  void OnGps(const localization::Gps& message);
  // Upon receiving INS status data
  void OnInsStat(const drivers::gnss::InsStat& msg);

  bool is_first_gps_msg_;

  Eigen::Vector3d last_position_;
  Eigen::Affine3d offset_;
  Eigen::Affine3d extrinsics_;

  // the complete pose data;
  std::map<double, Eigen::Affine3d> gps_poses_;
  // the complete cloud data;
  std::vector<pcl::PointCloud<PointXYZIT>> clouds_;

  // to ensure the pose of given timestamp can be found,
  // we pad some redundant clouds
  uint32_t top_redundant_cloud_count_;
  uint32_t bottom_redundant_cloud_count_;
  // if program has took enough clouds
  bool enough_data_;

  // the number of cloud count to take
  uint32_t cloud_count_;
  // the distance between two clouds
  double capture_distance_;

  // latest INS status
  uint32_t position_type_;
};

}  // namespace calibration
}  // namespace apollo

#endif  // MODEULES_CALIBRATION_LIDAR_EX_CHECKER_H_
