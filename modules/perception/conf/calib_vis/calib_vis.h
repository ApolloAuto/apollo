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

#ifndef MODEULES_PERCEPTION_PERCEPTION_CALIBVIS_H_
#define MODEULES_PERCEPTION_PERCEPTION_CALIBVIS_H_

#include <string>
#include <map>
#include <vector>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/localization/proto/gps.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */ 
namespace apollo {
namespace perception {

class CalibVis : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void LoadExtrinsics();
  void VisualizeClouds();

  // Upon receiving point cloud data
  void OnPointCloud(const sensor_msgs::PointCloud2& message);
  // Upon receiving GPS data
  void OnGps(const ::apollo::localization::Gps& message);
  // Upon receiving INS status data
  void OnInsStat();

  bool is_first_gps_msg_;

  Eigen::Vector3d last_position_;
  Eigen::Affine3d offset_;
  Eigen::Affine3d extrinsics_;
  
  std::map<double, Eigen::Affine3d> gps_poses_;
  std::vector<pcl::PointCloud<pcl_util::PointXYZIT> > clouds_;

  uint32_t top_redundant_cloud_count_;
  uint32_t bottom_redundant_cloud_count_;
  bool enough_data_;

  uint32_t cloud_count_;
  double capture_distance_;
  
  // latest INS status                                                                                                                                                                                                        
  uint32_t position_type_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_PERCEPTION_CALIBVIS_H_
