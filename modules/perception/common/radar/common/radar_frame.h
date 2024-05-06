/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/radar_point_cloud.h"
#include "modules/perception/common/base/sensor_meta.h"

namespace apollo {
namespace perception {
namespace radar4d {

struct RadarFrame {
  // point cloud
  std::shared_ptr<base::AttributeRadarPointCloud<base::RadarPointF>> cloud;
  // world point cloud
  std::shared_ptr<base::AttributeRadarPointCloud<base::RadarPointD>>
    world_cloud;
  // timestamp
  double timestamp = 0.0;
  // radar to world pose
  Eigen::Affine3d radar2world_pose = Eigen::Affine3d::Identity();
  // radar to world pose
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
  // radar to
  Eigen::Affine3d radar2novatel_extrinsics = Eigen::Affine3d::Identity();
  // segmented objects
  std::vector<std::shared_ptr<base::Object>> segmented_objects;
  // tracked objects
  std::vector<std::shared_ptr<base::Object>> tracked_objects;
  // point cloud roi indices
  base::PointIndices roi_indices;
  // sensor info
  base::SensorInfo sensor_info;
  // reserve string
  std::string reserve;

  void Reset() {
    if (cloud) {
      cloud->clear();
    }
    if (world_cloud) {
      world_cloud->clear();
    }
    timestamp = 0.0;
    radar2world_pose = Eigen::Affine3d::Identity();
    novatel2world_pose = Eigen::Affine3d::Identity();
    segmented_objects.clear();
    tracked_objects.clear();
    roi_indices.indices.clear();
  }

  void FilterPointCloud(
    base::RadarPointCloud<base::RadarPointF> *filtered_cloud,
    const std::vector<uint32_t> &indices) {
    if (cloud && filtered_cloud) {
      filtered_cloud->CopyRadarPointCloudExclude(*cloud, indices);
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // struct RadarFrame

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
