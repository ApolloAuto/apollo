/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct PointCloudPreprocessorInitOptions {
  std::string sensor_name = "velodyne64";
};

struct PointCloudPreprocessorOptions {
  Eigen::Affine3d sensor2novatel_extrinsics;
};

class PointCloudPreprocessor {
 public:
  PointCloudPreprocessor() = default;

  ~PointCloudPreprocessor() = default;

  bool Init(const PointCloudPreprocessorInitOptions& options =
                PointCloudPreprocessorInitOptions());

  // @brief: preprocess point cloud
  // @param [in]: options
  // @param [in]: point cloud message
  // @param [in/out]: frame
  // cloud should be filled, required,
  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) const;

  // @brief: preprocess point cloud
  // @param [in/out]: frame
  // cloud should be filled, required,
  bool Preprocess(const PointCloudPreprocessorOptions& options,
                  LidarFrame* frame) const;

  std::string Name() const { return "PointCloudPreprocessor"; }

 private:
  bool TransformCloud(const base::PointFCloudPtr& local_cloud,
                      const Eigen::Affine3d& pose,
                      base::PointDCloudPtr world_cloud) const;
  // params
  bool filter_naninf_points_ = true;
  bool filter_nearby_box_points_ = true;
  float box_forward_x_ = 0.0f;
  float box_backward_x_ = 0.0f;
  float box_forward_y_ = 0.0f;
  float box_backward_y_ = 0.0f;
  bool filter_high_z_points_ = true;
  float z_threshold_ = 5.0f;
  static const float kPointInfThreshold;
};  // class PointCloudPreprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
