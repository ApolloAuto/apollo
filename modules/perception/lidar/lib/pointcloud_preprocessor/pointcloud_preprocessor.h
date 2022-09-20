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

#include <string>
#include <memory>

#include "modules/perception/lidar/lib/interface/base_pointcloud_preprocessor.h"
#include "modules/perception/pipeline/proto/stage/pointcloud_preprocessor_config.pb.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudPreprocessor : public BasePointCloudPreprocessor {
 public:
  PointCloudPreprocessor() = default;
  virtual ~PointCloudPreprocessor() = default;

  bool Init(const PointCloudPreprocessorInitOptions& options =
                PointCloudPreprocessorInitOptions()) override;

  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) const override;

  bool Preprocess(const PointCloudPreprocessorOptions& options,
                  LidarFrame* frame) const override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

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


  PointcloudPreprocessorConfig pointcloud_preprocessor_config_;
};  // class PointCloudPreprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
