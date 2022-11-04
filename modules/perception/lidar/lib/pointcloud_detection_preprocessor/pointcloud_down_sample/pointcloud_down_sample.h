/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include <algorithm>
#include <deque>
#include <vector>
#include <string>

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/point.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudDownSample : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  PointCloudDownSample() = default;
  explicit PointCloudDownSample(const PluginConfig& plugin_config);
  virtual ~PointCloudDownSample() = default;

  bool Init(const PluginConfig& plugin_config) override;
  bool Process(DataFrame* data_frame, std::vector<float>* points_array,
               int* num_points_result);

  bool IsEnabled() const override { return enable_; }
  std::string Name() const override { return name_; }

 private:
  bool DownSample(LidarFrame* lidar_frame, std::vector<float>* points_array,
                  int* num_points_result);
  void FuseCloud(const base::PointFCloudPtr& out_cloud_ptr,
                 const std::deque<base::PointDCloudPtr>& fuse_clouds);



void CloudToArray(const base::PointFCloudPtr& pc_ptr,
                                         float* out_points_array,
                                         const float normalizing_factor);

  std::vector<int> GenerateIndices(int start_index, int size, bool shuffle);
  // to store lidar_frame
  LidarFrame* lidar_frame_ref_ = nullptr;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<base::AttributePointCloud<base::PointD>>
      original_world_cloud_;

  base::PointFCloudPtr cur_cloud_ptr_;

  // time statistics
  double cloud_to_array_time_ = 0.0;
  double downsample_time_ = 0.0;
  double fuse_time_ = 0.0;
  double shuffle_time_ = 0.0;

  float x_min_range_;
  float x_max_range_;
  float y_min_range_;
  float y_max_range_;
  float z_min_range_;
  float z_max_range_;

  bool enable_downsample_pointcloud_;
  bool enable_downsample_beams_;

  std::deque<base::PointDCloudPtr> prev_world_clouds_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
