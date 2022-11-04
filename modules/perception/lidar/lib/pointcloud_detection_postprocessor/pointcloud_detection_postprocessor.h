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

#include <vector>
#include <memory>
#include <string>

#include "modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects/pointcloud_get_objects.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointcloudDetectionPostprocessor : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;
  using PointCloudGetObjectsPtr = std::unique_ptr<PointCloudGetObjects>;

 public:
  PointcloudDetectionPostprocessor() = default;

  virtual ~PointcloudDetectionPostprocessor() = default;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool Process(const std::vector<float>& detections,
               const std::vector<int>& labels,
               DataFrame* data_frame);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  PointCloudGetObjectsPtr pointcloud_get_objects_;
};  // class PointcloudDetectionPostprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
