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
#include <vector>
#include <string>

#include "modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample/pointcloud_down_sample.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/stage.h"

#include "modules/perception/pipeline/proto/stage/pointcloud_detection_preprocessor_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointcloudDetectionPreprocessor : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;
  using PointCloudDownSamplePtr = std::unique_ptr<PointCloudDownSample>;

 public:
  PointcloudDetectionPreprocessor() = default;

  virtual ~PointcloudDetectionPreprocessor() = default;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool Process(DataFrame* data_frame,
               std::vector<float>* points_array,
               int* num_points);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  PointcloudDetectionPreprocessorConfig
      pointcloud_detection_preprocessor_config_;

  PointCloudDownSamplePtr pointcloud_downsample_;
};  // class PointcloudDetectionPreprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
