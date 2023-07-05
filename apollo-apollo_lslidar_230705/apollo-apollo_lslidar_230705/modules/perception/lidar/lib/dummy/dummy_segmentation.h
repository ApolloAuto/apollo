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

#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class DummySegmentation : public BaseLidarDetector {
 public:
  DummySegmentation() = default;

  virtual ~DummySegmentation() = default;

  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  // @brief: segment point cloud and get objects.
  // @param [in]: options
  // @param [in/out]: frame
  // segmented_objects should be filled, required,
  // label field of point cloud can be filled, optional,
  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }
};  // class DummySegmentation

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
