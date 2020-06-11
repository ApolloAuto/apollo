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

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct GroundDetectorInitOptions {};

struct GroundDetectorOptions {};

class BaseGroundDetector {
 public:
  BaseGroundDetector() = default;

  virtual ~BaseGroundDetector() = default;

  virtual bool Init(const GroundDetectorInitOptions& options =
                        GroundDetectorInitOptions()) = 0;

  // @brief: detect ground points from point cloud.
  // @param [in]: options
  // @param [in/out]: frame
  // non_ground_indices should be filled, required,
  // label field of point cloud can be filled, optional,
  virtual bool Detect(const GroundDetectorOptions& options,
                      LidarFrame* frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseGroundDetector);
};  // class BaseGroundDetector

PERCEPTION_REGISTER_REGISTERER(BaseGroundDetector);
#define PERCEPTION_REGISTER_GROUNDDETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseGroundDetector, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
