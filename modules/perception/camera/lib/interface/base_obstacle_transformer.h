/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

#include "modules/perception/camera/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace camera {

struct ObstacleTransformerInitOptions : public BaseInitOptions {};

struct ObstacleTransformerOptions {
  /*
  float min_dimension_val = 0.2f;
  bool check_dimension = true;
  */
};

class BaseObstacleTransformer {
 public:
  BaseObstacleTransformer() = default;

  virtual ~BaseObstacleTransformer() = default;

  virtual bool Init(const ObstacleTransformerInitOptions& options =
                        ObstacleTransformerInitOptions()) = 0;

  // @brief: transform 2d obstacle to 3D obstacle.
  // @param [in]: options
  // @param [in/out]: frame
  // 3D information of obstacle should be filled, required.
  virtual bool Transform(const ObstacleTransformerOptions& options,
                         CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseObstacleTransformer(const BaseObstacleTransformer&) = delete;
  BaseObstacleTransformer& operator=(const BaseObstacleTransformer&) = delete;
};  // class BaseObstacleTransformer

PERCEPTION_REGISTER_REGISTERER(BaseObstacleTransformer);
#define REGISTER_OBSTACLE_TRANSFORMER(name) \
  PERCEPTION_REGISTER_CLASS(BaseObstacleTransformer, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
