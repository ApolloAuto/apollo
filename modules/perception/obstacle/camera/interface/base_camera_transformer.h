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

// The base class of transforming camera space objects into other defined
// 3D spaces, like world space or ego-car space

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_TRANSFORMER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_TRANSFORMER_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"

namespace apollo {
namespace perception {

class BaseCameraTransformer {
 public:
  BaseCameraTransformer() {}
  virtual ~BaseCameraTransformer() {}

  virtual bool Init() = 0;

  // @brief: Transform 3D position of objects into targeted space
  // @param [in/out] objects : object lists with 3D positions in camera space,
  // which get transformed into targeted 3D space
  virtual bool Transform(
      std::vector<std::shared_ptr<VisualObject>>* objects) = 0;

  virtual bool SetExtrinsics(const Eigen::Matrix<double, 4, 4>& extrinsics) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseCameraTransformer);
};

REGISTER_REGISTERER(BaseCameraTransformer);
#define REGISTER_CAMERA_TRANSFORMER(name) \
  REGISTER_CLASS(BaseCameraTransformer, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_TRANSFORMER_H_
