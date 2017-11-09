/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_FILTER_BASE_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_FILTER_BASE_FILTER_H_

#include <Eigen/Core>

#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

class BaseFilter {
 public:
  BaseFilter() {
    name_ = "BaseFilter";
  };
  virtual ~BaseFilter() {};

  virtual void Initialize(const Object &state) = 0;
  virtual Eigen::Vector4d Predict(double time_diff) = 0;
  std::string name() {
    return name_;
  }
  virtual Eigen::Vector4d UpdateWithObject(const Object &new_object, const double time_diff) = 0;
  virtual Eigen::Matrix4d GetCovarianceMatrix() = 0;
 protected:
  std::string name_;
};

} // namesapce perception
} // namesapce apollo

#endif // MODULES_PERCEPTION_OBSTACLE_RADAR_FILTER_BASE_FILTER_H_
