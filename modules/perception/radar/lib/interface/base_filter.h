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

#include "cyber/common/macros.h"

#include "modules/perception/base/object.h"

namespace apollo {
namespace perception {
namespace radar {
class BaseFilter {
 public:
  BaseFilter() : name_("BaseFilter") {}
  virtual ~BaseFilter() {}
  virtual void Init(const base::Object& object) = 0;
  virtual Eigen::VectorXd Predict(double time_diff) = 0;
  virtual Eigen::VectorXd UpdateWithObject(const base::Object& new_object,
                                           double time_diff) = 0;
  virtual void GetState(Eigen::Vector3d* anchor_point,
                        Eigen::Vector3d* velocity) = 0;
  virtual Eigen::Matrix4d GetCovarianceMatrix() = 0;
  std::string Name() { return name_; }

 protected:
  std::string name_;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseFilter);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
