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

struct ClassifierInitOptions {};

struct ClassifierOptions {};

class BaseClassifier {
 public:
  BaseClassifier() = default;

  virtual ~BaseClassifier() = default;

  virtual bool Init(
      const ClassifierInitOptions& options = ClassifierInitOptions()) = 0;

  // @brief: classify object list, and fill type in object.
  // @param [in]: options
  // @param [in/out]: object list
  virtual bool Classify(const ClassifierOptions& options,
                        LidarFrame* frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseClassifier);
};  // class BaseClassifier

PERCEPTION_REGISTER_REGISTERER(BaseClassifier);
#define PERCEPTION_REGISTER_CLASSIFIER(name) \
  PERCEPTION_REGISTER_CLASS(BaseClassifier, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
