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

#include "modules/perception/lidar/lib/interface/base_classifier.h"

namespace apollo {
namespace perception {
namespace lidar {

class DummyClassifier : public BaseClassifier {
 public:
  DummyClassifier() = default;

  virtual ~DummyClassifier() = default;

  bool Init(
      const ClassifierInitOptions& options = ClassifierInitOptions()) override;

  // @brief: classify object list, and fill type in object.
  // @param [in]: options
  // @param [in/out]: object list
  bool Classify(const ClassifierOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "DummyClassifier"; }
};  // class DummyClassifier

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
