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

#include <memory>
#include <string>

#include "modules/perception/base/object.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/object_sequence.h"

namespace apollo {
namespace perception {
namespace lidar {

struct TypeFusionInitOption {};

struct TypeFusionOption {};

class BaseOneShotTypeFusion {
 public:
  virtual bool Init(const TypeFusionInitOption& option) = 0;
  virtual bool TypeFusion(const TypeFusionOption& option,
                          std::shared_ptr<perception::base::Object> object) = 0;
  virtual std::string Name() const = 0;
};

PERCEPTION_REGISTER_REGISTERER(BaseOneShotTypeFusion);
#define PERCEPTION_REGISTER_ONESHOTTYPEFUSION(name) \
  PERCEPTION_REGISTER_CLASS(BaseOneShotTypeFusion, name)

class BaseSequenceTypeFusion {
 public:
  typedef ObjectSequence::TrackedObjects TrackedObjects;

 public:
  virtual bool Init(const TypeFusionInitOption& option) = 0;
  virtual bool TypeFusion(const TypeFusionOption& option,
                          TrackedObjects* tracked_objects) = 0;
  virtual std::string Name() const = 0;
};

PERCEPTION_REGISTER_REGISTERER(BaseSequenceTypeFusion);
#define PERCEPTION_REGISTER_SEQUENCETYPEFUSION(name) \
  PERCEPTION_REGISTER_CLASS(BaseSequenceTypeFusion, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
