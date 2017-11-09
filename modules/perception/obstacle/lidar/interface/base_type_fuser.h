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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TYPE_FUSER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TYPE_FUSER_H_

#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct TypeFuserOptions {
    double timestamp = 0.0;
};

class BaseTypeFuser {
 public:
  BaseTypeFuser() {}
  virtual ~BaseTypeFuser() {}

  virtual bool init() = 0;

  virtual bool fuse_type(
  	const TypeFuserOptions& options,
  	std::vector<ObjectPtr>* objects) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTypeFuser);
};

REGISTER_REGISTERER(BaseTypeFuser);
#define REGISTER_TYPEFUSER(name) REGISTER_CLASS(BaseTypeFuser, name)

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TYPE_FUSER_H_