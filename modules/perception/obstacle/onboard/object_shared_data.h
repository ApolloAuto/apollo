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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H_

#include <string>

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
namespace perception {

#define OBJECT_SHARED_DATA(data_name)                        \
  class data_name : public CommonSharedData<SensorObjects> { \
   public:                                                   \
    data_name() : CommonSharedData<SensorObjects>() {}       \
    virtual ~data_name() {}                                  \
    std::string name() const override {                      \
      return #data_name;                                     \
    }                                                        \
                                                             \
   private:                                                  \
    DISALLOW_COPY_AND_ASSIGN(data_name);                     \
  }

OBJECT_SHARED_DATA(LidarObjectData);
OBJECT_SHARED_DATA(RadarObjectData);
OBJECT_SHARED_DATA(CameraObjectData);
OBJECT_SHARED_DATA(CIPVObjectData);

REGISTER_SHAREDDATA(LidarObjectData);
REGISTER_SHAREDDATA(RadarObjectData);
REGISTER_SHAREDDATA(CameraObjectData);
REGISTER_SHAREDDATA(CIPVObjectData);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H_
