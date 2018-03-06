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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_LANE_SHARED_DATA_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_LANE_SHARED_DATA_H_

#include <string>

#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
namespace perception {

class LaneSharedData : public CommonSharedData<LaneObjects> {
 public:
  LaneSharedData() = default;
  virtual ~LaneSharedData() = default;

  std::string name() const override { return "LaneSharedData"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(LaneSharedData);
};

REGISTER_SHAREDDATA(LaneSharedData);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_LANE_SHARED_DATA_H_
