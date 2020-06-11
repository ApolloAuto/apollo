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

#include <vector>

#include "cyber/time/time.h"
#include "modules/perception/base/object.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class MsgSerializer {
 public:
  MsgSerializer() = default;
  ~MsgSerializer() = default;

  static bool SerializeMsg(double timestamp, uint64_t lidar_timestamp,
                           int seq_num,
                           const std::vector<base::ObjectPtr>& objects,
                           const apollo::common::ErrorCode& error_code,
                           PerceptionObstacles* obstacles);

 private:
  static bool ConvertObjectToPb(const base::ObjectPtr& object_ptr,
                                PerceptionObstacle* pb_msg);
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
