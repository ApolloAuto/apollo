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

#include "modules/perception/common/base/object.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/perception_benchmark.pb.h"

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

  static bool SerializeBenchmarkMsg(
    double timestamp, uint64_t lidar_timestamp,
    int seq_num,
    const std::vector<base::ObjectPtr>& objects,
    PerceptionBenchmarkFrame* obstacles);

  static bool SerializeLidarFrameMsg(
    double timestamp, uint64_t lidar_timestamp,
    int seq_num, const Eigen::Affine3d& pose,
    const std::vector<base::ObjectPtr>& objects,
    PerceptionBenchmarkFrame* obstacles, bool use_lidar_cooridinate = true);

 private:
  static bool ConvertObjectToPb(const base::ObjectPtr& object_ptr,
                                PerceptionObstacle* pb_msg);

  static bool ConvertSegmentedObjectToPb(const base::ObjectPtr& object_ptr,
    PerceptionObstacle* pb_msg, const Eigen::Affine3d& pose,
    bool use_lidar_cooridinate = true);
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
