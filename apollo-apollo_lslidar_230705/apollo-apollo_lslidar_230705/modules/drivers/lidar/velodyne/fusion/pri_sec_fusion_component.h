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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"

// Eigen 3.3.7: #define ALIVE (0)
// fastrtps: enum ChangeKind_t { ALIVE, ... };
#if defined(ALIVE)
#undef ALIVE
#endif

#include "modules/drivers/lidar/proto/velodyne_config.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::PointCloud;

class PriSecFusionComponent : public Component<PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;

 private:
  bool Fusion(std::shared_ptr<PointCloud> target,
              std::shared_ptr<PointCloud> source);
  bool IsExpired(const std::shared_ptr<PointCloud>& target,
                 const std::shared_ptr<PointCloud>& source);
  bool QueryPoseAffine(const std::string& target_frame_id,
                       const std::string& source_frame_id,
                       Eigen::Affine3d* pose);
  void AppendPointCloud(std::shared_ptr<PointCloud> point_cloud,
                        std::shared_ptr<PointCloud> point_cloud_add,
                        const Eigen::Affine3d& pose);

  FusionConfig conf_;
  apollo::transform::Buffer* buffer_ptr_ = nullptr;
  std::shared_ptr<Writer<PointCloud>> fusion_writer_;
  std::vector<std::shared_ptr<Reader<PointCloud>>> readers_;
};

CYBER_REGISTER_COMPONENT(PriSecFusionComponent)
}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
