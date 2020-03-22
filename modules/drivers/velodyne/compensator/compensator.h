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

#include "Eigen/Eigen"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::PointCloud;
using apollo::transform::Buffer;

class Compensator {
 public:
  explicit Compensator(const CompensatorConfig& config) : config_(config) {}
  virtual ~Compensator() {}

  bool MotionCompensation(const std::shared_ptr<const PointCloud>& msg,
                          std::shared_ptr<PointCloud> msg_compensated);

 private:
  /**
   * @brief get pose affine from tf2 by gps timestamp
   *   novatel-preprocess broadcast the tf2 transfrom.
   */
  bool QueryPoseAffineFromTF2(const uint64_t& timestamp, void* pose,
                              const std::string& child_frame_id);

  /**
   * @brief motion compensation for point cloud
   */
  void MotionCompensation(const std::shared_ptr<const PointCloud>& msg,
                          std::shared_ptr<PointCloud> msg_compensated,
                          const uint64_t timestamp_min,
                          const uint64_t timestamp_max,
                          const Eigen::Affine3d& pose_min_time,
                          const Eigen::Affine3d& pose_max_time);
  /**
   * @brief get min timestamp and max timestamp from points in pointcloud2
   */
  inline void GetTimestampInterval(const std::shared_ptr<const PointCloud>& msg,
                                   uint64_t* timestamp_min,
                                   uint64_t* timestamp_max);

  bool IsValid(const Eigen::Vector3d& point);

  Buffer* tf2_buffer_ptr_ = transform::Buffer::Instance();
  CompensatorConfig config_;
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
