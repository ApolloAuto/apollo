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

#ifndef MODULES_DRIVERS_VELODYNE_PARSER_COMPENSATOR_H_
#define MODULES_DRIVERS_VELODYNE_PARSER_COMPENSATOR_H_

#include <Eigen/Eigen>
#include <memory>
#include <string>

#include "cybertron/cybertron.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"
#include "cybertron/tf2_cybertron/buffer.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/velodyne/parser/const_variables.h"
#include "modules/drivers/velodyne/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cybertron::tf2_cybertron::Buffer;
using apollo::drivers::PointCloud;
using apollo::drivers::velodyne::config::Config;

class Compensator {
 public:
  explicit Compensator(const Config& velodyne_config);
  virtual ~Compensator() {}

  bool motion_compensation(const std::shared_ptr<const PointCloud>& msg,
                           std::shared_ptr<PointCloud> msg_compensated);

 private:
  /**
   * @brief get pose affine from tf2 by gps timestamp
   *   novatel-preprocess broadcast the tf2 transfrom.
   */
  bool query_pose_affine_from_tf2(const uint64_t& timestamp, void* pose,
                                  const std::string& child_frame_id);

  /**
   * @brief motion compensation for point cloud
   */
  void motion_compensation(const std::shared_ptr<const PointCloud>& msg,
                           std::shared_ptr<PointCloud> msg_compensated,
                           const uint64_t timestamp_min,
                           const uint64_t timestamp_max,
                           const Eigen::Affine3d& pose_min_time,
                           const Eigen::Affine3d& pose_max_time);
  /**
   * @brief get min timestamp and max timestamp from points in pointcloud2
   */
  inline void get_timestamp_interval(
      const std::shared_ptr<const PointCloud>& msg, uint64_t* timestamp_min,
      uint64_t* timestamp_max);

  bool is_valid(const Eigen::Vector3d& point);

  std::shared_ptr<Buffer> tf2_buffer_ptr_;
  Config config_;
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_PARSER_COMPENSATOR_H_
