/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"

// Eigen 3.3.7: #define ALIVE (0)
// fastrtps: enum ChangeKind_t { ALIVE, ... };
#if defined(ALIVE)
#undef ALIVE
#endif

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar_fusion_and_compensator/proto/config.pb.h"

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace lidar {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::PointCloud;

class FusionAndCompensatorComponent : public Component<PointCloud> {
public:
    bool Init() override;
    bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;

private:
    bool FusionAndCompensator(
            const std::shared_ptr<PointCloud> source,
            const uint64_t& timestamp_min,
            const uint64_t& timestamp_max,
            std::shared_ptr<PointCloud> target);

    bool IsExpired(const std::shared_ptr<PointCloud>& target, const std::shared_ptr<PointCloud>& source);
    bool QueryPoseAffineFromTF2(
            const uint64_t& timestamp,
            const std::string& target_frame_id,
            const std::string& source_frame_id,
            Eigen::Affine3d* pose);

    bool IsFilteredPoint(const Eigen::Vector3f& point, const std::string& frame_id) const;

    /**
     * @brief get min timestamp and max timestamp from points in pointcloud2
     */
    void GetTimestampInterval(
            const std::vector<std::shared_ptr<PointCloud>>& point_clouds,
            uint64_t* timestamp_min,
            uint64_t* timestamp_max);
    /**
     * @brief compensation main method
     */
    void MotionCompensation(
            const std::shared_ptr<PointCloud>& source,
            const uint64_t& timestamp_min,
            const uint64_t& timestamp_max,
            const Eigen::Affine3d& world_tf_min_time,
            const Eigen::Affine3d& world_tf_max_time,
            const Eigen::Affine3f& lidar_tf,
            std::shared_ptr<PointCloud> target);

    FusionAndCompensatorConfig conf_;
    apollo::transform::Buffer* tf2_buffer_ptr_ = nullptr;
    std::shared_ptr<Writer<PointCloud>> writer_;
    std::vector<std::shared_ptr<Reader<PointCloud>>> readers_;

    size_t pool_size_ = 10;
    size_t pool_index_ = 0;
    size_t reserved_point_size_ = 500000;
    uint64_t seq_num_ = 1;

    std::map<
            std::string,
            Eigen::Affine3f,
            std::less<std::string>,
            Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>
            static_tf_map_;
    std::map<std::string, const FilterConfig*> filter_map_;
};

CYBER_REGISTER_COMPONENT(FusionAndCompensatorComponent)
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
