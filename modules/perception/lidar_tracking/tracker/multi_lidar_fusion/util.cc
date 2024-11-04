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

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/util.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

bool JudgeBlindTrafficCone(const MlfTrackDataConstPtr& track_data,
    double frame_timestamp, const Eigen::Vector3d& local_to_global_offset,
    Eigen::Affine3d& lidar2world_pose, Eigen::Affine3d& lidar2novatel_pose) {
    TrackedObjectConstPtr latest_object = track_data->GetLatestObject().second;
    if (!FLAGS_need_reserve_blind_cone || latest_object == nullptr) {
        return false;
    }
    // reserve condition: TrafficCone
    // reserve condition: static
    // reserve condition: within reserve time
    double time_diff = frame_timestamp - track_data->GetLatestObject().first;
    if (latest_object->output_velocity.head<2>().norm() > 0.01 ||
        latest_object->object_ptr->sub_type != base::ObjectSubType::TRAFFICCONE
        || time_diff >= FLAGS_cone_reserve_time) {
        return false;
    }

    // reserve condition: front
    Eigen::Vector3d world_center = Eigen::Vector3d(
        latest_object->center(0) + local_to_global_offset(0),
        latest_object->center(1) + local_to_global_offset(1),
        latest_object->center(2) + local_to_global_offset(2));

    // from world to lidar to imu
    Eigen::Vector3d lidar_center = Eigen::Vector3d(1, 0, 0);
    lidar_center = lidar2world_pose.inverse() * world_center;

    Eigen::Vector3d imu_center = Eigen::Vector3d(1, 0, 0);
    imu_center = lidar2novatel_pose * lidar_center;

    if (imu_center(0) <= FLAGS_cone_x_front &&
        imu_center(0) >= FLAGS_cone_x_back &&
        imu_center(1) <= FLAGS_cone_y_front &&
        imu_center(1) >= FLAGS_cone_y_back) {
        AINFO << "[JudgeBlindTrafficCone] track_id: " << track_data->track_id_
              << " loc: (x) " << imu_center(0) << " (y) " << imu_center(1)
              << " reserve. time_diff: " << time_diff;
        return true;
    }
    return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
