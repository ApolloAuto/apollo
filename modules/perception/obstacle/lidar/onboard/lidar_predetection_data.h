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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PREPROCESSING_DATA_H
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PREPROCESSING_DATA_H

#include <Eigen/Core>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

struct LidarPredetectionData {
    LidarPredetectionData() {
        lidar2world_pose = Eigen::Matrix4d::Zero();
    }

    double timestamp = 0.0;
    SeqId seq_num = 0;
    pcl_util::PointCloudPtr cloud;
    pcl_util::PointCloudPtr roi_cloud;
    pcl_util::PointIndices non_ground_indices;
    Eigen::Matrix4d lidar2world_pose;
};

}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_OBSTACLE_ONBOARD_PREPROCESSING_SHARED_DATA_H
