/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <set>
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/object.h"

namespace apollo {
namespace perception {
namespace benchmark {

bool load_pcl_pcds(const std::string& filename, PointCloudPtr cloud_out,
                   const std::string& cloud_type = "xyzit");

bool load_pcl_pcds_xyzit(const std::string& filename, PointCloudPtr cloud_out);

bool load_pcl_pcds_xyzl(const std::string& filename, PointCloudPtr cloud_out);

bool load_frame_objects(const std::string& filename,
                        const std::set<std::string>& black_list,
                        std::vector<ObjectPtr>* objects_out,
                        std::vector<PointCloud>* left_boundary = nullptr,
                        std::vector<PointCloud>* right_boundary = nullptr,
                        std::vector<PointCloud>* road_polygon = nullptr,
                        std::vector<PointCloud>* left_lane_boundary = nullptr,
                        std::vector<PointCloud>* right_lane_boundary = nullptr,
                        PointCloud* cloud = nullptr);

bool load_sensor2world_pose(const std::string& filename,
                            Eigen::Matrix4d* pose_out);

bool save_frame_objects(const std::string& filename,
                        const std::vector<ObjectPtr>& objects,
                        const int frame_id = 0);

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
