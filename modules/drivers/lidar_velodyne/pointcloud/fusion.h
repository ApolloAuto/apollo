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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_FUSION_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_FUSION_H_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/std_msgs/String.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_listener.h"

#include "modules/drivers/lidar_velodyne/pointcloud/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class Fusion {
 public:
  Fusion() {}
  ~Fusion() {}
  bool fusion(const sensor_msgs::PointCloud2Ptr& major_point_cloud,
              std::vector<sensor_msgs::PointCloud2Ptr> slave_point_cloud_vec,
              sensor_msgs::PointCloud2Ptr point_cloud_fusion);

 private:
  bool query_pose_affine_from_tf2(const std::string& target_frame_id,
                                  const std::string& child_frame_id,
                                  Eigen::Affine3d* pose);
  void append_point_cloud(pcl::PointCloud<PointXYZIT>* point_cloud,
                          pcl::PointCloud<PointXYZIT>* point_cloud_add,
                          const Eigen::Affine3d& pose);
};
}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_FUSION_H_
