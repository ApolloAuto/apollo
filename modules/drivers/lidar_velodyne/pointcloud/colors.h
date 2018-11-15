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

/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** @file
    Interface for converting a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.
    @author Jack O'Quin
*/

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_COLORS_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_COLORS_H_

#include "modules/drivers/lidar_velodyne/pointcloud/point_types.h"

#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "ros/ros.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

// shorter names for point cloud types in this namespace
typedef apollo::drivers::lidar_velodyne::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class RingColors {
 public:
  RingColors(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~RingColors() {}

 private:
  void convertPoints(const VPointCloud::ConstPtr &inMsg);

  ros::Subscriber input_;
  ros::Publisher output_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_COLORS_H_
