/******************************************************************************
 * Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** @file
    This ROS nodelet transforms raw Velodyne 3D LIDAR packets to a
    PointCloud2 in the /odom frame.
*/

#ifndef MODULES_DRIVERS_VELODYN_DRIVER_TRANSFORM_NODELET_H_
#define MODULES_DRIVERS_VELODYN_DRIVER_TRANSFORM_NODELET_H_

#include "ros/ros.h"

#include "modules/drivers/lidar_velodyne/pointcloud/transform.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class TransformNodelet {
 public:
  TransformNodelet() = default;
  ~TransformNodelet() = default;
  virtual void OnInit();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<Transform> tf_;
};

/** @brief Nodelet initialization. */
void TransformNodelet::OnInit() { tf_.reset(new Transform(nh_, private_nh_)); }

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYN_DRIVER_TRANSFORM_NODELET_H_
