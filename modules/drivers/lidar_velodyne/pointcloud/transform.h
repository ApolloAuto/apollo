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

/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** @file
    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /odom frame of reference.
*/

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_TRANSFORM_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_TRANSFORM_H_

#include <string>

#include "dynamic_reconfigure/server.h"
#include "message_filters/subscriber.h"
#include "pcl_ros/impl/transforms.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"

#include "modules/drivers/lidar_velodyne/proto/transform_node_conf.pb.h"

#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/pointcloud/point_types.h"
#include "modules/drivers/lidar_velodyne/pointcloud/rawdata.h"

/** types of point and cloud to work with */
typedef apollo::drivers::lidar_velodyne::VPoint VPoint;
typedef apollo::drivers::lidar_velodyne::VPointCloud VPointCloud;

// instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<VPoint>(
    const std::string &, const VPointCloud &, VPointCloud &,
    const tf::TransformListener &);

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class Transform {
 public:
  Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Transform() {}

 private:
  void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  // boost::shared_ptr<dynamic_reconfigure::Server<TransformNodeConf> > srv_;
  void reconfigure_callback(TransformNodeConf *config);

  const std::string tf_prefix_;
  boost::shared_ptr<RawData> data_;
  message_filters::Subscriber<velodyne_msgs::VelodyneScan> velodyne_scan_;
  tf::MessageFilter<velodyne_msgs::VelodyneScan> *tf_filter_;
  ros::Publisher output_;
  tf::TransformListener listener_;

  /// configuration parameters
  typedef struct {
    std::string frame_id;  ///< target frame ID
  } Config;
  Config config_;

  // Point cloud buffers for collecting points within a packet.  The
  // inPc_ and tfPc_ are class members only to avoid reallocation on
  // every message.
  VPointCloud inPc_;  ///< input packet point cloud
                      //    XYZIRBPointCloud inPc_;
  VPointCloud tfPc_;  ///< transformed packet point cloud
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_TRANSFORM_H_
