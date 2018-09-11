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

#include "modules/drivers/velodyne/parser/convert.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::velodyne::config::Config;
using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::PointCloud;

void Convert::init(const Config& velodyne_config) {
  // private_nh.param("max_range", config_.max_range, 130.0);
  config_ = velodyne_config;
  // we use beijing time by default
  // private_nh.param("queue_size", queue_size_, 10);

  parser_.reset(VelodyneParserFactory::create_parser(config_));
  if (parser_.get() == nullptr) {
    // ROS_BREAK();
    AFATAL << "Create parser failed.";
    return;
  }
  parser_->setup();
  // Emphasis no header available in published msg, which enables us to
  // customize header.seq.
  // Learn from
  // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
  // ros::AdvertiseOptions opt =
  //     ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
  //         topic_pointcloud_, queue_size_, &connected, &disconnected,
  //         ros::VoidPtr(), NULL);
  // opt.has_header = false;
  // velodyne_points_output_ = node.advertise(opt);
  // pointcloud_pub_ =
  //     node.advertise<sensor_msgs::PointCloud2>(topic_pointcloud_, queue_size_);
  //
  // subscribe to VelodyneScan packets
  // velodyne_scan_ = node.subscribe(
  //     topic_packets_, queue_size_, &Convert::convert_packets_to_pointcloud,
  //     (Convert*)this, ros::TransportHints().tcpNoDelay(true));
}

/** @brief Callback for raw scan messages. */
void  Convert::convert_packets_to_pointcloud(
    const std::shared_ptr<VelodyneScan>& scan_msg,
    std::shared_ptr<PointCloud>& point_cloud) {
  // ROS_INFO_ONCE("********************************************************");
  // ROS_INFO_ONCE("Start convert velodyne packets to pointcloud");
  // ROS_INFO_ONCE("********************************************************");
  // ROS_DEBUG_STREAM(scan_msg->header.seq);
  ADEBUG << "Convert scan msg seq " << scan_msg->header().sequence_num();

  parser_->generate_pointcloud(scan_msg, point_cloud);

  if (point_cloud == nullptr || point_cloud->point_size() == 0) {
    AERROR << "point cloud has no point";
    return;
  }

  if (config_.organized()) {
    ADEBUG << "reorder point cloud";
    parser_->order(point_cloud);
  }

  if (config_.organized()) {
    parser_->order(point_cloud);
    point_cloud->set_is_dense(false);
  } else {
    point_cloud->set_is_dense(true);
  }

  // publish the accumulated cloud message
  // pointcloud_pub_.publish(pointcloud);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
