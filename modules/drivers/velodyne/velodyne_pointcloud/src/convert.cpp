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

#include "velodyne_pointcloud/convert.h"

#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/advertise_options.h>

namespace apollo {
namespace drivers {
namespace velodyne {

// void disconnected(const ros::SingleSubscriberPublisher&) {}
// void connected(const ros::SingleSubscriberPublisher&) {}

void Convert::init(ros::NodeHandle& node, ros::NodeHandle& private_nh) {
  private_nh.param("max_range", config_.max_range, 130.0);
  private_nh.param("min_range", config_.min_range, 0.9);
  private_nh.param("view_direction", config_.view_direction, 0.0);
  private_nh.param("view_width", config_.view_width, 2.0 * M_PI);
  private_nh.param("model", config_.model, std::string("64E_S2"));
  private_nh.param("calibration_online", config_.calibration_online, true);
  private_nh.param("calibration", config_.calibration_file, std::string(""));
  private_nh.param("organized", config_.organized, false);
  private_nh.param("topic_packets", topic_packets_, TOPIC_PACKTES);
  private_nh.param("topic_pointcloud", topic_pointcloud_, TOPIC_POINTCLOUD);
  // we use beijing time by default
  private_nh.param("queue_size", queue_size_, 10);

  parser_ = VelodyneParserFactory::create_parser(config_);
  if (parser_ == nullptr) {
    ROS_BREAK();
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
  pointcloud_pub_ =
      node.advertise<sensor_msgs::PointCloud2>(topic_pointcloud_, queue_size_);

  // subscribe to VelodyneScan packets
  velodyne_scan_ = node.subscribe(
      topic_packets_, queue_size_, &Convert::convert_packets_to_pointcloud,
      (Convert*)this, ros::TransportHints().tcpNoDelay(true));
}

Convert::~Convert() {
  if (parser_ != nullptr) {
    delete parser_;
  }
}

/** @brief Callback for raw scan messages. */
void Convert::convert_packets_to_pointcloud(
    const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msg) {
  ROS_INFO_ONCE("********************************************************");
  ROS_INFO_ONCE("Start convert velodyne packets to pointcloud");
  ROS_INFO_ONCE("********************************************************");
  ROS_DEBUG_STREAM(scan_msg->header.seq);

  VPointCloud::Ptr pointcloud(new VPointCloud());
  parser_->generate_pointcloud(scan_msg, pointcloud);

  if (pointcloud->empty()) {
    return;
  }

  if (config_.organized) {
    ROS_DEBUG_STREAM("reorder point cloud");
    parser_->order(pointcloud);
  }

  // publish the accumulated cloud message
  pointcloud_pub_.publish(pointcloud);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
