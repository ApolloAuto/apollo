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
  private_nh.param("max_range", _config.max_range, 130.0);
  private_nh.param("min_range", _config.min_range, 0.9);
  private_nh.param("view_direction", _config.view_direction, 0.0);
  private_nh.param("view_width", _config.view_width, 2.0 * M_PI);
  private_nh.param("model", _config.model, std::string("64E_S2"));
  private_nh.param("calibration_online", _config.calibration_online, true);
  private_nh.param("calibration", _config.calibration_file, std::string(""));
  private_nh.param("organized", _config.organized, false);
  private_nh.param("topic_packets", _topic_packets, TOPIC_PACKTES);
  private_nh.param("topic_pointcloud", _topic_pointcloud, TOPIC_POINTCLOUD);
  // we use beijing time by default
  private_nh.param("time_zone", _config.time_zone, 8);
  private_nh.param("queue_size", _queue_size, 10);

  _parser = VelodyneParserFactory::create_parser(_config);
  if (_parser == nullptr) {
    ROS_BREAK();
  }
  _parser->setup();
  // Emphasis no header available in published msg, which enables us to
  // customize header.seq.
  // Learn from
  // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
  // ros::AdvertiseOptions opt =
  //     ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
  //         _topic_pointcloud, _queue_size, &connected, &disconnected,
  //         ros::VoidPtr(), NULL);
  // opt.has_header = false;
  // _velodyne_points_output = node.advertise(opt);
  _pointcloud_pub =
      node.advertise<sensor_msgs::PointCloud2>(_topic_pointcloud, _queue_size);

  // subscribe to VelodyneScan packets
  _velodyne_scan = node.subscribe(
      _topic_packets, _queue_size, &Convert::convert_packets_to_pointcloud,
      (Convert*)this, ros::TransportHints().tcpNoDelay(true));
}

Convert::~Convert() {
  if (_parser != nullptr) {
    delete _parser;
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
  _parser->generate_pointcloud(scan_msg, pointcloud);

  if (pointcloud->empty()) {
    return;
  }

  if (_config.organized) {
    ROS_DEBUG_STREAM("reorder point cloud");
    _parser->order(pointcloud);
  }

  // publish the accumulated cloud message
  _pointcloud_pub.publish(pointcloud);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
