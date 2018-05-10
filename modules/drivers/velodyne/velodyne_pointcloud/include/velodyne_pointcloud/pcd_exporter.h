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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_PCD_EXPORTER_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_PCD_EXPORTER_H_

#include "const_variables.h"

#include <pcl/common/time.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_listener.h"

namespace apollo {
namespace drivers {
namespace velodyne {

class PCDExporter {
 public:
  PCDExporter(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PCDExporter();
  void init();
  /**
   * @brief write pc data to pcd/pcd_pos/stamp file when recieve a msg
   */
  void pcd_writer_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

 private:
  // An exist folder to save pcd files
  std::string pcd_folder_;
  // File name of stamp_file_;
  std::string stamp_file_;
  FILE *stamp_file_handle_;
  // File name of pcd_pose file
  std::string pose_file_;
  FILE *pose_file_handle_;
  pcl::PCDWriter writer_;
  // Default "velodyne"
  std::string child_frame_id_;
  ros::Subscriber sub_;

  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  bool skip_static_frames_;
  // use the seq of a message as index or not
  bool use_seq_as_index_;
  float time_offset_;
  float loc_threshold_;
  // Message counter, use as index when use_seq_ = false
  unsigned int pc_msg_count_;
  std::string topic_pointcloud_;
  int queue_size_;

  std::list<sensor_msgs::PointCloud2ConstPtr> queue_;

  /**
   * @brief Query pose
   */
  bool get_pose(const ros::Time &time, Eigen::Matrix4d &pose);

  void write_pcd_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                      const std::string &filename);
  /**
   * @brief Write pose info with the index of message to a file
   */
  int write_pcd_pose_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                          int index);
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_PCD_EXPORTER_H_
