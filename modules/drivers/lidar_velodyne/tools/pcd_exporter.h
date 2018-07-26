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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_H_

#include <fstream>
#include <list>
#include <string>

#include "boost/filesystem.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "pcl/common/time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_listener.h"

#include "modules/drivers/lidar_velodyne/tools/proto/velodyne_tools_conf.pb.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class PCDExporter {
 public:
  PCDExporter();
  ~PCDExporter();
  bool init(const VelodyneToolsConf &conf);
  /**
   * @brief write pc data to pcd/pcd_pos/stamp file when recieve a msg
   */
  void pcd_writer_callback(const sensor_msgs::PointCloud2 &cloud);

 private:
  VelodyneToolsConf conf_;
  pcl::PCDWriter writer_;

  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // use the seq of a message as index or not
  float time_offset_;
  float loc_threshold_;
  // Message counter, use as index when use_seq_ = false
  unsigned int pc_msg_count_;
  FILE *stamp_file_handle_;
  FILE *pose_file_handle_;

  std::list<sensor_msgs::PointCloud2ConstPtr> queue_;

  /**
   * @brief Query pose
   */
  bool get_pose(const ros::Time &time, Eigen::Matrix4d *pose);

  void write_pcd_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                      const std::string &filename);
  /**
   * @brief Write pose info with the index of message to a file
   */
  int write_pcd_pose_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                          int index);
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_H_
