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

#ifndef MODULES_DRIVERS_PANDORA_PANDORA_POINTCLOUD_COMPENSATOR_H_
#define MODULES_DRIVERS_PANDORA_PANDORA_POINTCLOUD_COMPENSATOR_H_

#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/time.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <string>

namespace apollo {
namespace drivers {
namespace pandora {

class Compensator {
 public:
  Compensator(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~Compensator() {}

 private:
  /**
  * @brief get pointcloud2 msg, compensate it,publish pointcloud2 after
  * compensator
  */
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  /**
  * @brief get pose affine from tf2 by gps timestamp
  *   novatel-preprocess broadcast the tf2 transfrom.
  */
  bool query_pose_affine_from_tf2(const double& timestamp,
                                  Eigen::Affine3d* pose);
  /**
  * @brief check if message is valid, check width, height, timesatmp.
  *   set timestamp_offset and point data type
  */
  bool check_message(const sensor_msgs::PointCloud2ConstPtr& msg);
  /**
  * @brief motion compensation for point cloud
  */
  template <typename Scalar>
  void motion_compensation(const sensor_msgs::PointCloud2::Ptr& msg,
                           const double timestamp_min,
                           const double timestamp_max,
                           const Eigen::Affine3d& pose_min_time,
                           const Eigen::Affine3d& pose_max_time);
  /**
  * @brief get min timestamp and max timestamp from points in pointcloud2
  */
  inline void get_timestamp_interval(
      const sensor_msgs::PointCloud2ConstPtr& msg, double* timestamp_min,
      double* timestamp_max);
  /**
  * @brief get point field size by sensor_msgs::datatype
  */
  inline uint get_field_size(const int data_type);

  // subsrcibe pandora pointcloud2 msg.
  ros::Subscriber pointcloud_sub_;
  // publish point cloud2 after motion compensation
  ros::Publisher compensation_pub_;
  //   ros::Publisher metastatus_publisher_;
  // tf2 buffer
  tf2_ros::Buffer tf2_buffer_;
  // tf2 transform listener to get transform by gps timestamp.
  tf2_ros::TransformListener tf2_transform_listener_;
  // transform child frame id(world -> child frame)
  std::string child_frame_id_;
  float tf_timeout_;

  // varibes for point fields value, we get point x,y,z by these offset
  int x_offset_;
  int y_offset_;
  int z_offset_;
  int timestamp_offset_;
  uint timestamp_data_size_;

  // topic names
  std::string topic_compensated_pointcloud_;
  std::string topic_pointcloud_;
  // ros queue size for publisher and subscriber
  int queue_size_;
};

}  // namespace pandora
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_PANDORA_PANDORA_POINTCLOUD_COMPENSATOR_H_
