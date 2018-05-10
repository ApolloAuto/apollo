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

#include "velodyne_pointcloud/pcd_exporter.h"

namespace apollo {
namespace drivers {
namespace velodyne {

PCDExporter::PCDExporter(ros::NodeHandle node, ros::NodeHandle private_nh)
    : time_offset_(0.1),
      loc_threshold_(0.1),
      pc_msg_count_(1),
      stamp_file_handle_(NULL),
      pose_file_handle_(NULL) {
  private_nh.param("pcd_folder", pcd_folder_, std::string(""));
  private_nh.param("stamp_file", stamp_file_, std::string(""));
  private_nh.param("pose_file", pose_file_, std::string(""));
  private_nh.param("skip_static_frames", skip_static_frames_, false);
  private_nh.param("child_frame_id", child_frame_id_, std::string("velodyne"));
  private_nh.param("use_seq", use_seq_as_index_, false);
  private_nh.param("topic_pointcloud", topic_pointcloud_,
                   velodyne::TOPIC_POINTCLOUD);
  private_nh.param("queue_size", queue_size_, 10);

  sub_ = node.subscribe(topic_pointcloud_, queue_size_,
                        &PCDExporter::pcd_writer_callback, (PCDExporter *)this);
}

PCDExporter::~PCDExporter() {
  if (stamp_file_handle_ != NULL) {
    fclose(stamp_file_handle_);
  }
  if (pose_file_handle_ != NULL) {
    fclose(pose_file_handle_);
  }
}

void PCDExporter::init() {
  tf_buffer_ptr_ = boost::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer());
  tf_listener_ptr_ = boost::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*tf_buffer_ptr_));
  if (pcd_folder_ == "") {
    ROS_ERROR_STREAM("No pcd_folder input");
    ROS_BREAK();
  }

  ROS_INFO_STREAM("pcd_folder :" << pcd_folder_);
  // check output directory, if not exist create it
  if (!boost::filesystem::exists(pcd_folder_)) {
    ROS_INFO_STREAM("The directory " << pcd_folder_
                                     << " is not exists, create now");
    if (boost::filesystem::create_directory(pcd_folder_)) {
      ROS_INFO("Create directory success.");
    } else {
      ROS_ERROR("Create directory failed! ");
      ROS_BREAK();
    }
  }

  if (boost::filesystem::exists(pose_file_)) {
    boost::filesystem::remove(pose_file_);
    ROS_INFO_STREAM("Remove the legacy pose file in pcd folder");
  }
  if ((pose_file_handle_ = fopen(pose_file_.c_str(), "a")) == NULL) {
    ROS_ERROR_STREAM("Cannot open pose file!");
    ROS_BREAK();
  }

  if (boost::filesystem::exists(stamp_file_)) {
    boost::filesystem::remove(stamp_file_);
    ROS_INFO("Remove the legacy stamp file in pcd folder");
  }
  if ((stamp_file_handle_ = fopen(stamp_file_.c_str(), "a")) == NULL) {
    ROS_ERROR_STREAM("Cannot open stamp file!");
    ROS_BREAK();
  }
}

void PCDExporter::write_pcd_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                 const std::string &filename) {
  ROS_INFO_STREAM("export pcd filename :" << filename);
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);
  writer_.writeBinaryCompressed(filename, pcl_cloud);
}

int PCDExporter::write_pcd_pose_file(
    const sensor_msgs::PointCloud2::ConstPtr &msg, int index) {
  double time = msg->header.stamp.toSec();

  Eigen::Matrix4d pose;
  if (!get_pose(msg->header.stamp, pose)) {
    return -1;
  }

  if (skip_static_frames_) {
    // check pose around time, if transform is small, then skip the frame
    double time2 = time - time_offset_;
    ros::Time query_time(time2);
    Eigen::Matrix4d pose2;
    if (!get_pose(query_time, pose2)) {
      return -1;
    }

    // compute transform
    Eigen::Matrix4d transform = pose.inverse() * pose2;
    double loc = transform.topRightCorner(3, 1).norm();
    if (loc <= loc_threshold_) {
      ROS_INFO("[SUCCESS] : skip static frames...");
      return 0;
    }
  }

  Eigen::Affine3d affine(pose);
  Eigen::Matrix<double, 3, 1> t;
  t[0] = affine.translation()[0];
  t[1] = affine.translation()[1];
  t[2] = affine.translation()[2];
  Eigen::Quaterniond quat = (Eigen::Quaterniond)affine.linear();

  fprintf(pose_file_handle_, "%d %lf %lf %lf %lf %lf %lf %lf %lf\n", index,
          time, t(0), t(1), t(2), quat.x(), quat.y(), quat.z(), quat.w());
  return 0;
}

bool PCDExporter::get_pose(const ros::Time &time, Eigen::Matrix4d &pose) {
  if (!tf_buffer_ptr_->canTransform("world", child_frame_id_, time,
                                    ros::Duration(0.1))) {
    ROS_ERROR_STREAM("Cannot get correspondence pose!");
    return false;
  }

  try {
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform =
        tf_buffer_ptr_->lookupTransform("world", child_frame_id_, time);
    Eigen::Affine3d affine;
    tf::transformMsgToEigen(stamped_transform.transform, affine);
    pose = affine.matrix();
    if (!pose.allFinite()) {
      ROS_ERROR_STREAM("Invalid pose, has NaN or +/- INF value");
      return false;
    }
  } catch (tf2::TransformException &ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  return true;
}

void PCDExporter::pcd_writer_callback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud) {
  queue_.push_back(cloud);
  for (auto iter = queue_.begin(); iter != queue_.end();) {
    sensor_msgs::PointCloud2ConstPtr &msg = *iter;

    int index = use_seq_as_index_ ? msg->header.seq : pc_msg_count_;
    std::string pcd_filename =
        pcd_folder_ + "/" + boost::lexical_cast<std::string>(index) + ".pcd";

    int ret = write_pcd_pose_file(msg, index);
    if (ret == 0) {
      write_pcd_file(msg, pcd_filename);
      fprintf(stamp_file_handle_, "%d %lf\n", index, msg->header.stamp.toSec());
      ++pc_msg_count_;
      iter = queue_.erase(iter);
      // Once there is a successful compensated PCD, all PCDs before that should
      // be discarded from the queue
      std::list<sensor_msgs::PointCloud2ConstPtr>::iterator iter_inside =
          queue_.begin();
      while (iter_inside != iter) {
        iter_inside = queue_.erase(iter_inside);
      }
    } else if (ret == -1) {
      ++iter;
    }
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
