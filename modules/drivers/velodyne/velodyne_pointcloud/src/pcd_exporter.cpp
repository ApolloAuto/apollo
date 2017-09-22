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
    : _time_offset(0.1),
      _loc_threshold(0.1),
      _pc_msg_count(1),
      _stamp_file_handle(NULL),
      _pose_file_handle(NULL) {
  private_nh.param("pcd_folder", _pcd_folder, std::string(""));
  private_nh.param("stamp_file", _stamp_file, std::string(""));
  private_nh.param("pose_file", _pose_file, std::string(""));
  private_nh.param("skip_static_frames", _skip_static_frames, false);
  private_nh.param("child_frame_id", _child_frame_id, std::string("velodyne"));
  private_nh.param("use_seq", _use_seq_as_index, false);
  private_nh.param("topic_pointcloud", _topic_pointcloud,
                   velodyne::TOPIC_POINTCLOUD);
  private_nh.param("queue_size", _queue_size, 10);

  _sub = node.subscribe(_topic_pointcloud, _queue_size,
                        &PCDExporter::pcd_writer_callback, (PCDExporter *)this);
}

PCDExporter::~PCDExporter() {
  if (_stamp_file_handle != NULL) {
    fclose(_stamp_file_handle);
  }
  if (_pose_file_handle != NULL) {
    fclose(_pose_file_handle);
  }
}

void PCDExporter::init() {
  _tf_buffer_ptr = boost::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer());
  _tf_listener_ptr = boost::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*_tf_buffer_ptr));
  if (_pcd_folder == "") {
    ROS_ERROR_STREAM("No pcd_folder input");
    ROS_BREAK();
  }

  ROS_INFO_STREAM("pcd_folder :" << _pcd_folder);
  // check output directory, if not exist create it
  if (!boost::filesystem::exists(_pcd_folder)) {
    ROS_INFO_STREAM("The directory " << _pcd_folder
                                     << " is not exists, create now");
    if (boost::filesystem::create_directory(_pcd_folder)) {
      ROS_INFO("Create directory success.");
    } else {
      ROS_ERROR("Create directory failed! ");
      ROS_BREAK();
    }
  }

  if (boost::filesystem::exists(_pose_file)) {
    boost::filesystem::remove(_pose_file);
    ROS_INFO_STREAM("Remove the legacy pose file in pcd folder");
  }
  if ((_pose_file_handle = fopen(_pose_file.c_str(), "a")) == NULL) {
    ROS_ERROR_STREAM("Cannot open pose file!");
    ROS_BREAK();
  }

  if (boost::filesystem::exists(_stamp_file)) {
    boost::filesystem::remove(_stamp_file);
    ROS_INFO("Remove the legacy stamp file in pcd folder");
  }
  if ((_stamp_file_handle = fopen(_stamp_file.c_str(), "a")) == NULL) {
    ROS_ERROR_STREAM("Cannot open stamp file!");
    ROS_BREAK();
  }
}

void PCDExporter::write_pcd_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                 const std::string &filename) {
  ROS_INFO_STREAM("export pcd filename :" << filename);
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);
  _writer.writeBinaryCompressed(filename, pcl_cloud);
}

int PCDExporter::write_pcd_pose_file(
    const sensor_msgs::PointCloud2::ConstPtr &msg, int index) {
  double time = msg->header.stamp.toSec();

  Eigen::Matrix4d pose;
  if (!get_pose(msg->header.stamp, pose)) {
    return -1;
  }

  if (_skip_static_frames) {
    // check pose around time, if transform is small, then skip the frame
    double time2 = time - _time_offset;
    ros::Time query_time(time2);
    Eigen::Matrix4d pose2;
    if (!get_pose(query_time, pose2)) {
      return -1;
    }

    // compute transform
    Eigen::Matrix4d transform = pose.inverse() * pose2;
    double loc = transform.topRightCorner(3, 1).norm();
    if (loc <= _loc_threshold) {
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

  fprintf(_pose_file_handle, "%d %lf %lf %lf %lf %lf %lf %lf %lf\n", index,
          time, t(0), t(1), t(2), quat.x(), quat.y(), quat.z(), quat.w());
  return 0;
}

bool PCDExporter::get_pose(const ros::Time &time, Eigen::Matrix4d &pose) {
  if (!_tf_buffer_ptr->canTransform("world", _child_frame_id, time,
                                    ros::Duration(0.1))) {
    ROS_ERROR_STREAM("Cannot get correspondence pose!");
    return false;
  }

  try {
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform =
        _tf_buffer_ptr->lookupTransform("world", _child_frame_id, time);
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
  _queue.push_back(cloud);
  for (auto iter = _queue.begin(); iter != _queue.end();) {
    sensor_msgs::PointCloud2ConstPtr &msg = *iter;

    int index = _use_seq_as_index ? msg->header.seq : _pc_msg_count;
    std::string pcd_filename =
        _pcd_folder + "/" + boost::lexical_cast<std::string>(index) + ".pcd";

    int ret = write_pcd_pose_file(msg, index);
    if (ret == 0) {
      write_pcd_file(msg, pcd_filename);
      fprintf(_stamp_file_handle, "%d %lf\n", index, msg->header.stamp.toSec());
      ++_pc_msg_count;
      iter = _queue.erase(iter);
      // Once there is a successful compensated PCD, all PCDs before that should
      // be discarded from the queue
      std::list<sensor_msgs::PointCloud2ConstPtr>::iterator iter_inside =
          _queue.begin();
      while (iter_inside != iter) {
        iter_inside = _queue.erase(iter_inside);
      }
    } else if (ret == -1) {
      ++iter;
    }
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
