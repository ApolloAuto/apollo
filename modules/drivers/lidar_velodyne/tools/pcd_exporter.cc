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

#include <string>
#include <list>

#include "modules/drivers/lidar_velodyne/tools/pcd_exporter.h"
#include "modules/drivers/lidar_velodyne/tools/proto/velodyne_tools_conf.pb.h"
#include "modules/common/log.h"
#include "modules/common/adapters/adapter_manager.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using apollo::common::util::GetProtoFromFile;

PCDExporter::PCDExporter()
    : time_offset_(0.1),
      loc_threshold_(0.1),
      pc_msg_count_(1),
      stamp_file_handle_(NULL),
      pose_file_handle_(NULL) {
}

PCDExporter::~PCDExporter() {
  if (stamp_file_handle_ != NULL) {
    fclose(stamp_file_handle_);
  }
  if (pose_file_handle_ != NULL) {
    fclose(pose_file_handle_);
  }
}

bool PCDExporter::init(const VelodyneToolsConf& conf) {
  conf_ = conf;
  tf_buffer_ptr_ = boost::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer());
  tf_listener_ptr_ = boost::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*tf_buffer_ptr_));
  if (conf_.pcd_folder() == "") {
    AERROR << "no pcd_folder input";
    return false;
  }
  AINFO << "pcd_folder : " << conf_.pcd_folder();

  // check output directory, if not exist create it
  if (!boost::filesystem::exists(conf_.pcd_folder())) {
    AINFO << "The directory" << conf_.pcd_folder()
        << "is not exists, create now";
    if (boost::filesystem::create_directory(conf_.pcd_folder())) {
      AINFO << "Create directory success.";
    } else {
      AERROR << "Create directory failed! ";
      return false;
    }
  }

  if (boost::filesystem::exists(conf_.pose_file())) {
    boost::filesystem::remove(conf_.pose_file());
    AINFO << "Remove the legacy pose file in pcd folder";
  }
  if ((pose_file_handle_ = fopen(conf_.pose_file().c_str(), "a")) == NULL) {
    AERROR << "Cannot open pose file!";
    return false;
  }

  if (boost::filesystem::exists(conf_.stamp_file())) {
    boost::filesystem::remove(conf_.stamp_file());
    AINFO << "Remove the legacy stamp file in pcd folder";
  }
  if ((stamp_file_handle_ = fopen(conf_.stamp_file().c_str(), "a")) == NULL) {
    AERROR << "Cannot open stamp file!";
    return false;
  }

  return true;
}

void PCDExporter::write_pcd_file(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                 const std::string &filename) {
  AINFO << "export pcd filename :" << filename;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);
  writer_.writeBinaryCompressed(filename, pcl_cloud);
}

int PCDExporter::write_pcd_pose_file(
    const sensor_msgs::PointCloud2::ConstPtr &msg, int index) {
  double time = msg->header.stamp.toSec();

  Eigen::Matrix4d pose;
  if (!get_pose(msg->header.stamp, &pose)) {
    return -1;
  }

  if (conf_.skip_static_frames()) {
    // check pose around time, if transform is small, then skip the frame
    double time2 = time - time_offset_;
    ros::Time query_time(time2);
    Eigen::Matrix4d pose2;
    if (!get_pose(query_time, &pose2)) {
      return -1;
    }

    // compute transform
    Eigen::Matrix4d transform = pose.inverse() * pose2;
    double loc = transform.topRightCorner(3, 1).norm();
    if (loc <= loc_threshold_) {
      AINFO << "[SUCCESS] : skip static frames...";
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

bool PCDExporter::get_pose(const ros::Time &time, Eigen::Matrix4d* pose) {
  if (!tf_buffer_ptr_->canTransform("world", conf_.child_frame_id(), time,
                                    ros::Duration(0.1))) {
    AERROR << "Cannot get correspondence pose!";
    return false;
  }

  try {
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform =
        tf_buffer_ptr_->lookupTransform("world", conf_.child_frame_id(), time);
    Eigen::Affine3d affine;
    tf::transformMsgToEigen(stamped_transform.transform, affine);
    *pose = affine.matrix();
    if (pose->allFinite()) {
      AERROR << "Invalid pose, has NaN or +/- INF value";
      return false;
    }
  } catch (tf2::TransformException &ex) {
    AERROR << ex.what();
    return false;
  }
  return true;
}

void PCDExporter::pcd_writer_callback(
    const sensor_msgs::PointCloud2& cloud) {
  sensor_msgs::PointCloud2Ptr pt(new sensor_msgs::PointCloud2());
  *pt = cloud;
  queue_.push_back(pt);
  for (auto iter = queue_.begin(); iter != queue_.end();) {
    sensor_msgs::PointCloud2ConstPtr &msg = *iter;

    int index = conf_.use_seq() ? msg->header.seq : pc_msg_count_;
    std::string pcd_filename = conf_.pcd_folder() + "/" +
        boost::lexical_cast<std::string>(index) + ".pcd";

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

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
