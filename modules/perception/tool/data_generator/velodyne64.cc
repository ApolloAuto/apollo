/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/perception/tool/data_generator/velodyne64.h"

#include <cmath>

namespace apollo {
namespace perception {
namespace data_generator {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::perception::pcl_util::PointCloud;
using apollo::perception::pcl_util::PointCloudPtr;
using apollo::perception::pcl_util::PointD;
using apollo::perception::pcl_util::PointXYZIT;
using Eigen::Affine3d;
using Eigen::Matrix4d;

bool Velodyne64::Process() {
  const auto& point_cloud_msg =
      AdapterManager::GetPointCloud()->GetLatestObserved();
  ADEBUG << "PointCloud: " << point_cloud_msg.header;
  ProcessPointCloudData(point_cloud_msg);
  return true;
}

bool Velodyne64::ProcessPointCloudData(
    const sensor_msgs::PointCloud2& message) {
  PointCloudPtr cld(new PointCloud);
  TransPointCloudMsgToPCL(message, &cld);
  AINFO << "PointCloud size = " << cld->points.size();

  std::shared_ptr<Matrix4d> velodyne_to_novatel_trans =
      std::make_shared<Matrix4d>();
  if (!GetTrans(FLAGS_novatel_frame_name, FLAGS_velodyne64_frame_name,
                message.header.stamp.toSec(),
                velodyne_to_novatel_trans.get())) {
    AERROR << "Fail to transform velodyne64 to novatel at time: "
           << message.header.stamp.toSec();
    return false;
  }

  if (!TransformPointCloudToWorld(velodyne_to_novatel_trans, &cld)) {
    AERROR << "Fail to transform point cloud to world.";
    return false;
  }

  for (size_t i = 0; i < cld->points.size(); ++i) {
    auto& point = cld->points[i];
    ss_ << point.x << " " << point.y << " " << point.x << " " << point.intensity
        << ", ";
  }
  ss_ << "\n";
  data_ = ss_.str();
  return true;
}

void Velodyne64::TransPointCloudMsgToPCL(
    const sensor_msgs::PointCloud2& cloud_msg, PointCloudPtr* cloud_pcl) {
  // transform from ros to pcl
  pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
  pcl::fromROSMsg(cloud_msg, in_cloud);
  // transform from xyzit to xyzi
  PointCloudPtr& cloud = *cloud_pcl;
  cloud->header = in_cloud.header;
  cloud->width = in_cloud.width;
  cloud->height = in_cloud.height;
  cloud->is_dense = in_cloud.is_dense;
  cloud->sensor_origin_ = in_cloud.sensor_origin_;
  cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
  cloud->points.resize(in_cloud.points.size());
  size_t points_num = 0;
  for (size_t i = 0; i < in_cloud.size(); ++i) {
    pcl_util::PointXYZIT& pt = in_cloud.points[i];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        !std::isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      ++points_num;
    }
  }
  cloud->points.resize(points_num);
}

bool Velodyne64::GetTrans(const std::string& to_frame,
                          const std::string& from_frame,
                          const double query_time, Matrix4d* trans) {
  CHECK_NOTNULL(trans);
  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = AdapterManager::Tf2Buffer();
  const double kTf2BuffSize = 10 / 1000.0;  // buff size
  std::string err_msg;
  if (!tf2_buffer.canTransform(to_frame, from_frame, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame from " << from_frame << " to frame "
           << to_frame << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped =
        tf2_buffer.lookupTransform(to_frame, from_frame, query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }

  Affine3d affine_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_3d);
  *trans = affine_3d.matrix();
  ADEBUG << from_frame << " to " << to_frame << ",  trans = " << *trans;
  return true;
}

bool Velodyne64::TransformPointCloudToWorld(
    std::shared_ptr<Matrix4d> velodyne_trans, PointCloudPtr* cld) {
  Affine3d affine_3d_trans(*velodyne_trans);
  for (size_t i = 0; i < (*cld)->points.size(); ++i) {
    auto& pt = (*cld)->points[i];
    PointD point = {pt.x, pt.y, pt.z, 0};
    PointD point_world = pcl::transformPoint(point, affine_3d_trans);
    pt.x = point_world.x;
    pt.y = point_world.y;
    pt.z = point_world.z;
  }
  return true;
}

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo
