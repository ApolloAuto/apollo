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

#include "modules/perception/tool/data_generator/data_generator.h"

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/perception/tool/data_generator/common/data_generator_gflags.h"

namespace apollo {
namespace perception {
namespace data_generator {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::perception::pcl_util::PointCloudPtr;
using apollo::perception::pcl_util::PointXYZIT;
using Eigen::Affine3d;
using Eigen::Matrix4d;

std::string DataGenerator::Name() const {
  return "data_generator";
}

Status DataGenerator::Init() {
  AdapterManager::Init(FLAGS_data_generator_adapter_config_filename);

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  return Status::OK();
}

void DataGenerator::OnTimer(const ros::TimerEvent&) {
  RunOnce();
}

void DataGenerator::RunOnce() {
  AdapterManager::Observe();

  // point cloud
  if (AdapterManager::GetPointCloud()->Empty()) {
    AERROR << "PointCloud is NOT ready.";
    return;
  }
  // localization
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "Localization is NOT ready.";
    return;
  }
  // chassis
  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis is NOT ready.";
    return;
  }

  const auto& point_cloud_msg =
      AdapterManager::GetPointCloud()->GetLatestObserved();
  ADEBUG << "PointCloud: " << point_cloud_msg.header;

  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Localization: " << localization.DebugString();

  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Chassis: " << chassis.DebugString();

  VehicleStateProvider::instance()->Update(localization, chassis);
  AINFO << "VehicleState updated.";

  Process(point_cloud_msg);
}

void DataGenerator::Process(const sensor_msgs::PointCloud2& message) {
  PointCloudPtr cld;
  TransPointCloudMsgToPCL(message, &cld);
  AINFO << "PointCloud size = " << cld->points.size();

  // TODO(All): label point cloud and generate data automatically below
}

Status DataGenerator::Start() {
  constexpr double kDataGeneratorCycleDuration = 0.1;  // in seconds
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kDataGeneratorCycleDuration),
                                  &DataGenerator::OnTimer, this);
  AINFO << "DataGenerator started";
  return Status::OK();
}

void DataGenerator::Stop() {}

void DataGenerator::TransPointCloudMsgToPCL(
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
  for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
    pcl_util::PointXYZIT& pt = in_cloud.points[idx];
    if (!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z) && !isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      ++points_num;
    }
  }
  cloud->points.resize(points_num);
}

bool DataGenerator::GetTrans(const std::string from_coordinate,
                             const std::string to_coordinate,
                             const double query_time, Matrix4d* trans) {
  CHECK_NOTNULL(trans);
  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = AdapterManager::Tf2Buffer();
  const double kTf2BuffSize = 10 / 1000.0;  // buff size
  std::string err_msg;
  if (!tf2_buffer.canTransform(from_coordinate, to_coordinate, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame from " << from_coordinate << " to frame "
           << to_coordinate << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped =
        tf2_buffer.lookupTransform(from_coordinate, to_coordinate, query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }

  Affine3d affine_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_3d);
  *trans = affine_3d.matrix();
  ADEBUG << from_coordinate << " to " << to_coordinate
         << ",  trans = " << *trans;
  return true;
}

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo
