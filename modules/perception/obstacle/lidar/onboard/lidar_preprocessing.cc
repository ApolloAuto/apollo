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

#include <Eigen/Core>
#include <string>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/lidar/onboard/lidar_preprocessing.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/common/geometry_util.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointIndices;
using pcl_util::PointIndicesPtr;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::string;
using sensor_msgs::PointCloud2;

std::string LidarPreprocessing::Name() const {
  return "LidarPreprocessing";
};

bool LidarPreprocessing::Init() {
  /// init config
  if (!InitConfig()) {
    AERROR << "Failed to init config.";
    return false;
  }
  /// init algorithm plugin
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init algorithm plugin.";
    return false;
  }
  return true;
}

bool LidarPreprocessing::Proc(
    const PointCloud2& message, LidarPredetectionData* data) {
  AINFO << "in LidarPreprocessing proc";
  return true;
}

bool LidarPreprocessing::InitConfig() {
  return true;
}

bool LidarPreprocessing::InitAlgorithmPlugin() {
  /// get roi filter instance
  /*roi_filter_.reset(
      BaseROIFilterRegisterer::get_instance_by_name(onboard_roi_filter_));
  if (roi_filter_ == nullptr) {
    AERROR << "Failed to get instance: " << onboard_roi_filter_;
    return false;
  }

  /// init roi filter
  if (!roi_filter_->Init()) {
    AERROR << "Failed to init roi filter: " << roi_filter_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, roi_filter_: "
           << roi_filter_->name();*/
  return true;
}

void LidarPreprocessing::trans_pointcloud_to_pcl(
        const ::sensor_msgs::PointCloud2& in_msg,
        PointCloudPtr* out_cloud) {
    // transform from ros to pcl
    pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
    //FIXME: there is no ros_pcl lib yet in platform
    //pcl::fromROSMsg(in_msg, in_cloud);
    // transform from xyzit to xyzi
    PointCloudPtr& cloud = *out_cloud;
    cloud->header = in_cloud.header;
    cloud->width = in_cloud.width;
    cloud->height = in_cloud.height;
    cloud->is_dense = in_cloud.is_dense;
    cloud->sensor_origin_ = in_cloud.sensor_origin_;
    cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
    cloud->points.resize(in_cloud.points.size());
    for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
        cloud->points[idx].x = in_cloud.points[idx].x;
        cloud->points[idx].y = in_cloud.points[idx].y;
        cloud->points[idx].z = in_cloud.points[idx].z;
        cloud->points[idx].intensity = in_cloud.points[idx].intensity;
    }
}

bool LidarPreprocessing::GetVelodyneWorldTrans(double timestamp,
                                               Matrix4d* trans) {
    return true;
}

}  // namespace perception
}  // namespace apollo
