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

#include "modules/drivers/lidar_velodyne/pointcloud/fusion.h"

#include "pcl/common/time.h"
#include "ros/this_node.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using ::apollo::common::adapter::AdapterManager;

bool Fusion::fusion(
    const sensor_msgs::PointCloud2Ptr& major_point_cloud,
    std::vector<sensor_msgs::PointCloud2Ptr> slave_point_cloud_vec,
    sensor_msgs::PointCloud2Ptr point_cloud_fusion) {
  Eigen::Affine3d pose;
  pcl::PointCloud<PointXYZIT> pcl_major_point_cloud;
  pcl::fromROSMsg(*major_point_cloud, pcl_major_point_cloud);
  for (auto slave_point_cloud : slave_point_cloud_vec) {
    if (!query_pose_affine_from_tf2(major_point_cloud->header.frame_id,
                                    slave_point_cloud->header.frame_id,
                                    &pose)) {
      return false;
    }
    pcl::PointCloud<PointXYZIT> pcl_slave_point_cloud;
    pcl::fromROSMsg(*slave_point_cloud, pcl_slave_point_cloud);
    append_point_cloud(&pcl_major_point_cloud, &pcl_slave_point_cloud, pose);
  }
  pcl::toROSMsg(pcl_major_point_cloud, *point_cloud_fusion);
  return true;
}
bool Fusion::query_pose_affine_from_tf2(const std::string& target_frame_id,
                                        const std::string& child_frame_id,
                                        Eigen::Affine3d* pose) {
  std::string err_string;
  const tf2_ros::Buffer& tf2_buffer = AdapterManager::Tf2Buffer();
  if (!tf2_buffer.canTransform(target_frame_id, child_frame_id, ros::Time(0),
                               ros::Duration(0.02), &err_string)) {
    AERROR << "Can not find transform. "
           << "target_id:" << target_frame_id << " frame_id:" << child_frame_id
           << " Error info: " << err_string;
    return false;
  }

  geometry_msgs::TransformStamped stamped_transform;

  try {
    stamped_transform = tf2_buffer.lookupTransform(
        target_frame_id, child_frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  tf::transformMsgToEigen(stamped_transform.transform, *pose);
  return true;
}
void Fusion::append_point_cloud(pcl::PointCloud<PointXYZIT>* point_cloud,
                                pcl::PointCloud<PointXYZIT>* point_cloud_add,
                                const Eigen::Affine3d& pose) {
  uint32_t origin_width = point_cloud->width;
  if (std::isnan(pose(0, 0))) {
    for (auto& point : point_cloud_add->points) {
      point_cloud->push_back(point);
    }
  } else {
    for (auto& point : point_cloud_add->points) {
      if (std::isnan(point.x)) {
        point_cloud->push_back(point);
      } else {
        PointXYZIT point_new;
        point_new.intensity = point.intensity;
        point_new.timestamp = point.timestamp;
        Eigen::Matrix<float, 3, 1> pt(point.x, point.y, point.z);
        point_new.x = static_cast<float>(
            pose(0, 0) * pt.coeffRef(0) + pose(0, 1) * pt.coeffRef(1) +
            pose(0, 2) * pt.coeffRef(2) + pose(0, 3));
        point_new.y = static_cast<float>(
            pose(1, 0) * pt.coeffRef(0) + pose(1, 1) * pt.coeffRef(1) +
            pose(1, 2) * pt.coeffRef(2) + pose(1, 3));
        point_new.z = static_cast<float>(
            pose(2, 0) * pt.coeffRef(0) + pose(2, 1) * pt.coeffRef(1) +
            pose(2, 2) * pt.coeffRef(2) + pose(2, 3));
        point_cloud->push_back(point_new);
      }
    }
  }
  // update height,TODO: more check
  point_cloud->width = origin_width;
  int new_height = point_cloud->size() / point_cloud->width;
  point_cloud->height = new_height;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
