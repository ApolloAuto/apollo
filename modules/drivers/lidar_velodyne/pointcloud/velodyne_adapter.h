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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_VELODYNE_ADAPTER_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_VELODYNE_ADAPTER_H_

#include "ros/include/sensor_msgs/PointCloud2.h"
#include "ros/include/std_msgs/String.h"
#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {
using apollo::common::adapter::AdapterManager;
using velodyne_msgs::VelodyneScanUnifiedPtr;

class VelodyneAdapter {
 public:
  static bool CheckVelodyne() {
    // TODO(All): implements here
    return true;
  }

  static void PublishVelodyneScanByIndex(
      uint32_t index, velodyne_msgs::VelodyneScanUnifiedPtr scan) {
    switch (index) {
      case 0:
        AdapterManager::PublishVelodyneScanDense(*(scan.get()));
        break;
      case 1:
        AdapterManager::PublishVelodyneScanSparse1(*(scan.get()));
        break;
      case 2:
        AdapterManager::PublishVelodyneScanSparse2(*(scan.get()));
        break;
      case 3:
        AdapterManager::PublishVelodyneScanSparse3(*(scan.get()));
        break;
      default:
        AERROR << "no index match!";
        break;
    }
    return;
  }
  static void PublishPointCloudRawByIndex(
      uint32_t index, sensor_msgs::PointCloud2Ptr pointcloud) {
    switch (index) {
      case 0:
        AdapterManager::PublishPointCloudDenseRaw(*(pointcloud.get()));
        break;
      case 1:
        AdapterManager::PublishPointCloudSparseRaw1(*(pointcloud.get()));
        break;
      case 2:
        AdapterManager::PublishPointCloudSparseRaw2(*(pointcloud.get()));
        break;
      case 3:
        AdapterManager::PublishPointCloudSparseRaw3(*(pointcloud.get()));
        break;
      default:
        AERROR << "no index match!";
        break;
    }
    return;
  }
  static void PublishPointCloudByIndex(
      uint32_t index, sensor_msgs::PointCloud2Ptr com_pointcloud) {
    switch (index) {
      case 0:
        AdapterManager::PublishPointCloudDense(*com_pointcloud);
        break;
      case 1:
        AdapterManager::PublishPointCloudSparse1(*com_pointcloud);
        break;
      case 2:
        AdapterManager::PublishPointCloudSparse2(*com_pointcloud);
        break;
      case 3:
        AdapterManager::PublishPointCloudSparse3(*com_pointcloud);
        break;
      default:
        AERROR << "no index match!";
        break;
    }
    return;
  }
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_POINTCLOUD_VELODYNE_ADAPTER_H_
