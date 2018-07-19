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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_MULTIPLE_VELODYNE_ADAPTER_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_MULTIPLE_VELODYNE_ADAPTER_H_

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

class MultipleVelodyneAdapter {
 public:
  MultipleVelodyneAdapter();
  ~MultipleVelodyneAdapter();

  static bool CheckMultipleVelodyne() {
    if (nullptr == AdapterManager::GetPointCloudRaw0() ||
        nullptr == AdapterManager::GetPointCloud() ||
        nullptr == AdapterManager::GetVelodyneRaw0()) {
      AERROR << "PointCloudRaw0,PointCloud0,Velodyne0 Adapter not initialized";
      return false;
    }
    return true;
  }

  static void PublishVelodyneRawByIndex(
      uint32_t index, velodyne_msgs::VelodyneScanUnifiedPtr scan) {
    switch (index) {
      case 0:
        AdapterManager::PublishVelodyneRaw0(*(scan.get()));
        break;
      case 1:
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
        AdapterManager::PublishPointCloudRaw0(*(pointcloud.get()));
        break;
      case 1:
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
        AdapterManager::PublishPointCloud(*com_pointcloud);
        break;
      case 1:
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

#endif  // MODULES_DRIVERS_VELODYNE_POINTCLOUD_MULTIPLE_VELODYNE_ADAPTER_H_
