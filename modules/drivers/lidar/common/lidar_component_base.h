/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef APOLLO_LIDAR_COMPONENT_BASE_H
#define APOLLO_LIDAR_COMPONENT_BASE_H

#include <memory>

#include "modules/drivers/lidar/common/lidar_component_base_impl.h"

namespace apollo {
namespace drivers {
namespace lidar {
template <typename ScanType>
class LidarComponentBase : public LidarComponentBaseImpl<ScanType> {
 public:
  using BaseComponent = LidarComponentBaseImpl<ScanType>;

  virtual ~LidarComponentBase() = default;

  virtual bool Init() = 0;

  virtual void ReadScanCallback(
      const std::shared_ptr<ScanType>& scan_message) = 0;

  bool InitBase(const LidarConfigBase& lidar_config_base) override {
    RETURN_VAL_IF(!this->InitPacket(lidar_config_base), false);
    RETURN_VAL_IF(!this->InitConverter(lidar_config_base), false);
    return true;
  }

  bool WriteScan(const std::shared_ptr<ScanType>& scan_message) {
    return BaseComponent::WriteScan(scan_message);
  }

  std::shared_ptr<PointCloud> AllocatePointCloud() {
    return BaseComponent::AllocatePointCloud();
  }

  bool WritePointCloud(const std::shared_ptr<PointCloud>& point_cloud) {
    return BaseComponent::WritePointCloud(point_cloud);
  }
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo

#endif  // APOLLO_LIDAR_COMPONENT_BASE_H
