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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_APP_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_APP_H_

#include <string>

#include "modules/drivers/lidar_velodyne/tools/pcd_exporter.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/apollo_app.h"
#include "sensor_msgs/PointCloud2.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class PcdApp : public apollo::common::ApolloApp {
 public:
  PcdApp() : exporter_(nullptr) {}
  virtual ~PcdApp() = default;

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  PCDExporter* exporter_;
  void OnPointCloud(const sensor_msgs::PointCloud2& message);
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_PCD_EXPORTER_APP_H_
