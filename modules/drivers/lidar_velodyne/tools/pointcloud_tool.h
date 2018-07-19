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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_POINTCLOUD_DUMP_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_POINTCLOUD_DUMP_H_

#include <fstream>
#include <string>

#include "pcl_ros/point_cloud.h"
#include "ros/include/sensor_msgs/PointCloud2.h"
#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"
#include "ros/ros.h"

#include "modules/drivers/lidar_velodyne/tools/proto/velodyne_tools_conf.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/pointcloud/compensator.h"
#include "modules/drivers/lidar_velodyne/pointcloud/converter.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

// save msg to file: file name is file_prefix_ + msg.seq + .msg
class PointCloudTool : public apollo::common::ApolloApp {
 public:
  PointCloudTool() : converter_(nullptr), compensator_(nullptr) {}
  virtual ~PointCloudTool() {}

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void pointcloud_dump(const sensor_msgs::PointCloud2& message);
  void pointcloud_compensate(const sensor_msgs::PointCloud2& message);
  void pointcloud_convert(const velodyne_msgs::VelodyneScanUnified& message);

  VelodyneToolsConf conf_;
  Converter* converter_;
  Compensator* compensator_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_TOOLS_POINTCLOUD_DUMP_H_
