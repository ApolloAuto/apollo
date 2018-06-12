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

#ifndef MODULES_PERCEPTION_DATA_GENERATOR_VELODYNE64_H_
#define MODULES_PERCEPTION_DATA_GENERATOR_VELODYNE64_H_

#include <memory>
#include <string>

#include "Eigen/Core"
#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/perception/tool/data_generator/proto/config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/tool/data_generator/common/data_generator_gflags.h"
#include "modules/perception/tool/data_generator/sensor.h"

namespace apollo {
namespace perception {
namespace data_generator {

class Velodyne64 : public Sensor {
 public:
  explicit Velodyne64(const SensorConfig& config) : Sensor(config) {}
  virtual ~Velodyne64() = default;
  bool Process() override;

 private:
  bool ProcessPointCloudData(const sensor_msgs::PointCloud2& message);
  void TransPointCloudMsgToPCL(const sensor_msgs::PointCloud2& cloud_msg,
                               pcl_util::PointCloudPtr* cloud_pcl);
  bool GetTrans(const std::string& to_frame, const std::string& from_frame,
                const double query_time, Eigen::Matrix4d* trans);
  bool TransformPointCloudToWorld(
      std::shared_ptr<Eigen::Matrix4d> velodyne_trans,
      pcl_util::PointCloudPtr* cld);
};

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_DATA_GENERATOR_VELODYNE64_H_
