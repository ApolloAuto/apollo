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
 */

#ifndef MODEULES_PERCEPTION_TOOL_DATE_GENERATOR_DATA_GENERATOR_
#define MODEULES_PERCEPTION_TOOL_DATE_GENERATOR_DATA_GENERATOR_

#include <memory>
#include <string>

#include "sensor_msgs/PointCloud2.h"

#include "Eigen/Core"
#include "ros/include/ros/ros.h"

#include "modules/perception/tool/data_generator/proto/config.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/util/factory.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/tool/data_generator/sensor.h"

/**
 * @namespace apollo::calibration
 * @brief apollo::calibration
 */
namespace apollo {
namespace perception {
namespace data_generator {

class DataGenerator : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void RunOnce();
  bool Process(const sensor_msgs::PointCloud2& message);
  void OnTimer(const ros::TimerEvent&);

  void TransPointCloudMsgToPCL(const sensor_msgs::PointCloud2& cloud_msg,
                               pcl_util::PointCloudPtr* cloud_pcl);
  bool GetTrans(const std::string& to_frame, const std::string& from_frame,
                const double query_time, Eigen::Matrix4d* trans);
  bool TransformPointCloudToWorld(
      std::shared_ptr<Eigen::Matrix4d> velodyne_trans,
      pcl_util::PointCloudPtr* cld);

  void RegisterSensors();

  ros::Timer timer_;

  std::ofstream* data_file_ = nullptr;
  int num_data_frame_ = 0;

  common::util::Factory<SensorConfig::SensorId, Sensor,
                        Sensor* (*)(const SensorConfig& config)>
      sensor_factory_;
  google::protobuf::RepeatedPtrField<SensorConfig> sensor_configs_;
};

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_TOOL_DATE_GENERATOR_DATA_GENERATOR_
