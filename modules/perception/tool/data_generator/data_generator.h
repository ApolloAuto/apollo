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
#include <vector>

#include "Eigen/Core"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/perception/tool/data_generator/proto/config.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/util/factory.h"
#include "modules/perception/common/pcl_types.h"
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
  DataGenerator() = default;
  ~DataGenerator() { delete data_file_; }

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void RunOnce();

  void OnTimer(const ros::TimerEvent&);

  void RegisterSensors();

  bool Process();

  ros::Timer timer_;

  std::ofstream* data_file_ = nullptr;
  int num_data_frame_ = 0;

  common::util::Factory<SensorConfig::SensorId, Sensor,
                        Sensor* (*)(const SensorConfig& config)>
      sensor_factory_;
  google::protobuf::RepeatedPtrField<SensorConfig> sensor_configs_;

  std::vector<Sensor*> sensors_;

  DataGeneratorInfo data_generator_info_;
};

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_TOOL_DATE_GENERATOR_DATA_GENERATOR_
