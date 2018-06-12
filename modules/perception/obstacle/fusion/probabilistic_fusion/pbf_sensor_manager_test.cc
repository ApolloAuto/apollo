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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

#include <gtest/gtest.h>
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

TEST(PbfSensorManagerTest, pbf_sensor_frame_manage_test) {
  PbfSensorManager *sensor_manager = PbfSensorManager::instance();
  EXPECT_TRUE(sensor_manager != nullptr);
  SensorObjects lidar_frame;
  lidar_frame.timestamp = 1234567891.01;
  lidar_frame.sensor_type = SensorType::VELODYNE_64;
  std::string lidar_name = GetSensorType(lidar_frame.sensor_type);
  std::shared_ptr<Object> obj(new Object());
  lidar_frame.objects.push_back(obj);
  SensorObjects radar_frame;
  radar_frame.timestamp = 1234567891.11;
  radar_frame.sensor_type = SensorType::RADAR;
  std::string radar_name = GetSensorType(radar_frame.sensor_type);
  std::shared_ptr<Object> radar_obj(new Object());
  radar_frame.objects.push_back(radar_obj);
  sensor_manager->AddSensorMeasurements(lidar_frame);
  EXPECT_TRUE(sensor_manager->GetSensor(lidar_name) != nullptr);
  Eigen::Matrix4d pose;
  const double kEpsilon = 1e-3;
  EXPECT_TRUE(sensor_manager->GetPose(lidar_name, lidar_frame.timestamp,
                                      kEpsilon, &pose));
  sensor_manager->AddSensorMeasurements(radar_frame);
  std::vector<PbfSensorFramePtr> frames;
  sensor_manager->GetLatestFrames(radar_frame.timestamp, &frames);
  EXPECT_EQ(frames.size(), 2);
}

}  // namespace perception
}  // namespace apollo
