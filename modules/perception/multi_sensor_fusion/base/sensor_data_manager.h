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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/multi_sensor_fusion/base/sensor.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

class SensorDataManager {
 public:
  /**
   * @brief Initialization
   *
   * @return true
   * @return false
   */
  bool Init();

  /**
   * @brief Reset the data
   *
   */
  void Reset();

  /**
   * @brief Add the data frame, which contains the obstacles detected
   * by the sensor
   *
   * @param frame_ptr Sensor detection input data frame
   */
  void AddSensorMeasurements(const base::FrameConstPtr& frame_ptr);

  /**
   * @brief Is the data frame detected by lidar
   *
   * @param frame_ptr
   * @return true detected by lidar
   * @return false not detected by lidar
   */
  bool IsLidar(const base::FrameConstPtr& frame_ptr);

  /**
   * @brief Is the data frame detected by radar
   *
   * @param frame_ptr
   * @return true detected by radar
   * @return false not detected by radar
   */
  bool IsRadar(const base::FrameConstPtr& frame_ptr);

  /**
   * @brief Is the data frame detected by camera
   *
   * @param frame_ptr
   * @return true detected by camera
   * @return false not detected by camera
   */
  bool IsCamera(const base::FrameConstPtr& frame_ptr);

  /**
   * @brief Get the latest sensor frames
   *
   * @param timestamp
   * @param sensor_id
   * @param frames
   */
  void GetLatestSensorFrames(double timestamp, const std::string& sensor_id,
                             std::vector<SensorFramePtr>* frames) const;

  /**
   * @brief Get the latest sensor frames
   *
   * @param timestamp
   * @param frames
   */
  void GetLatestFrames(double timestamp,
                       std::vector<SensorFramePtr>* frames) const;

  /**
   * @brief Get the Pose object
   *
   * @param sensor_id
   * @param timestamp
   * @param pose
   * @return true
   * @return false
   */
  bool GetPose(const std::string& sensor_id, double timestamp,
               Eigen::Affine3d* pose) const;

  /**
   * @brief Get the Camera Intrinsic object
   *
   * @param sensor_id
   * @return base::BaseCameraModelPtr
   */
  base::BaseCameraModelPtr GetCameraIntrinsic(
      const std::string& sensor_id) const;

 private:
  bool inited_ = false;
  std::unordered_map<std::string, SensorPtr> sensors_;

  const algorithm::SensorManager* sensor_manager_ = nullptr;

  FRIEND_TEST(SensorDataManagerTest, test);
  DECLARE_SINGLETON(SensorDataManager)
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
