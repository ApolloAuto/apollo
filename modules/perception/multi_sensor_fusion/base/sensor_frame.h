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
#include <vector>

#include "Eigen/Core"

#include "modules/perception/common/base/frame.h"
#include "modules/perception/multi_sensor_fusion/base/base_forward_declaration.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace fusion {

struct SensorFrameHeader {
  base::SensorInfo sensor_info;
  double timestamp = 0.0;
  Eigen::Affine3d sensor2world_pose;

  SensorFrameHeader() = default;
  SensorFrameHeader(const base::SensorInfo& info, double ts,
                    const Eigen::Affine3d& pose)
      : sensor_info(info), timestamp(ts), sensor2world_pose(pose) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class SensorFrame {
 public:
  SensorFrame();

  /**
   * @brief Construct a new Sensor Frame object
   *
   * @param base_frame_ptr
   */
  explicit SensorFrame(const base::FrameConstPtr& base_frame_ptr);

  /**
   * @brief Initialize sensor frame
   *
   * @param base_frame_ptr
   */
  void Initialize(const base::FrameConstPtr& base_frame_ptr);

  /**
   * @brief Initialize sensor frame
   *
   * @param base_frame_ptr
   * @param sensor
   */
  void Initialize(const base::FrameConstPtr& base_frame_ptr,
                  const SensorPtr& sensor);

  /**
   * @brief Get the timestamp of the sensor frame
   *
   * @return double
   */
  inline double GetTimestamp() const { return header_->timestamp; }

  /**
   * @brief Get the pose of the sensor frame
   *
   * @param pose
   * @return true
   * @return false
   */
  inline bool GetPose(Eigen::Affine3d* pose) const {
    if (pose == nullptr) {
      AERROR << "pose is not available";
      return false;
    }
    *pose = header_->sensor2world_pose;
    return true;
  }

  /**
   * @brief Get the Foreground Objects object
   *
   * @return std::vector<SensorObjectPtr>&
   */
  inline std::vector<SensorObjectPtr>& GetForegroundObjects() {
    return foreground_objects_;
  }

  /**
   * @brief Get the Foreground Objects object
   *
   * @return const std::vector<SensorObjectPtr>&
   */
  inline const std::vector<SensorObjectPtr>& GetForegroundObjects() const {
    return foreground_objects_;
  }

  /**
   * @brief Get the Background Objects object
   *
   * @return std::vector<SensorObjectPtr>&
   */
  inline std::vector<SensorObjectPtr>& GetBackgroundObjects() {
    return background_objects_;
  }

  /**
   * @brief Get the Background Objects object
   *
   * @return const std::vector<SensorObjectPtr>&
   */
  inline const std::vector<SensorObjectPtr>& GetBackgroundObjects() const {
    return background_objects_;
  }

  /**
   * @brief Get the sensor id
   *
   * @return std::string
   */
  std::string GetSensorId() const;

  /**
   * @brief Get the sensor type
   *
   * @return base::SensorType
   */
  base::SensorType GetSensorType() const;

  /**
   * @brief Get sensor frame header
   *
   * @return SensorFrameHeaderConstPtr
   */
  SensorFrameHeaderConstPtr GetHeader() const { return header_; }

 private:
  std::vector<SensorObjectPtr> foreground_objects_;
  std::vector<SensorObjectPtr> background_objects_;

  // sensor-specific frame supplements
  base::LidarFrameSupplement lidar_frame_supplement_;
  base::RadarFrameSupplement radar_frame_supplement_;
  base::CameraFrameSupplement camera_frame_supplement_;

  SensorFrameHeaderPtr header_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
