/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <memory>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "modules/perception/radar4d_detection/interface/base_preprocessor.h"

namespace apollo {
namespace perception {
namespace radar4d {

class RadarPreprocessor : public BasePreprocessor {
 public:
  RadarPreprocessor() : BasePreprocessor(), rcs_offset_(0.0) {}
  virtual ~RadarPreprocessor() {}

  /**
   * @brief Init RadarPreprocessor config
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const PreprocessorInitOptions& options) override;

  /**
   * @brief Process radar point cloud.
   *
   * @param message raw data obtained from radar driver
   * @param options preprocess options
   * @param frame radar frame with preprocessed point cloud
   * @return true
   * @return false
   */
  bool Preprocess(
    const std::shared_ptr<apollo::drivers::OculiiPointCloud const>& message,
    const PreprocessorOptions& options,
    RadarFrame* frame) override;

  /**
   * @brief The name of the RadarPreprocessor
   *
   * @return std::string
   */
  std::string Name() const override { return "RadarPreprocessor"; }

 private:
  bool TransformCloud(const base::RadarPointFCloudPtr& local_cloud,
                      const Eigen::Affine3d& pose,
                      base::RadarPointDCloudPtr world_cloud) const;

  float CalCompensatedVelocity(
      const base::RadarPointF& point,
      const PreprocessorOptions& options);

  float rcs_offset_ = 0.0f;
  bool filter_naninf_points_ = true;
  bool filter_high_z_points_ = true;
  float z_threshold_ = 5.0f;
  static const float kPointInfThreshold;

  static int current_idx_;
  static std::unordered_map<int, int> local2global_;

  DISALLOW_COPY_AND_ASSIGN(RadarPreprocessor);
};

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
