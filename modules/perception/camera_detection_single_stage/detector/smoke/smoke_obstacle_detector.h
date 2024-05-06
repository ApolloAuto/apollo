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

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera_detection_single_stage/detector/smoke/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image_8u.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class SmokeObstacleDetector : public BaseObstacleDetector {
 public:
  SmokeObstacleDetector() = default;
  virtual ~SmokeObstacleDetector() = default;
  /**
   * @brief Init function for SmokeObstacleDetector
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const ObstacleDetectorInitOptions &options) override;
  /**
   * @brief Main part to detect obstacle
   *
   * @param frame
   * @return true
   * @return false
   */
  bool Detect(onboard::CameraFrame *frame) override;

  std::string Name() const override { return "SmokeObstacleDetector"; }

 protected:
  void InitImageOffset(const smoke::ModelParam &model_param);
  void InitImageSize(const smoke::ModelParam &model_param);
  void InitParam(const smoke::ModelParam &model_param);
  void InitObstacleTypes();

  bool Preprocess(const base::Image8U *image,
                  std::shared_ptr<base::Blob<float>> input_blob);

 private:
  smoke::ModelParam model_param_;
  ObstacleDetectorInitOptions options_;
  std::vector<base::ObjectSubType> types_;

  int width_ = 0;
  int height_ = 0;
  int offset_y_ = 0;

  int ori_cycle_ = 1;
  float confidence_threshold_ = 0.f;
  float border_ratio_ = 0.f;

  smoke::MinDims min_dims_;

  std::shared_ptr<base::Image8U> image_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
