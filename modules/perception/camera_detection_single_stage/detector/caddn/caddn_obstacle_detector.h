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
#include <vector>

#include "modules/perception/camera_detection_single_stage/detector/caddn/proto/model_param.pb.h"

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image_8u.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class CaddnObstacleDetector : public BaseObstacleDetector {
 public:
  CaddnObstacleDetector() = default;
  virtual ~CaddnObstacleDetector() = default;
  /**
   * @brief Init function for CaddnObstacleDetector
   *
   * @param options configuration options
   * @return true
   * @return false
   */
  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;
  /**
   * @brief Main part to detect obstacle
   *
   * @param frame camera frame
   * @return true
   * @return false
   */
  bool Detect(onboard::CameraFrame *frame) override;

  std::string Name() const override { return "CaddnObstacleDetector"; }

 private:
  bool InitTypes(const caddn::ModelParam &model_param);
  void InitParam(const caddn::ModelParam &model_param);
  void InitImageSize(const caddn::ModelParam &model_param);
  bool Preprocess(const base::Image8U *image, base::BlobPtr<float> input_blob);

 private:
  caddn::ModelParam model_param_;
  ObstacleDetectorInitOptions options_;
  std::vector<base::ObjectSubType> types_;

  int height_;
  int width_;
  float score_threshold_;

  std::vector<float> lidar_to_cam_ = {
      0.0048523,   -0.9999298, -0.01081266, -0.00711321,
      -0.00302069, 0.01079808, -0.99993706, -0.06176636,
      0.99998367,  0.00488465, -0.00296808, -0.26739058,
      0.,          0.,         0.,          1.};
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
