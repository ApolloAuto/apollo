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

#include "Eigen/Core"

#include "modules/perception/common/proto/model_info.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/camera_detection_bev/camera_frame.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

struct ObstacleDetectorInitOptions : public BaseInitOptions {
  int image_height;
  int image_width;
  Eigen::Matrix3f intrinsic;
};

class BaseObstacleDetector {
 public:
  BaseObstacleDetector() = default;

  virtual ~BaseObstacleDetector() = default;
  /**
   * @brief Interface for loading obstacle detector config files
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(const ObstacleDetectorInitOptions &options) = 0;
  /**
   * @brief Interface for obstacle detector main part
   *
   * @param frame obstacle type and 2D bbox should be filled, required, 3D
   * information of obstacle can be filled, optional.
   * @return true
   * @return false
   */
  virtual bool Detect(CameraFrame *frame) = 0;
  /**
   * @brief Interface for obstacle detector name
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;
  /**
   * @brief Interface for network initialization
   *
   * @param model_info network config
   * @param model_root root path of network model
   * @return true
   * @return false
   */
  virtual bool InitNetwork(const common::ModelInfo &model_info,
                           const std::string &model_root);

 protected:
  int gpu_id_ = 0;
  std::shared_ptr<inference::Inference> net_;

  DISALLOW_COPY_AND_ASSIGN(BaseObstacleDetector);
};  // class BaseObstacleDetector

PERCEPTION_REGISTER_REGISTERER(BaseObstacleDetector);
#define REGISTER_OBSTACLE_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseObstacleDetector, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
