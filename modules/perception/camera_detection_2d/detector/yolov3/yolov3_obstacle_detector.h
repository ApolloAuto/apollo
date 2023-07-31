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
#include <algorithm>

#include "modules/perception/camera_detection_2d/detector/yolov3/proto/model_param.pb.h"
#include "modules/perception/common/base/blob.h"

#include "modules/perception/camera_detection_2d/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class Yolov3ObstacleDetector : public BaseObstacleDetector {
 public:
  Yolov3ObstacleDetector() : BaseObstacleDetector() {}
  virtual ~Yolov3ObstacleDetector() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  /**
  * @brief Necessary, Init yolov3 model params normal,
           but now use pipline instead
  * @param options obstacle detection init options
  * @return init status, yolov3 detector stage status
  */
  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;

  bool Detect(onboard::CameraFrame *frame) override;

  /**
   * @brief return detector name
   * @param None
   * @return now detector type
   */
  std::string Name() const override { return "Yolov3ObstacleDetector"; }

 protected:
  /**
  * @brief Preprocess of image before inference,
           resize input data blob and fill image data to blob
  * @param image image read from camera frame of 6mm camera
  * @param input_blob image input blob address pointer
  * @return preprocess status
  */
  bool Preprocess(const base::Image8U *image, base::BlobPtr<float> input_blob);

  /**
   * @brief Resize model input picture size according to the config
   *        file
   * @param model_param yolov3 proto param read from yolov3.pt
   * @return None
   */
  void LoadInputShape(const yolov3::ModelParam &model_param);

  /**
  * @brief Load yolo libtorch model params from model file
  * @param yolov3_param yolov3 proto param read from yolov3.pt,
            include ModelParam、NetworkParam and NMSParam
  * @return None
  */
  void LoadParam(const yolov3::ModelParam &model_param);

 private:
  ObstacleDetectorInitOptions options_;
  yolov3::ModelParam model_param_;
  yolov3::NMSParam nms_;

  int gpu_id_ = 0;
  cudaStream_t stream_ = nullptr;

  // yolo input image size
  int width_ = 0;
  int height_ = 0;
  // image size of raw image
  int image_width_ = 0;
  int image_height_ = 0;
  float confidence_threshold_ = 0.f;

  float border_ratio_ = 0.f;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
