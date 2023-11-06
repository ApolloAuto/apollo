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

#include "modules/perception/camera_detection_multi_stage/detector/yolo/proto/model_param.pb.h"

#include "modules/perception/camera_detection_multi_stage/detector/yolo/postprocess.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class YoloObstacleDetector : public BaseObstacleDetector {
 public:
  YoloObstacleDetector() = default;
  virtual ~YoloObstacleDetector() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }
  /**
   * @brief Load obstacle detector configs
   *
   * @param options the option object of obstacle
   * @return true
   * @return false
   */
  bool Init(const ObstacleDetectorInitOptions &options =
                ObstacleDetectorInitOptions()) override;
  /**
   * @brief Get obstacles detection result, detector main part
   *
   * @param frame  camera frame
   * @return true
   * @return false
   */
  bool Detect(onboard::CameraFrame *frame) override;
  /**
   * @brief ObstacleDetector name
   *
   * @return std::string
   */
  std::string Name() const override { return "YoloObstacleDetector"; }

 protected:
  void LoadInputShape(const yolo::ModelParam &model_param);
  void LoadParam(const yolo::ModelParam &model_param);
  void InitYoloBlob();

 private:
  ObstacleDetectorInitOptions options_;
  yolo::ModelParam model_param_;

  std::vector<base::ObjectSubType> types_;
  std::vector<float> expands_;
  std::vector<float> anchors_;

  NMSParam nms_;
  MinDims min_dims_;
  cudaStream_t stream_;
  int height_ = 0;
  int width_ = 0;
  int offset_y_ = 0;
  int gpu_id_ = 0;
  int obj_k_ = kMaxObjSize;

  int ori_cycle_ = 1;
  float confidence_threshold_ = 0.f;
  float light_vis_conf_threshold_ = 0.f;
  float light_swt_conf_threshold_ = 0.f;

  YoloBlobs yolo_blobs_;

  std::shared_ptr<base::Blob<bool>> overlapped_ = nullptr;
  std::shared_ptr<base::Blob<int>> idx_sm_ = nullptr;

  bool with_box3d_ = false;
  bool with_frbox_ = false;
  bool with_lights_ = false;
  bool with_ratios_ = false;
  bool with_area_id_ = false;
  float border_ratio_ = 0.f;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
