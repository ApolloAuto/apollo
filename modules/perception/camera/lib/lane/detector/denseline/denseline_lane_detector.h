/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/base/camera.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/lib/interface/base_lane_detector.h"
#include "modules/perception/camera/lib/lane/common/denseline.pb.h"
#include "modules/perception/inference/tensorrt/rt_net.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

class DenselineLaneDetector : public BaseLaneDetector {
 public:
  DenselineLaneDetector() : BaseLaneDetector() {
    input_height_ = 0;
    input_width_ = 0;
    input_offset_y_ = 0;
    input_offset_x_ = 0;
    crop_height_ = 0;
    crop_width_ = 0;
    resize_height_ = 0;
    resize_width_ = 0;
    image_mean_[0] = 0;
    image_mean_[1] = 0;
    image_mean_[2] = 0;
    image_scale_ = 0;
  }

  virtual ~DenselineLaneDetector() {}

  bool Init(const LaneDetectorInitOptions &options =
                LaneDetectorInitOptions()) override;

  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  bool Detect(const LaneDetectorOptions &options, CameraFrame *frame) override;
  std::string Name() const override;

 private:
  std::shared_ptr<inference::Inference> rt_net_ = nullptr;
  std::shared_ptr<base::BaseCameraModel> base_camera_model_ = nullptr;
  denseline::DenselineParam denseline_param_;
  uint16_t input_height_;
  uint16_t input_width_;
  uint16_t input_offset_y_;
  uint16_t input_offset_x_;
  uint16_t crop_height_;
  uint16_t crop_width_;
  uint16_t resize_height_;
  uint16_t resize_width_;
  int image_mean_[3];
  float image_scale_;

  DataProvider::ImageOptions data_provider_image_option_;
  base::Image8U image_src_;
  std::vector<std::string> net_inputs_;
  std::vector<std::string> net_outputs_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
