/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "opencv2/opencv.hpp"

#include "modules/perception/base/camera.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/lib/interface/base_lane_detector.h"
#include "modules/perception/pipeline/proto/stage/darkSCNN.pb.h"
#if GPU_PLATFORM == NVIDIA
  #include "modules/perception/inference/tensorrt/rt_net.h"
#elif GPU_PLATFORM == AMD
  #include "modules/perception/inference/migraphx/mi_net.h"
#endif
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class DarkSCNNLaneDetector : public BaseLaneDetector {
 public:
  DarkSCNNLaneDetector() : BaseLaneDetector() {
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
    confidence_threshold_lane_ = 0;
    lane_output_height_ = 0;
    lane_output_width_ = 0;
    num_lanes_ = 0;
  }

  virtual ~DarkSCNNLaneDetector() = default;

  bool Init(const LaneDetectorInitOptions &options =
                LaneDetectorInitOptions()) override;

  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  bool Detect(const LaneDetectorOptions &options, CameraFrame *frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  std::shared_ptr<inference::Inference> cnnadapter_lane_ = nullptr;
  std::shared_ptr<base::BaseCameraModel> base_camera_model_ = nullptr;
  darkSCNN::DarkSCNNParam darkscnn_param_;

  // parameters for data provider
  uint16_t input_height_;
  uint16_t input_width_;
  uint16_t input_offset_y_;
  uint16_t input_offset_x_;
  uint16_t crop_height_;
  uint16_t crop_width_;
  uint16_t resize_height_;
  uint16_t resize_width_;
  int image_mean_[3];
  std::vector<float> vpt_mean_;
  std::vector<float> vpt_std_;
  // parameters for network output
  float confidence_threshold_lane_;
  int lane_output_height_;
  int lane_output_width_;
  int num_lanes_;

  int64_t time_1 = 0;
  int64_t time_2 = 0;
  int time_num = 0;

  DataProvider::ImageOptions data_provider_image_option_;
  base::Image8U image_src_;
  std::vector<std::string> net_inputs_;
  std::vector<std::string> net_outputs_;
  std::shared_ptr<base::Blob<float>> lane_blob_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
