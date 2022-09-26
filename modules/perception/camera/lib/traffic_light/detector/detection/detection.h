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

#include "cyber/common/macros.h"
#include "modules/perception/base/blob.h"
#include "modules/perception/base/image_8u.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/cropbox.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/select.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/pipeline/proto/stage/detection.pb.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class TrafficLightDetection : public BaseTrafficLightDetector {
 public:
  TrafficLightDetection();

  ~TrafficLightDetection() = default;

  bool Init(const TrafficLightDetectorInitOptions &options) override;

  // @brief: detect traffic_light from image.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  bool Detect(const TrafficLightDetectorOptions &options,
              CameraFrame *frame) override;
  bool SelectOutputBoxes(const std::vector<base::RectI> &crop_box_list,
                         const std::vector<float> &resize_scale_list_col,
                         const std::vector<float> &resize_scale_list_row,
                         std::vector<base::TrafficLightPtr> *lights);
  void ApplyNMS(std::vector<base::TrafficLightPtr> *lights,
                double iou_thresh = 0.6);
  bool Inference(std::vector<base::TrafficLightPtr> *lights,
                 DataProvider *data_provider);
  const std::vector<base::TrafficLightPtr> &getDetectedBoxes() {
    return detected_bboxes_;
  }

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  TrafficLightDetectionConfig detection_param_;
  std::string detection_root_dir;

  DataProvider::ImageOptions data_provider_image_option_;
  std::shared_ptr<inference::Inference> rt_net_ = nullptr;
  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<base::Blob<float>> param_blob_;
  std::shared_ptr<base::Blob<float>> mean_buffer_;
  std::shared_ptr<IGetBox> crop_;
  std::vector<base::TrafficLightPtr> detected_bboxes_;
  std::vector<base::TrafficLightPtr> selected_bboxes_;
  std::vector<std::string> net_inputs_;
  std::vector<std::string> net_outputs_;
  Select select_;
  int max_batch_size_;
  int param_blob_length_;
  float mean_[3];
  std::vector<base::RectI> crop_box_list_;
  std::vector<float> resize_scale_list_;
  int gpu_id_;

  DISALLOW_COPY_AND_ASSIGN(TrafficLightDetection);
};  // class TrafficLightDetection

}  // namespace camera
}  // namespace perception
}  // namespace apollo
