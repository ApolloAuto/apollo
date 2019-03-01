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

#include "modules/perception/base/blob.h"
#include "modules/perception/base/image_8u.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.pb.h"
#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace camera {

class ClassifyBySimple {
 public:
  ClassifyBySimple() = default;
  ~ClassifyBySimple() = default;

  void Init(const traffic_light::recognition::ClassifyParam& model_config,
            const int gpu_id, const std::string work_root);

  void Perform(const CameraFrame* frame,
               std::vector<base::TrafficLightPtr>* lights);

 private:
  void Prob2Color(const float* out_put_data, float threshold,
                  base::TrafficLightPtr light);
  std::shared_ptr<inference::Inference> rt_net_ = nullptr;
  DataProvider::ImageOptions data_provider_image_option_;
  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<base::Blob<float>> mean_buffer_;
  std::shared_ptr<base::Blob<float>> mean_;
  std::vector<std::string> net_inputs_;
  std::vector<std::string> net_outputs_;
  int resize_width_;
  int resize_height_;
  float unknown_threshold_;
  float scale_;
  int gpu_id_ = 0;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
