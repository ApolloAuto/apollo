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
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

// @brief: feature_extractor init options for each detected object
// @param [input_width/input_height]: input image width/height. required
// @param [feat_width/height/channel]: feat blob width/height/channel. required
// @param [max_object]: maximum num of objs for feature extractor. optional
// @param [param]: feature
struct FeatureExtractorInitOptions : public BaseInitOptions {
  int input_width = 0;
  int input_height = 0;
  std::shared_ptr<base::Blob<float>> feat_blob;
  int gpu_id = 0;
};

struct FeatureExtractorOptions {
  bool normalized = true;
};
class BaseFeatureExtractor {
 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;
  virtual bool Init(const FeatureExtractorInitOptions &init_options) = 0;
  // @brief: extract feature for each detected object
  // @param [in/out]: objects with bounding boxes and feature vector.
  virtual bool Extract(const FeatureExtractorOptions &options,
                       CameraFrame *frame) = 0;
  virtual std::string Name() const = 0;

  void set_roi(int x, int y, int w, int h) {
    roi_x_ = x;
    roi_y_ = y;
    roi_w_ = w;
    roi_h_ = h;
  }

  void decode_bbox(std::vector<std::shared_ptr<base::Object>> *objects) {
    for (auto obj : *objects) {
      auto &xmin = obj->camera_supplement.box.xmin;
      auto &ymin = obj->camera_supplement.box.ymin;
      auto &xmax = obj->camera_supplement.box.xmax;
      auto &ymax = obj->camera_supplement.box.ymax;
      xmin = xmin * static_cast<float>(roi_w_) + static_cast<float>(roi_x_);
      xmax = xmax * static_cast<float>(roi_w_) + static_cast<float>(roi_x_);
      ymin = ymin * static_cast<float>(roi_h_) + static_cast<float>(roi_y_);
      ymax = ymax * static_cast<float>(roi_h_) + static_cast<float>(roi_y_);
    }
  }

  void encode_bbox(std::vector<std::shared_ptr<base::Object>> *objects) {
    for (auto obj : *objects) {
      auto &xmin = obj->camera_supplement.box.xmin;
      auto &ymin = obj->camera_supplement.box.ymin;
      auto &xmax = obj->camera_supplement.box.xmax;
      auto &ymax = obj->camera_supplement.box.ymax;
      xmin = (xmin - static_cast<float>(roi_x_)) / static_cast<float>(roi_w_);
      xmax = (xmax - static_cast<float>(roi_x_)) / static_cast<float>(roi_w_);
      ymin = (ymin - static_cast<float>(roi_y_)) / static_cast<float>(roi_h_);
      ymax = (ymax - static_cast<float>(roi_y_)) / static_cast<float>(roi_h_);
    }
  }

 protected:
  std::shared_ptr<base::Blob<float>> feat_blob_ = nullptr;
  int roi_x_ = 0;
  int roi_y_ = 0;
  int roi_w_ = 0;
  int roi_h_ = 0;
};
PERCEPTION_REGISTER_REGISTERER(BaseFeatureExtractor);
#define REGISTER_FEATURE_EXTRACTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseFeatureExtractor, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
