/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/recognizer/classify.h"

#include <vector>

#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

ClassifyBySimple::ClassifyBySimple(const std::string &class_net_,
                                   const std::string &class_model_,
                                   float threshold, unsigned int resize_width,
                                   unsigned int resize_height) {
  Init(class_net_, class_model_, threshold, resize_width, resize_height);
}

void ClassifyBySimple::SetCropBox(const cv::Rect &box) { crop_box_ = box; }
void ClassifyBySimple::Init(const std::string &class_net_,
                            const std::string &class_model_, float threshold,
                            unsigned int resize_width,
                            unsigned int resize_height) {
  AINFO << "Creating testing net...";
  classify_net_ptr_.reset(new caffe::Net<float>(class_net_, caffe::TEST));

  AINFO << "restore parameters...";
  classify_net_ptr_->CopyTrainedLayersFrom(class_model_);

  resize_height_ = resize_height;
  resize_width_ = resize_width;
  unknown_threshold_ = threshold;

  AINFO << "Init Done";
}

void ClassifyBySimple::Perform(const cv::Mat &ros_image,
                               std::vector<LightPtr> *lights) {
  caffe::Blob<float> *input_blob_recog = classify_net_ptr_->input_blobs()[0];
  caffe::Blob<float> *output_blob_recog =
      classify_net_ptr_->top_vecs()[classify_net_ptr_->top_vecs().size() - 1]
                                   [0];
  cv::Mat img = ros_image(crop_box_);
  for (LightPtr light : *lights) {
    if (!light->region.is_detected ||
        !BoxIsValid(light->region.rectified_roi, ros_image.size())) {
      continue;
    }

    cv::Mat img_light = img(light->region.rectified_roi).clone();
    assert(img_light.rows > 0);
    assert(img_light.cols > 0);

    cv::resize(img_light, img_light, cv::Size(resize_width_, resize_height_));
    float *data = input_blob_recog->mutable_cpu_data();
    uchar *pdata = img_light.data;
    for (int h = 0; h < resize_height_; ++h) {
      pdata = img_light.data + h * img_light.step;
      for (int w = 0; w < resize_width_; ++w) {
        for (int channel = 0; channel < 3; channel++) {
          int index = (channel * resize_height_ + h) * resize_width_ + w;
          data[index] = static_cast<float>((*pdata));
          ++pdata;
        }
      }
    }

    classify_net_ptr_->ForwardFrom(0);
    float *out_put_data = output_blob_recog->mutable_cpu_data();
    ProbToColor(out_put_data, unknown_threshold_, light);
  }
}

void ClassifyBySimple::ProbToColor(const float *out_put_data, float threshold,
                                   LightPtr light) {
  int max_color_id = 0;
  std::vector<TLColor> status_map = {BLACK, RED, YELLOW, GREEN};
  std::vector<std::string> name_map = {"Black", "Red", "Yellow", "Green"};
  std::vector<float> prob(out_put_data, out_put_data + status_map.size());
  auto max_prob = std::max_element(prob.begin(), prob.end());
  max_color_id = (*max_prob > threshold)
                     ? static_cast<int>(std::distance(prob.begin(), max_prob))
                     : 0;

  light->status.color = status_map[max_color_id];
  light->status.confidence = out_put_data[max_color_id];
  AINFO << "Light status recognized as " << name_map[max_color_id];
  AINFO << "Color Prob:";
  for (size_t j = 0; j < status_map.size(); ++j) {
    AINFO << out_put_data[j];
  }
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
