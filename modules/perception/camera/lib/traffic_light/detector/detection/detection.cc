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
#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"

#include <algorithm>
#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/resize.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool TrafficLightDetection::Init(
    const camera::TrafficLightDetectorInitOptions &options) {
  std::string proto_path = GetAbsolutePath(options.root_dir, options.conf_file);
  AINFO << "proto_path " << proto_path;
  if (!cyber::common::GetProtoFromFile(proto_path, &detection_param_)) {
    AINFO << "load proto param failed, root dir: " << options.root_dir;
    return false;
  }

  std::string param_str;
  google::protobuf::TextFormat::PrintToString(detection_param_, &param_str);
  AINFO << "TL detection param: " << param_str;

  std::string model_root =
      GetAbsolutePath(options.root_dir, detection_param_.model_name());
  AINFO << "model_root " << model_root;

  std::string proto_file =
      GetAbsolutePath(model_root, detection_param_.proto_file());
  AINFO << "proto_file " << proto_file;

  std::string weight_file =
      GetAbsolutePath(model_root, detection_param_.weight_file());
  AINFO << "weight_file " << weight_file;

  if (detection_param_.is_bgr()) {
    data_provider_image_option_.target_color = base::Color::BGR;
    mean_[0] = detection_param_.mean_b();
    mean_[1] = detection_param_.mean_g();
    mean_[2] = detection_param_.mean_r();
  } else {
    data_provider_image_option_.target_color = base::Color::RGB;
    mean_[0] = detection_param_.mean_r();
    mean_[1] = detection_param_.mean_g();
    mean_[2] = detection_param_.mean_b();
  }

  net_inputs_.push_back(detection_param_.input_blob_name());
  net_inputs_.push_back(detection_param_.im_param_blob_name());
  net_outputs_.push_back(detection_param_.output_blob_name());

  AINFO << "net input blobs: "
        << std::accumulate(net_inputs_.begin(), net_inputs_.end(),
                           std::string(""),
                           [](std::string &sum, const std::string &s) {
                             return sum + "\n" + s;
                           });
  AINFO << "net output blobs: "
        << std::accumulate(net_outputs_.begin(), net_outputs_.end(),
                           std::string(""),
                           [](std::string &sum, const std::string &s) {
                             return sum + "\n" + s;
                           });

  const auto &model_type = detection_param_.model_type();
  AINFO << "model_type: " << model_type;

  rt_net_.reset(inference::CreateInferenceByName(model_type, proto_file,
                                                 weight_file, net_outputs_,
                                                 net_inputs_, model_root));

  AINFO << "rt_net_ create succeed";
  rt_net_->set_gpu_id(options.gpu_id);
  AINFO << "set gpu id " << options.gpu_id;
  gpu_id_ = options.gpu_id;

  int resize_height = detection_param_.min_crop_size();
  int resize_width = detection_param_.min_crop_size();
  max_batch_size_ = detection_param_.max_batch_size();
  param_blob_length_ = 6;

  CHECK_GT(resize_height, 0);
  CHECK_GT(resize_width, 0);
  CHECK_GT(max_batch_size_, 0);

  std::vector<int> shape_input = {max_batch_size_, resize_height, resize_width,
                                  3};
  std::vector<int> shape_param = {max_batch_size_, 1, param_blob_length_, 1};

  std::map<std::string, std::vector<int>> input_reshape;
  input_reshape.insert(
      (std::pair<std::string, std::vector<int>>(net_inputs_[0], shape_input)));
  input_reshape.insert(
      (std::pair<std::string, std::vector<int>>(net_inputs_[1], shape_param)));

  if (!rt_net_->Init(input_reshape)) {
    AINFO << "net init fail.";
    return false;
  }
  AINFO << "net init success.";

  mean_buffer_.reset(new base::Blob<float>(1, resize_height, resize_height, 3));

  param_blob_ = rt_net_->get_blob(net_inputs_[1]);
  float *param_data = param_blob_->mutable_cpu_data();
  for (int i = 0; i < max_batch_size_; ++i) {
    auto offset = i * param_blob_length_;
    param_data[offset + 0] = static_cast<float>(resize_width);
    param_data[offset + 1] = static_cast<float>(resize_height);
    param_data[offset + 2] = 1;
    param_data[offset + 3] = 1;
    param_data[offset + 4] = 0;
    param_data[offset + 5] = 0;
  }

  switch (detection_param_.crop_method()) {
    default:
    case 0:
      crop_.reset(new CropBox(detection_param_.crop_scale(),
                              detection_param_.min_crop_size()));
      break;
    case 1:
      crop_.reset(new CropBoxWholeImage());
      break;
  }

  select_.Init(resize_width, resize_height);
  image_.reset(
      new base::Image8U(resize_height, resize_width, base::Color::BGR));
  return true;
}

bool TrafficLightDetection::Inference(
    std::vector<base::TrafficLightPtr> *lights, DataProvider *data_provider) {
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }
  crop_box_list_.clear();
  resize_scale_list_.clear();
  int img_width = data_provider->src_width();
  int img_height = data_provider->src_height();
  int resize_index = 0;
  auto batch_num = lights->size();
  auto input_img_blob = rt_net_->get_blob(net_inputs_[0]);
  auto input_param = rt_net_->get_blob(net_inputs_[1]);

  input_img_blob->Reshape(static_cast<int>(batch_num),
                          static_cast<int>(detection_param_.min_crop_size()),
                          static_cast<int>(detection_param_.min_crop_size()),
                          3);
  param_blob_->Reshape(static_cast<int>(batch_num), 6, 1, 1);
  float *param_data = param_blob_->mutable_cpu_data();
  for (size_t i = 0; i < batch_num; ++i) {
    auto offset = i * param_blob_length_;
    param_data[offset + 0] =
        static_cast<float>(detection_param_.min_crop_size());
    param_data[offset + 1] =
        static_cast<float>(detection_param_.min_crop_size());
    param_data[offset + 2] = 1;
    param_data[offset + 3] = 1;
    param_data[offset + 4] = 0;
    param_data[offset + 5] = 0;
  }

  AINFO << "reshape inputblob " << input_img_blob->shape_string();

  for (size_t i = 0; i < batch_num; ++i) {
    base::TrafficLightPtr light = lights->at(i);
    base::RectI cbox;
    crop_->getCropBox(img_width, img_height, light, &cbox);
    AINFO << "get crop box success " << cbox.x << " " << cbox.y << " "
          << cbox.width << " " << cbox.height;

    if (!OutOfValidRegion(cbox, img_width, img_height) && cbox.Area() > 0) {
      crop_box_list_.push_back(cbox);
      light->region.debug_roi[0] = cbox;
      light->region.crop_roi = cbox;

      data_provider_image_option_.do_crop = true;
      data_provider_image_option_.crop_roi = cbox;
      data_provider_image_option_.target_color = base::Color::BGR;
      data_provider->GetImage(data_provider_image_option_, image_.get());
      AINFO << "get image data success ";

      float resize_scale =
          static_cast<float>(detection_param_.min_crop_size()) /
          static_cast<float>(std::min(cbox.width, cbox.height));
      resize_scale_list_.push_back(resize_scale);

      inference::ResizeGPU(*image_, input_img_blob, img_width, resize_index,
                           mean_[0], mean_[1], mean_[2], true, 1.0);
      resize_index++;
    }
  }
  // _detection
  cudaDeviceSynchronize();
  rt_net_->Infer();
  cudaDeviceSynchronize();
  AINFO << "rt_net run success";

  // dump the output
  SelectOutputBoxes(crop_box_list_, resize_scale_list_, resize_scale_list_,
                    &detected_bboxes_);

  ApplyNMS(&detected_bboxes_);

  return true;
}

bool TrafficLightDetection::Detect(const TrafficLightDetectorOptions &options,
                                   CameraFrame *frame) {
  if (frame->traffic_lights.empty()) {
    AINFO << "no lights to detect";
    return true;
  }

  const auto &data_provider = frame->data_provider;
  auto input_blob = rt_net_->get_blob(net_inputs_[0]);
  int img_width = data_provider->src_width();
  int img_height = data_provider->src_height();
  std::vector<base::TrafficLightPtr> &lights_ref = frame->traffic_lights;

  AINFO << "detection input " << lights_ref.size() << " lights";

  selected_bboxes_.clear();
  detected_bboxes_.clear();

  for (auto &light : lights_ref) {
    base::RectI debug_rect(0, 0, 0, 0);
    light->region.detection_roi = light->region.projection_roi;
    light->region.debug_roi.clear();
    light->region.debug_roi_detect_scores.clear();
    light->region.debug_roi.push_back(debug_rect);
    light->region.debug_roi_detect_scores.push_back(0.0f);
  }

  for (auto &light : lights_ref) {
    if (light->region.outside_image ||
        OutOfValidRegion(light->region.projection_roi, img_width, img_height) ||
        light->region.projection_roi.Area() <= 0) {
      light->region.projection_roi.x = 0;
      light->region.projection_roi.y = 0;
      light->region.projection_roi.width = 0;
      light->region.projection_roi.height = 0;
    }
  }

  Inference(&lights_ref, data_provider);

  AINFO << "Dump output Done! Get box num:" << detected_bboxes_.size();

  for (size_t j = 0; j < detected_bboxes_.size(); ++j) {
    base::RectI &region = detected_bboxes_[j]->region.detection_roi;
    float score = detected_bboxes_[j]->region.detect_score;
    lights_ref[0]->region.debug_roi.push_back(region);
    lights_ref[0]->region.debug_roi_detect_scores.push_back(score);
  }

  AINFO << "start select";
  select_.SelectTrafficLights(detected_bboxes_, &lights_ref);
  AINFO << "select success";

  AINFO << "detection success";
  return true;
}

bool TrafficLightDetection::SelectOutputBoxes(
    const std::vector<base::RectI> &crop_box_list,
    const std::vector<float> &resize_scale_list_col,
    const std::vector<float> &resize_scale_list_row,
    std::vector<base::TrafficLightPtr> *lights) {
  auto output_blob = rt_net_->get_blob(net_outputs_[0]);
  int result_box_num = output_blob->shape(0);
  int each_box_length = output_blob->shape(1);

  AINFO << "output blob size " << output_blob->shape(0) << " "
        << output_blob->shape(1) << " " << output_blob->shape(2) << " "
        << output_blob->shape(3);

  for (int candidate_id = 0; candidate_id < result_box_num; candidate_id++) {
    const float *result_data =
        output_blob->cpu_data() + candidate_id * each_box_length;
    int img_id = static_cast<int>(result_data[0]);
    if (img_id >= static_cast<int>(crop_box_list.size())) {
      AINFO << "img id " << img_id << " > " << crop_box_list.size();
      continue;
    }
    base::TrafficLightPtr tmp(new base::TrafficLight);
    float inflate_col = 1 / resize_scale_list_col.at(img_id);
    float inflate_row = 1 / resize_scale_list_row.at(img_id);
    float x1 = result_data[1];
    float y1 = result_data[2];
    float x2 = result_data[3];
    float y2 = result_data[4];
    std::vector<float> score{result_data[5], result_data[6], result_data[7],
                             result_data[8]};
    for (int i = 0; i < 9; ++i) {
      ADEBUG << "result_data " << result_data[i];
    }

    std::vector<float>::iterator biggest =
        std::max_element(std::begin(score), std::end(score));
    tmp->region.detect_class_id =
        base::TLDetectionClass(std::distance(std::begin(score), biggest) - 1);

    if (static_cast<int>(tmp->region.detect_class_id) >= 0) {
      tmp->region.detection_roi.x = static_cast<int>(x1 * inflate_col);
      tmp->region.detection_roi.y = static_cast<int>(y1 * inflate_row);
      tmp->region.detection_roi.width =
          static_cast<int>((x2 - x1 + 1) * inflate_col);
      tmp->region.detection_roi.height =
          static_cast<int>((y2 - y1 + 1) * inflate_row);
      tmp->region.detect_score = *biggest;

      if (OutOfValidRegion(tmp->region.detection_roi,
                           crop_box_list.at(img_id).width,
                           crop_box_list.at(img_id).height) ||
          tmp->region.detection_roi.Area() <= 0) {
        AINFO << "Invalid width or height or x or y: "
              << tmp->region.detection_roi.width << " | "
              << tmp->region.detection_roi.height << " | "
              << tmp->region.detection_roi.x << " | "
              << tmp->region.detection_roi.y;
        AINFO << " max width " << crop_box_list.at(img_id).width
              << " max height " << crop_box_list.at(img_id).height
              << " at img_id " << img_id;
        continue;
      }

      RefineBox(tmp->region.detection_roi, crop_box_list.at(img_id).width,
                crop_box_list.at(img_id).height, &(tmp->region.detection_roi));
      tmp->region.detection_roi.x += crop_box_list.at(img_id).x;
      tmp->region.detection_roi.y += crop_box_list.at(img_id).y;
      tmp->region.is_detected = true;
      AINFO << "detect roi x " << tmp->region.detection_roi.x << " "
            << tmp->region.detection_roi.y << " "
            << tmp->region.detection_roi.width << " "
            << tmp->region.detection_roi.height;

      lights->push_back(tmp);
    } else {
      AINFO << "Invalid classid  "
            << static_cast<int>(tmp->region.detect_class_id);
    }
  }

  return true;
}

void TrafficLightDetection::ApplyNMS(std::vector<base::TrafficLightPtr> *lights,
                                     double iou_thresh) {
  if (lights == nullptr) {
    AERROR << "lights are not available";
    return;
  }

  // (score, index) pairs sorted by detect score
  std::vector<std::pair<float, int>> score_index_vec(lights->size());
  for (size_t i = 0; i < lights->size(); ++i) {
    score_index_vec[i].first = lights->at(i)->region.detect_score;
    score_index_vec[i].second = static_cast<int>(i);
  }
  std::stable_sort(
      score_index_vec.begin(), score_index_vec.end(),
      [](const std::pair<float, int> &pr1, const std::pair<float, int> &pr2) {
        return pr1.first < pr2.first;
      });

  std::vector<int> kept_indices;
  while (!score_index_vec.empty()) {
    const int idx = score_index_vec.back().second;
    bool keep = true;
    for (size_t k = 0; k < kept_indices.size(); ++k) {
      const int kept_idx = kept_indices[k];
      const auto &rect1 = lights->at(idx)->region.detection_roi;
      const auto &rect2 = lights->at(kept_idx)->region.detection_roi;
      float overlap =
          static_cast<float>((rect1 & rect2).Area() / (rect1 | rect2).Area());
      // if current bbox has large overlap(>=iou_thresh) with any
      // kept bbox, drop it
      keep = std::fabs(overlap) < iou_thresh;
      if (!keep) {
        break;
      }
    }
    if (keep) {
      kept_indices.push_back(idx);
    }
    score_index_vec.pop_back();
  }

  int idx = 0;
  auto parted_itr = std::stable_partition(
      lights->begin(), lights->end(), [&](const base::TrafficLightPtr &light) {
        return std::find(kept_indices.begin(), kept_indices.end(), idx++) !=
               kept_indices.end();
      });
  lights->erase(parted_itr, lights->end());
}

std::string TrafficLightDetection::Name() const {
  return "TrafficLightDetection";
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(TrafficLightDetection);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
