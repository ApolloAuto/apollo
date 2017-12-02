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
#include "modules/perception/traffic_light/rectify/detection.h"
#include <algorithm>
#include "modules/common/log.h"
#include "modules/perception/lib/base/timer.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void Detection::Perform(const cv::Mat &ros_image,
                        std::vector<LightPtr> *lights) {
  cv::Mat crop_image = ros_image(crop_box_);
  apollo::perception::Timer timer;
  timer.Start();
  lights->clear();
  // resize
  cv::Mat fw_image;
  float _crop_col_shrink = 0;
  float _crop_row_shrink = 0;
  float col_shrink = static_cast<float>(resize_len_) / (crop_image.cols);
  float row_shrink = static_cast<float>(resize_len_) / (crop_image.rows);
  _crop_col_shrink = std::max(col_shrink, row_shrink);
  _crop_row_shrink = _crop_col_shrink;
  cv::resize(crop_image, fw_image,
             cv::Size(crop_image.cols * _crop_col_shrink,
                      crop_image.rows * _crop_row_shrink));
  AINFO << "resize fw image Done at " << fw_image.size();
  // _detection
  refine_input_layer_->FetchOutterImageFrame(fw_image);
  AINFO << "FetchOutterImage Done ";
  refine_net_ptr_->ForwardFrom(0);
  int forward_time_for_this_sample =
      refine_input_layer_->GetForwardTimesForCurSample();

  for (int iter = 1; iter < forward_time_for_this_sample; ++iter) {
    refine_net_ptr_->ForwardFrom(0);
  }
  AINFO << "net forward Done!";
  // dump the output

  float inflate_col = 1.0f / _crop_col_shrink;
  float inflate_row = 1.0f / _crop_row_shrink;
  SelectOutputBboxes(crop_image, 0, inflate_col, inflate_row, lights);
  SelectOutputBboxes(crop_image, 1, inflate_col, inflate_row, lights);

  AINFO << "Dump output Done! Get box num:" << lights->size();

  uint64_t elapsed_time = timer.End("Running _detection: ");
  AINFO << "Running _detection: " << elapsed_time << " ms";
}

void Detection::Init(const int &resize_len, const std::string &refine_net,
                     const std::string &refine_model) {
  refine_net_ptr_ = new caffe::Net<float>(refine_net, caffe::TEST);
  refine_net_ptr_->CopyTrainedLayersFrom(refine_model);
  refine_input_layer_ =
      static_cast<caffe::PyramidImageOnlineDataLayer<float> *>(
          refine_net_ptr_->layers()[0].get());
  refine_output_layer_ = static_cast<caffe::ROIOutputSSDLayer<float> *>(
      refine_net_ptr_->layers()[refine_net_ptr_->layers().size() - 1].get());
  AINFO << refine_input_layer_->resize_scale << " "
        << refine_input_layer_->type();

  resize_len_ = resize_len;
}
Detection::Detection(int min_crop_size, const std::string &refine_net,
                     const std::string &refine_model) {
  Init(min_crop_size, refine_net, refine_model);
}
Detection::~Detection() {
  delete refine_net_ptr_;
}

bool Detection::SelectOutputBboxes(const cv::Mat &crop_image, int class_id,
                                   float inflate_col, float inflate_row,
                                   std::vector<LightPtr> *lights) {
  if (crop_image.empty()) {
    AERROR << "DenseBoxDetection crop_image empty, "
           << "select_output_bboxes failed.";
    return false;
  }

  if (class_id < 0 || class_id >= 2) {
    AERROR << "DenseBoxDetection invalid class_id, "
           << "select_output_bboxes failed.";
    return false;
  }

  vector<caffe::BBox<float>> &result_bbox =
      refine_output_layer_->GetFilteredBBox(class_id);
  for (int candidate_id = 0; candidate_id < result_bbox.size();
       candidate_id++) {
    LightPtr tmp(new Light);
    tmp->region.rectified_roi.x =
        static_cast<int>(result_bbox[candidate_id].x1 * inflate_col);
    tmp->region.rectified_roi.y =
        static_cast<int>(result_bbox[candidate_id].y1 * inflate_row);
    tmp->region.rectified_roi.width = static_cast<int>(
        (result_bbox[candidate_id].x2 - result_bbox[candidate_id].x1 + 1) *
        inflate_col);
    tmp->region.rectified_roi.height = static_cast<int>(
        (result_bbox[candidate_id].y2 - result_bbox[candidate_id].y1 + 1) *
        inflate_row);
    tmp->region.detect_score = result_bbox[candidate_id].score;

    if (!BoxIsValid(tmp->region.rectified_roi, crop_image.size())) {
      AINFO << "Invalid width or height or x or y: "
            << tmp->region.rectified_roi.width << " | "
            << tmp->region.rectified_roi.height << " | "
            << tmp->region.rectified_roi.x << " | "
            << tmp->region.rectified_roi.y;
      continue;
    }

    tmp->region.rectified_roi =
        RefinedBox(tmp->region.rectified_roi, crop_image.size());
    tmp->region.is_detected = true;
    tmp->region.detect_class_id = DetectionClassId(class_id);
    lights->push_back(tmp);
  }

  return true;
}
void Detection::SetCropBox(const cv::Rect &box) {
  crop_box_ = box;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
