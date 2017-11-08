//
// Created by gaohan02 on 16-8-1.
//

#include <traffic_light/base/utils.h>
#include "module/perception/traffic_light/rectify/densebox/detection/detection.h"
#include "module/perception/traffic_light/rectify/densebox/detection/detection_nms.hpp"
#include "lib/base/timer.h"

namespace adu {
namespace perception {
namespace traffic_light {

void DenseboxDetection::perform(cv::Mat &ros_image, vector<BoundBox_t> &refined_bboxes) {
  cv::Mat crop_image = ros_image(_crop_box.rect);
  adu::perception::base::Timer timer;
  timer.start();
  vector<BoundBox_t> detected_bboxes;
  detected_bboxes.clear();
  refined_bboxes.clear();

  // resize
  const float shrink_bias = 0.03f;
  cv::Mat fw_image;
  float _crop_col_shrink = 0;
  float _crop_row_shrink = 0;
  float col_shrink = float(_min_crop_size) / float(crop_image.cols) + shrink_bias;
  float row_shrink = float(_min_crop_size) / float(crop_image.rows) + shrink_bias;
  _crop_col_shrink = max(col_shrink, row_shrink);
  _crop_row_shrink = _crop_col_shrink;
  cv::resize(crop_image, fw_image,
             cv::Size(crop_image.cols * _crop_col_shrink,
                      crop_image.rows * _crop_row_shrink));
  AINFO << "resize fw image Done at " << fw_image.size();
  // _detection
  _refine_input_layer->FetchOutterImageFrame(fw_image);
  AINFO << "FetchOutterImage Done ";
  _refine_net_ptr->ForwardFrom(0);
  int forward_time_for_this_sample = _refine_input_layer->GetForwardTimesForCurSample();

  for (int iter = 1; iter < forward_time_for_this_sample; ++iter) {
    _refine_net_ptr->ForwardFrom(0);
  }
  AINFO << "net forward Done!";
  // dump the output
  float _refine_scale = _refine_input_layer->resize_scale;
  float scale_up = 1.0f / _refine_scale;
  float inflate_col = 1.0f / _crop_col_shrink;
  float inflate_row = 1.0f / _crop_row_shrink;
  for (int class_id = 0; class_id < 1; class_id++) {// [zoo] take care
    vector<caffe::BBox<float>> &result_bbox = _refine_output_layer->GetFilteredBBox(class_id);
    for (int candidate_id = 0; candidate_id < result_bbox.size(); candidate_id++) {
      BoundBox_t tmp;
      tmp.rect.x = (int) (result_bbox[candidate_id].x1 * scale_up * inflate_col);
      tmp.rect.y = (int) (result_bbox[candidate_id].y1 * scale_up * inflate_row);
      tmp.rect.width = (int) (
          (result_bbox[candidate_id].x2 - result_bbox[candidate_id].x1 + 1) *
              scale_up *
              inflate_col);
      tmp.rect.height = (int) (
          (result_bbox[candidate_id].y2 - result_bbox[candidate_id].y1 + 1) *
              scale_up *
              inflate_row);
      tmp.score = result_bbox[candidate_id].score;

      if (tmp.rect.width <= 0 || tmp.rect.height <= 0
          || tmp.rect.x >= crop_image.cols || tmp.rect.y >= crop_image.rows
          || tmp.rect.x + tmp.rect.width - 1 < 0 || tmp.rect.y + tmp.rect.height - 1 < 0) {
        AINFO << "Invalid width or height or x or y: " << tmp.rect.width << " | "
              << tmp.rect.height << " | " << tmp.rect.x << " | " << tmp.rect.y;
        continue;
      }

      tmp.rect = refined_box(tmp.rect, crop_image.size());
      tmp.isValid = true;
      detected_bboxes.push_back(tmp);
    }
  }
  AINFO << "Dump output Done! Get box num:" << detected_bboxes.size();


  // do the NMS
  nms(detected_bboxes, crop_image.rows, crop_image.cols, _nms_overlap, refined_bboxes);
  uint64_t elapsed_time = timer.end("Running _detection: ");
  AINFO << "Running _detection: " << elapsed_time << " ms";

}
void DenseboxDetection::set_crop_box(BoundBox_t &box) {
  _crop_box = box;
}
void
DenseboxDetection::init(int &min_crop_size, const string &refine_net, const string &refine_model,
                        float nms_overlap) {
  _refine_net_ptr = new caffe::Net<float>(refine_net, caffe::TEST);
  _refine_net_ptr->CopyTrainedLayersFrom(refine_model);
  _refine_input_layer = static_cast <caffe::PyramidImageOnlineDataLayer<float> *>
  (_refine_net_ptr->layers()[0].get());
  _refine_output_layer = static_cast <caffe::DetectionOutputLayer<float> *>
  (_refine_net_ptr->layers()[
          _refine_net_ptr->layers().size() - 1].get());
  AINFO << _refine_input_layer->resize_scale;
  min_crop_size = (int) (min_crop_size / _refine_input_layer->resize_scale);
  _min_crop_size = min_crop_size;
  _nms_overlap = nms_overlap;
}
DenseboxDetection::DenseboxDetection(int &min_crop_size, const string &refine_net,
                                     const string &refine_model,
                                     float nms_overlap) {
  init(min_crop_size, refine_net, refine_model, nms_overlap);
}
DenseboxDetection::~DenseboxDetection() {
  delete _refine_net_ptr;
}
}
}
}
