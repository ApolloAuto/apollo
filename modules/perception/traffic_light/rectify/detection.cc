//
// Created by gaohan02 on 16-8-1.
//

#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "detection.h"
#include "modules/perception/lib/base/timer.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void Detection::Perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights) {
  cv::Mat crop_image = ros_image(crop_box_);
  apollo::perception::Timer timer;
  timer.Start();
  lights->clear();
  // resize
  cv::Mat fw_image;
  float _crop_col_shrink = 0;
  float _crop_row_shrink = 0;
  float col_shrink = float(resize_len_) / float(crop_image.cols);
  float row_shrink = float(resize_len_) / float(crop_image.rows);
  _crop_col_shrink = std::max(col_shrink, row_shrink);
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
  _refine_net_ptr = new caffe::Net<float>(refine_net, caffe::TEST);
  _refine_net_ptr->CopyTrainedLayersFrom(refine_model);
  _refine_input_layer = static_cast <caffe::PyramidImageOnlineDataLayer<float> *>
  (_refine_net_ptr->layers()[0].get());
  _refine_output_layer = static_cast <caffe::ROIOutputSSDLayer<float> *>
  (_refine_net_ptr->layers()[_refine_net_ptr->layers().size() - 1].get());
  AINFO << _refine_input_layer->resize_scale;
  resize_len_ = resize_len;
}
Detection::Detection(int &min_crop_size, const std::string &refine_net,
                     const std::string &refine_model) {
  Init(min_crop_size, refine_net, refine_model);
}
Detection::~Detection() {
  delete _refine_net_ptr;
}

bool Detection::SelectOutputBboxes(const cv::Mat &crop_image,
                                   int class_id, float inflate_col, float inflate_row,
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

  vector<caffe::BBox<float>> &result_bbox = _refine_output_layer->GetFilteredBBox(class_id);
  for (int candidate_id = 0; candidate_id < result_bbox.size(); candidate_id++) {
    LightPtr tmp(new Light);
    tmp->region.rectified_roi.x = (int) (result_bbox[candidate_id].x1 * inflate_col);
    tmp->region.rectified_roi.y = (int) (result_bbox[candidate_id].y1 * inflate_row);
    tmp->region.rectified_roi.width = (int) (
        (result_bbox[candidate_id].x2 - result_bbox[candidate_id].x1 + 1) *
            inflate_col);
    tmp->region.rectified_roi.height = (int) (
        (result_bbox[candidate_id].y2 - result_bbox[candidate_id].y1 + 1) *
            inflate_row);
    tmp->region.detect_score = result_bbox[candidate_id].score;

    if (!BoxIsValid(tmp->region.rectified_roi, crop_image.size())) {
      AINFO << "Invalid width or height or x or y: " << tmp->region.rectified_roi.width
            << " | "
            << tmp->region.rectified_roi.height << " | " << tmp->region.rectified_roi.x
            << " | " << tmp->region.rectified_roi.y;
      continue;
    }

    tmp->region.rectified_roi = RefinedBox(tmp->region.rectified_roi, crop_image.size());
    tmp->region.is_detected = true;
    tmp->region.detect_class_id = DetectionClassId(class_id);// == 0 ? DAY_CLASS : NIGHT_CLASS);
    lights->push_back(tmp);
  }

  return true;
}
void Detection::SetCropBox(const cv::Rect &box) {
  crop_box_ = box;
}

}
}
}
