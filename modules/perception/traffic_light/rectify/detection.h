//
// Created by gaohan02 on 16-8-1.
//

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_UNITY_DETECTION_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_UNITY_DETECTION_H

#include <caffe/layers/pyramid_layers.hpp>
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "modules/perception/traffic_light/base/light.h"
#include "caffe/caffe.hpp"

namespace apollo {
namespace perception {
namespace traffic_light {

enum DetectOutputBoxType {
  BOX_ALL = 0,
  BOX_VERTICAL = 1,
  BOX_QUADRATE = 2,
  BOX_HORIZONTAL = 3,
  DETECT_OUTPUT_BOX_TYPE_COUNT = 4
};

class Detection : public IRefine {
 public:
  Detection(int &min_crop_size, const std::string &refine_net,
            const std::string &refine_model);

  void Init(const int &resize_len, const std::string &refine_net, const std::string &refine_model);

  virtual void Perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights);

  virtual void SetCropBox(const cv::Rect &box);

  ~Detection();

 private:

  bool SelectOutputBboxes(const cv::Mat &crop_image,
                          int class_id, float inflate_col, float inflate_row,
                          std::vector<LightPtr> *lights);

  caffe::Net<float> *_refine_net_ptr;
  caffe::PyramidImageOnlineDataLayer<float> *_refine_input_layer;
  caffe::ROIOutputSSDLayer<float> *_refine_output_layer;

  int resize_len_;
  cv::Rect crop_box_;
  DetectOutputBoxType detect_output_type_ = DetectOutputBoxType::BOX_ALL;
};
}
}
}
#endif //GREEN_DenseBoxDetection_H
