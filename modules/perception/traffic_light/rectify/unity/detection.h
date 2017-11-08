//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_UNITY_DETECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_UNITY_DETECTION_H

#include <caffe/layers/pyramid_layers.hpp>
#include <traffic_light/interface/green_interface.h>
#include <traffic_light/base/light.h>
#include "caffe/caffe.hpp"

namespace adu {
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

  void init(int &min_crop_size, const std::string &refine_net, const std::string &refine_model);

  virtual void perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights);

  virtual void set_crop_box(const cv::Rect &box);

  ~Detection();

  bool set_output_box_type(DetectOutputBoxType type);

  bool select_output_bboxes(const cv::Mat &crop_image,
                            int class_id, float inflate_col, float inflate_row,
                            std::vector<LightPtr> *lights);

 private:
  caffe::Net<float> *_refine_net_ptr;
  caffe::PyramidImageOnlineDataLayer<float> *_refine_input_layer;
  caffe::ROIOutputSSDLayer<float> *_refine_output_layer;

  int _min_crop_size;
  cv::Rect _crop_box;
  DetectOutputBoxType _detect_output_type = DetectOutputBoxType::BOX_ALL;
  float _output_threshold;
};
}
}
}
#endif //GREEN_DenseBoxDetection_H
