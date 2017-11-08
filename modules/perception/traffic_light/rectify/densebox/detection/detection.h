//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_DETECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_DETECTION_H

#include <caffe/layers/pyramid_layers.hpp>
#include <traffic_light/interface/green_interface.h>
#include "caffe/caffe.hpp"

namespace adu {
namespace perception {
namespace traffic_light {
class DenseboxDetection : public IRefine {
 public:
  DenseboxDetection(int &min_crop_size, const string &refine_net, const string &refine_model,
                    float nms_overlap);

  void init(int &min_crop_size, const string &refine_net, const string &refine_model,
            float nms_overlap);

  virtual void perform(cv::Mat &ros_image, vector<BoundBox_t> &boxes);

  virtual void set_crop_box(BoundBox_t &box);

  ~DenseboxDetection();

 private:
  caffe::Net<float> *_refine_net_ptr;
  caffe::PyramidImageOnlineDataLayer<float> *_refine_input_layer;
  caffe::DetectionOutputLayer<float> *_refine_output_layer;

  int _min_crop_size;
  BoundBox_t _crop_box;
  float _nms_overlap;
};
}
}
}
#endif //GREEN_DETECTION_H
