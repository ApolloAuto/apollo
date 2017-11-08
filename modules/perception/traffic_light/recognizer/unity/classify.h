//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H

#include <string>
#include <utility>
#include "modules/perception/traffic_light/base/light.h"
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "caffe/caffe.hpp"

namespace apollo {
namespace perception {
namespace traffic_light {

class ClassifyBySimple : public IRefine {
 public:
  ClassifyBySimple(const std::string &_class_net, const std::string &_class_model,
                   float threshold, unsigned int resize_width,
                   unsigned int resize_height);

  void init(const std::string &_class_net, const std::string &_class_model, float threshold,
            unsigned int resize_width, unsigned int resize_height);

  virtual void perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights);

  virtual void set_crop_box(const cv::Rect &box);

  ~ClassifyBySimple();

 private:
  void prob_to_color(const float *out_put_data, float threshold, LightPtr light);
  caffe::Net<float> *_classify_net_ptr;
  cv::Rect _crop_box;
  int _resize_width;
  int _resize_height;
  float _unknown_threshold;
};
}
}
}
#endif //GREEN_CLASSIFY_H
