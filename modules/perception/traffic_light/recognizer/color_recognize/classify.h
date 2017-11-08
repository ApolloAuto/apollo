//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H

#include <string>
#include <utility>
#include "module/perception/traffic_light/interface/green_interface.h"
#include "caffe/caffe.hpp"

namespace adu {
namespace perception {
namespace traffic_light {

class ClassifyByDenseBoxMulti : public IRefine {
 public:
  ClassifyByDenseBoxMulti(const std::string &_class_net, const std::string &_class_model,
                          float threshold, float scale, unsigned int resize_width,
                          unsigned int resize_height, float rgb_mean[3]);

  void init(const std::string &_class_net, const std::string &_class_model, float threshold,
            float scale, unsigned int resize_width, unsigned int resize_height,
            float *rgb_mean);

  virtual void perform(cv::Mat &ros_image, std::vector<BoundBox_t> &boxes);

  virtual void set_crop_box(BoundBox_t &box);

  ~ClassifyByDenseBoxMulti();

 private:
  void prob_to_color(BoundBox_t &box, const float *out_put_data, float threshold);
  caffe::Net<float> *_classify_net_ptr;
  BoundBox_t _crop_box;
  int _resize_width;
  int _resize_height;
  float _rgb_mean[3];
  float _data_scale;
  float _unknown_threshold;
};
}
}
}
#endif //GREEN_CLASSIFY_H
