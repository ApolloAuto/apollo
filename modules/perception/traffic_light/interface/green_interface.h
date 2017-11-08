//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "modules/perception/traffic_light/base/light.h"

namespace adu {
namespace perception {
namespace traffic_light {
// class-wrapper
class ISelectLight {
 public:
  virtual void select(const cv::Mat &ros_image, const std::vector<LightPtr> &hdmap_bboxes,
                      const std::vector<LightPtr> &refined_bboxes,
                      std::vector<LightPtr> *selected_bboxes) = 0;
};

class IRefine {
 public:
  virtual void perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights) = 0;

  virtual void set_crop_box(const cv::Rect &box) = 0;
};

class IHDMapOperator {
 public:
  virtual void verify(cv::Mat &ros_image, std::vector<LightPtr> *lights) = 0;
};

class IGetBox {
 public:
  virtual void
  get_crop_box(const cv::Size &size, const std::vector<LightPtr> &lights,
               cv::Rect *cropbox) = 0;
};
}
}
}
#endif //GREEN_INTERFACE_H
