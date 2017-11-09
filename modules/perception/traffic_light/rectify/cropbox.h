//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_CROPBOX_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_CROPBOX_H

#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {
class CropBox : public IGetBox {
 public:
  CropBox(float crop_scale, float min_crop_size);

  void Init(float crop_scale, float min_crop_size);

  virtual void
  GetCropBox(const cv::Size &size, const std::vector<LightPtr> &lights, cv::Rect
  *cropbox);

 private:
  float crop_scale_;
  float min_crop_size_;
};

class CropBoxWholeImage : public IGetBox {
 public:
  virtual void
  GetCropBox(const cv::Size &size, const std::vector<LightPtr> &lights, cv::Rect
  *cropbox);
};

}
}
}
#endif //GREEN_CROPBOX_H
