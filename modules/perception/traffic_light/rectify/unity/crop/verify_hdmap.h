//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_VERIFY_HDMAP_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_VERIFY_HDMAP_H

#include <traffic_light/interface/green_interface.h>

namespace adu {
namespace perception {
namespace traffic_light {
class HDmapVerify : public IHDMapOperator {
 public:
  HDmapVerify(float hd_scale);

  void init(float hd_scale);

  virtual void verify(cv::Mat &ros_image, std::vector<LightPtr> *lights);

 private:
  float _hd_scale;
};
}
}
}
#endif //PROJECT_HDMAP_VERIFY_H
