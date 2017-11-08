//
// Created by gaohan02 on 17-5-23.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_SELECT_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_SELECT_H

#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {
class Select : public ISelectLight {
 public:
  Select() = default;

  virtual void select(const cv::Mat &ros_image, const std::vector<LightPtr> &hdmap_bboxes,
                      const std::vector<LightPtr> &refined_bboxes,
                      std::vector<LightPtr> *selected_bboxes);
};
}
}
}

#endif //PERCEPTION_MATCH_H
