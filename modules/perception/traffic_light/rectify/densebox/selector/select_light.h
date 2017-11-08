//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_SELECT_LIGHT_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_SELECT_LIGHT_H

#include <traffic_light/interface/green_interface.h>

namespace adu {
namespace perception {
namespace traffic_light {

class SelectLight : public ISelectLight {
 public:
  SelectLight(float iou_thresh);

  void init(float iou_thresh);

  virtual void select(cv::Mat &ros_image, std::vector<BoundBox_t> &hdmap_bboxes,
                      std::vector<BoundBox_t> &refined_bboxes,
                      std::vector<BoundBox_t> &selected_bboxes);

 private:
  float _iou_thresh;
};
}
}
}
#endif //GREEN_SELECT_LIGHT_H
