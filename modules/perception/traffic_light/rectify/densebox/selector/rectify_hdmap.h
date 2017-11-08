//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_HDMAP_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_HDMAP_H

#include <traffic_light/interface/green_interface.h>

namespace adu {
namespace perception {
namespace traffic_light {
class RectifyHDmap : public IRectify {
 public:
  RectifyHDmap(float vert_thresh, float iou_thresh);

  void init(float vert_thresh, float iou_thresh);

  virtual void rectify(cv::Mat &ros_image, std::vector<BoundBox_t> &hdmap_bboxes,
                       std::vector<BoundBox_t> &refined_bboxes);

 private:
  void find_pair(std::vector<BoundBox_t> &boxes, std::vector<std::vector<BoundBox_t>> &pairs);

  float _vert_thresh;
  float _iou_thresh;
};
}
}
}
#endif //GREEN_RECTIFY_HDMAP_H
