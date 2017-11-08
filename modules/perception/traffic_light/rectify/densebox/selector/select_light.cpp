//
// Created by gaohan02 on 16-8-1.
//

#include "module/perception/traffic_light/rectify/densebox/selector/select_light.h"
#include "lib/base/timer.h"

namespace adu {
namespace perception {
namespace traffic_light {

void SelectLight::select(cv::Mat &ros_image, std::vector<BoundBox_t> &hdmap_bboxes,
                         std::vector<BoundBox_t> &refined_bboxes,
                         std::vector<BoundBox_t> &selected_bboxes) {
  adu::perception::base::Timer select_timer;
  select_timer.start();

  // compute the IOU, currently, only choose 1 image for each HD-box
  int top_k = 1;
  selected_bboxes.clear();
  // TODO: when hdmap_num > 1, need to eliminate the reduplicated rect
  int refine_num = (int) refined_bboxes.size();
  int hdmap_num = (int) hdmap_bboxes.size();

  for (int h = 0; h < hdmap_num; h++) {
    BoundBox_t hbox = hdmap_bboxes[h];
    // 1st: $refine_num == 0
    if (refine_num == 0 || !hbox.isValid) {
      hbox.isValid = false;
      hbox.selected = false;
      selected_bboxes.push_back(hbox);
      continue;
    }
    // 2nd: $refine_num > 0
    for (int r = 0; r < refine_num; r++) {
      BoundBox_t &rbox = refined_bboxes[r];
      cv::Rect _inter = rbox.rect & hbox.rect;
      cv::Rect _union = rbox.rect | hbox.rect;
      //rbox.iou = (float)_inter.area() / (float)_union.area();
      rbox.iou = (float) _inter.area() / (float) rbox.rect.area();
      if (rbox.class_id != -1 && rbox.class_id == hbox.class_id) {
        rbox.iou += 1.0f;
      }
    }
    std::stable_sort(refined_bboxes.begin(), refined_bboxes.end(), BoundBox_t::iou_greater);
    // redundant loop: hdmap.size == selected.size
    for (int k = 0; k < top_k && k < refine_num; k++) {
      BoundBox_t dbox = refined_bboxes[k];
      if (dbox.iou >= _iou_thresh) {
        dbox.isValid = true;
        dbox.selected = true;
        dbox.box_id = hbox.box_id;
        dbox.time_stamp = hbox.time_stamp;
        selected_bboxes.push_back(dbox);
      } else {
        dbox = hbox;
        dbox.isValid = false;        // special case: for outter usage
        dbox.selected = true;        // special case:
        selected_bboxes.push_back(dbox);
      }
    }
  }
  uint64_t elapsed_time = select_timer.end("Selection: ");
  AINFO << "Selection:" << elapsed_time << " ms";
}
void SelectLight::init(float iou_thresh) {
  this->_iou_thresh = iou_thresh;
}
SelectLight::SelectLight(float iou_thresh) {
  init(iou_thresh);
}
}
}
}
