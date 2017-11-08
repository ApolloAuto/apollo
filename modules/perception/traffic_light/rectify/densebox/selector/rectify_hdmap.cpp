//
// Created by gaohan02 on 16-8-1.
//

#include "module/perception/traffic_light/rectify/densebox/selector/rectify_hdmap.h"
#include "module/perception/traffic_light/base/utils.h"
#include "lib/base/timer.h"

namespace adu {
namespace perception {
namespace traffic_light {

void RectifyHDmap::find_pair(std::vector<BoundBox_t> &boxes,
                             std::vector<std::vector<BoundBox_t>> &pairs) {
  std::stable_sort(boxes.begin(), boxes.end(), BoundBox_t::vertical_less);
  int box_num = boxes.size();

  /*
   * 把传入的框按照高度排序
   * 比较相邻两个框的高度IOU
   * 每个框与前一个框的IOU足够高则认为是同一组灯
   * 否则认为是新一组灯的开始
   */
  std::vector<BoundBox_t> pair;
  BoundBox_t lastbox;
  int i = 0;
  for (i = 0; i < box_num; i++) {
    if (boxes[i].isValid) {
      boxes[i].detection_id = i;
      pair.push_back(boxes[i]);
      lastbox = boxes[i];
      ++i;
      break;
    }
  }

  for (; i < box_num; ++i) {
    if (!boxes[i].isValid) {
      continue;
    }
    boxes[i].detection_id = i;
    int top = std::max(boxes[i].rect.y, lastbox.rect.y);
    int bottom = std::min(boxes[i].rect.br().y, lastbox.rect.br().y);
    float vert_ratio = float(bottom - top) / lastbox.rect.height;
    if (vert_ratio >= _vert_thresh) {
      pair.push_back(boxes[i]);
    } else {
      std::stable_sort(pair.begin(), pair.end(), BoundBox_t::horizontal_less);
      pairs.push_back(pair);
      pair.clear();
      pair.push_back(boxes[i]);
    }
    lastbox = boxes[i];
  }
  if (pair.size() > 0) {
    std::stable_sort(pair.begin(), pair.end(), BoundBox_t::horizontal_less);
    pairs.push_back(pair);
  }
}
void RectifyHDmap::rectify(cv::Mat &ros_image, std::vector<BoundBox_t> &hdmap_bboxes,
                           std::vector<BoundBox_t> &refined_bboxes) {
  using cv::Point2f;

  adu::perception::base::Timer rect_timer;
  rect_timer.start();
  // 0, prepare for data set
  int refine_num = (int) refined_bboxes.size();
  int hdmap_num = (int) hdmap_bboxes.size();
  if (!refine_num || !hdmap_num) {
    return;
  }
  std::vector<std::vector<BoundBox_t>> det_pairs;
  std::vector<std::vector<BoundBox_t>> hd_pairs;
  det_pairs.clear();
  hd_pairs.clear();
  // 1, process the detected bboxes
  if (refine_num > 0) {
    find_pair(refined_bboxes, det_pairs);
  }
  // 2, process the hdmap bboxes
  if (hdmap_num) {
    find_pair(hdmap_bboxes, hd_pairs);
    for (int i = 0; i < hd_pairs.size(); ++i) {
      for (int j = 0; j < hd_pairs[i].size(); ++j) {
        hd_pairs[i][j].class_id = i;
      }
    }
  }
  // 3, search for the matched pairs
  // [zoo] hdmap search for its matches, use the max-overlapped pair
  Point2f loc_delta;
  float loc_delta_sum = 0;
  int hd_class_num = (int) hd_pairs.size();
  int det_class_num = (int) det_pairs.size();
  bool hd_mod = false;
  for (int hc = 0; hc < hd_class_num; hc++) {    // hc: Hdmap Class
    loc_delta.x = loc_delta.y = loc_delta_sum = 0;
    // for each hd pair:
    std::vector<BoundBox_t> &hd_pair = hd_pairs[hc];
    bool hd_match = false;
    bool hd_best_match = false;
    int hd_best_matid = -1;
    float hd_best_delta_sum = 0;
    cv::Point2f hd_best_delta;
    // if found the best match, the det box record the related hd box's id
    //for(int h = 0, dc= 0; h < hd_class_num && dc < det_class_num; dc++, h=hd_mat)
    for (int dc = 0; dc < det_class_num; dc++) {
      //	hd_match = false;
      std::vector<BoundBox_t> &dt_pair = det_pairs[dc];
      // compare the candidated det pair(s): hd --> det
      //BoundBox_t hd_guard = hd_pair[0];
      BoundBox_t hd_guard_l = hd_pair[0];
      BoundBox_t hd_guard_r = hd_pair[hd_pair.size() - 1];
      int dp = 0;
      int hp = 0;
      int det = 0;
      for (dp = 0; dp < dt_pair.size(); dp++) {    // inner pair: best match ?
        BoundBox_t dt_guard = dt_pair[dp];
        Point2f dt_core = get_center(dt_guard.rect);
        Point2f hd_core = get_center(hd_guard_l.rect);
        Point2f tmp_delta_l = dt_core - hd_core;
        float tmp_delta_sum_l = tmp_delta_l.dot(tmp_delta_l);
        // right one
        Point2f tmp_delta_r(1920, 1080);
        float tmp_delta_sum_r = tmp_delta_r.dot(tmp_delta_r);
        if (dt_pair.size() == 1) {
          hd_core = get_center(hd_guard_r.rect);
          tmp_delta_r = dt_core - hd_core;
          tmp_delta_sum_r = tmp_delta_r.dot(tmp_delta_r);
        }
        // smaller one
        Point2f tmp_delta;
        float tmp_delta_sum = 0;
        if (tmp_delta_sum_l <= tmp_delta_sum_r) {
          tmp_delta = tmp_delta_l;
          tmp_delta_sum = tmp_delta_sum_l;
        } else {
          tmp_delta = tmp_delta_r;
          tmp_delta_sum = tmp_delta_sum_r;
        }
        // NOTE: update the $det
        for (hp = 1, det = dp + 1; hp < hd_pair.size() && det < dt_pair.size(); det++) {
          BoundBox_t dbox = dt_pair[det];
          BoundBox_t hbox = hd_pair[hp];

          hbox.rect.x = int(hbox.rect.x + tmp_delta.x);
          hbox.rect.y = int(hbox.rect.y + tmp_delta.y);
          cv::Rect _inter = dbox.rect & hbox.rect;
          cv::Rect _union = dbox.rect | hbox.rect;
          dbox.iou = float(_inter.area()) / float(dbox.rect.area());
          if (dbox.iou < _iou_thresh) {
            continue;
          } else {
            hp += 1;
          }
        }

        // TODO, coarse-grained, only process pair(s)
        // if found, jump out
        if (!hd_match || hp == hd_pair.size() || hp >= 2) {
          // TODO do statistics, record the tiniest change
          if (loc_delta_sum < 0.01 || loc_delta_sum > tmp_delta_sum) {
            loc_delta_sum = tmp_delta_sum;
            loc_delta = tmp_delta;
          }
          if (hp >= 2 && !hd_match) {    // first found a pair:
            hd_best_matid = dc;        // store the hdmap-pair class's id
            hd_best_match = true;    // special case: use the current delta
            hd_best_delta_sum = tmp_delta_sum /*loc_delta_sum*/;
            hd_best_delta = tmp_delta /*loc_delta_x*/;
            loc_delta_sum = tmp_delta_sum;
            loc_delta = tmp_delta;
          } else if (hp >= 2 && hd_match) {    // found a new pair:
            if (hd_best_delta_sum > loc_delta_sum) {
              hd_best_matid = dc;        // store the hdmap-pair class's id
              hd_best_match = true;
              hd_best_delta_sum = loc_delta_sum;
              hd_best_delta = loc_delta;
            }
          } else if (!hd_match) {    // found other types of mathc
            //	if(hd_best_delta_sum == 0 || hd_best_delta_sum > loc_delta_sum)
            if (hd_best_delta_sum < 0.01 || hd_best_delta_sum > loc_delta_sum) {
              hd_best_matid = dc;        // store the hdmap-pair class's id
              hd_best_match = true;
              hd_best_delta_sum = loc_delta_sum;
              hd_best_delta = loc_delta;
            }
          }
          // if found a pair(match) i.e. >= 2, exclude case of hd.size() == 1
          if (hp >= 2) {
            hd_match = true;
          }
          //break;		// inner the det class: first match
        }
      }    // end dp
    }
    if (hd_best_match) {
      // update switch
      hd_mod = true;
      // update hdmap
      for (int m = 0; m < hd_pair.size(); m++) {
        hd_pairs[hc][m].rect.x = (int) (hd_pair[m].rect.x + hd_best_delta.x);
        hd_pairs[hc][m].rect.y = (int) (hd_pair[m].rect.y + hd_best_delta.y);
      }
      // update refined bboxes' class_id
      std::vector<BoundBox_t> &dt_pair = det_pairs[hd_best_matid];
      int hd_class_id = hd_pair[0].class_id;
      for (int d = 0; d < dt_pair.size(); d++) {
        BoundBox_t dbox = dt_pair[d];
        refined_bboxes[dbox.detection_id].class_id = hd_class_id;
      }
      // TODO: if to find the best match, do not jump out of loop
    } else {
      // update counter
    }
  }
  // 4, gather delta, do the statistics (e.f., expected delta, avg, variance)

  // 5, dump out the modified hdmap/reconstruct the hdmap
  if (hd_mod) {    //TODO; pass refs to the pairs to eliminate the extra copies here
    for (int c = 0, h = 0; c < hd_pairs.size(); c++) {
      for (int p = 0; p < hd_pairs[c].size(); p++) {
        while (!hdmap_bboxes[h].isValid) {
          h++;
        }
        if (h < hdmap_bboxes.size()) {
          hdmap_bboxes[h++] = hd_pairs[c][p];
        }
      }
    }
  }
  uint64_t elapsed_time = rect_timer.end("Rectify: ");
  AINFO << "Rectify:" << elapsed_time << " ms";
}
void RectifyHDmap::init(float vert_thresh, float iou_thresh) {
  _vert_thresh = vert_thresh;
  _iou_thresh = iou_thresh;
}
RectifyHDmap::RectifyHDmap(float vert_thresh, float iou_thresh) {
  init(vert_thresh, iou_thresh);
}
}
}
}