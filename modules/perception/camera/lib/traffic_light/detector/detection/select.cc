/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/lib/traffic_light/detector/detection/select.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

bool Select::Init(int rows, int cols) {
  if (rows < 0 || cols < 0) {
    return false;
  }

  munkres_.costs()->Reserve(rows, cols);

  return true;
}

double Select::Calc2dGaussianScore(base::Point2DI p1, base::Point2DI p2,
                                   float sigma1, float sigma2) {
  return std::exp(-0.5 * (static_cast<float>((p1.x - p2.x) * (p1.x - p2.x)) /
                              (sigma1 * sigma1) +
                          (static_cast<float>((p1.y - p2.y) * (p1.y - p2.y)) /
                           (sigma2 * sigma2))));
}

void Select::SelectTrafficLights(
    const std::vector<base::TrafficLightPtr> &refined_bboxes,
    std::vector<base::TrafficLightPtr> *hdmap_bboxes) {
  std::vector<std::pair<size_t, size_t> > assignments;
  munkres_.costs()->Resize(hdmap_bboxes->size(), refined_bboxes.size());

  for (size_t row = 0; row < hdmap_bboxes->size(); ++row) {
    auto center_hd = (*hdmap_bboxes)[row]->region.detection_roi.Center();
    if ((*hdmap_bboxes)[row]->region.outside_image) {
      AINFO << "projection_roi outside image, set score to 0.";
      for (size_t col = 0; col < refined_bboxes.size(); ++col) {
        (*munkres_.costs())(row, col) = 0.0;
      }
      continue;
    }
    for (size_t col = 0; col < refined_bboxes.size(); ++col) {
      float gaussian_score = 100.0f;
      auto center_refine = refined_bboxes[col]->region.detection_roi.Center();
      // use gaussian score as metrics of distance and width
      double distance_score = Calc2dGaussianScore(
          center_hd, center_refine, gaussian_score, gaussian_score);

      double max_score = 0.9;
      auto detect_score = refined_bboxes[col]->region.detect_score;
      double detection_score =
          detect_score > max_score ? max_score : detect_score;

      double distance_weight = 0.7;
      double detection_weight = 1 - distance_weight;
      (*munkres_.costs())(row, col) =
          static_cast<float>(detection_weight * detection_score +
                             distance_weight * distance_score);
      const auto &crop_roi = (*hdmap_bboxes)[row]->region.crop_roi;
      const auto &detection_roi = refined_bboxes[col]->region.detection_roi;
      if ((detection_roi & crop_roi) != detection_roi) {
        AINFO << "detection_roi outside crop_roi, set score to 0."
              << " detection_roi: " << detection_roi.x << " " << detection_roi.y
              << " " << detection_roi.width << " " << detection_roi.height
              << " crop_roi: " << crop_roi.x << " " << crop_roi.y << " "
              << crop_roi.width << " " << crop_roi.height;
        (*munkres_.costs())(row, col) = 0.0;
      }
      AINFO << "score " << (*munkres_.costs())(row, col);
    }
  }

  munkres_.Maximize(&assignments);

  for (size_t i = 0; i < hdmap_bboxes->size(); ++i) {
    (*hdmap_bboxes)[i]->region.is_selected = false;
    (*hdmap_bboxes)[i]->region.is_detected = false;
  }

  for (size_t i = 0; i < assignments.size(); ++i) {
    if (static_cast<size_t>(assignments[i].first) >= hdmap_bboxes->size() ||
        static_cast<size_t>(
            assignments[i].second >= refined_bboxes.size() ||
            (*hdmap_bboxes)[assignments[i].first]->region.is_selected ||
            refined_bboxes[assignments[i].second]->region.is_selected)) {
    } else {
      auto &refined_bbox_region = refined_bboxes[assignments[i].second]->region;
      auto &hdmap_bbox_region = (*hdmap_bboxes)[assignments[i].first]->region;
      refined_bbox_region.is_selected = true;
      hdmap_bbox_region.is_selected = true;

      const auto &crop_roi = hdmap_bbox_region.crop_roi;
      const auto &detection_roi = refined_bbox_region.detection_roi;
      bool outside_crop_roi = ((crop_roi & detection_roi) != detection_roi);
      if (hdmap_bbox_region.outside_image || outside_crop_roi) {
        hdmap_bbox_region.is_detected = false;
      } else {
        hdmap_bbox_region.detection_roi = refined_bbox_region.detection_roi;
        hdmap_bbox_region.detect_class_id = refined_bbox_region.detect_class_id;
        hdmap_bbox_region.detect_score = refined_bbox_region.detect_score;
        hdmap_bbox_region.is_detected = refined_bbox_region.is_detected;
        hdmap_bbox_region.is_selected = refined_bbox_region.is_selected;
      }
    }
  }

  for (size_t i = 0; i < hdmap_bboxes->size(); ++i) {
    AINFO << "hdmap_bboxes-" << i << ":"
          << " projection_roi: "
          << (*hdmap_bboxes)[i]->region.projection_roi.ToStr()
          << " detection_roi: "
          << (*hdmap_bboxes)[i]->region.detection_roi.ToStr();
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
