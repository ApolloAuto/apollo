/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/traffic_light/rectify/select.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void GaussianSelect::Select(const cv::Mat &ros_image,
                            const std::vector<LightPtr> &hdmap_bboxes,
                            const std::vector<LightPtr> &refined_bboxes,
                            std::vector<LightPtr> *selected_bboxes) {
  // find bbox with max area in refined_bboxes
  auto max_area_refined_bbox =
      std::max_element(refined_bboxes.begin(), refined_bboxes.end(),
                       [](const LightPtr lhs, const LightPtr rhs) {
                         return lhs->region.rectified_roi.area() <
                             rhs->region.rectified_roi.area();
                       });

//  cv::Mat_<int> cost_matrix(hdmap_bboxes.size(), refined_bboxes.size());
  std::vector<std::vector<double> >
      score_matrix
      (hdmap_bboxes.size(), std::vector<double>(refined_bboxes.size(), 0));
  for (size_t row = 0; row < hdmap_bboxes.size(); ++row) {
    cv::Point2f center_hd = GetCenter(hdmap_bboxes[row]->region.rectified_roi);
    auto width_hd = hdmap_bboxes[row]->region.rectified_roi.width;
    for (size_t col = 0; col < refined_bboxes.size(); ++col) {
      cv::Point2f center_refine =
          GetCenter(refined_bboxes[col]->region.rectified_roi);
      auto width_refine = refined_bboxes[col]->region.rectified_roi.width;

      // use gaussian score as metrics of distance and width
      // distance_score:
      //    larger distance => 0.
      //    smaller distance => 1
      double distance_score = static_cast<double>(
          Get2dGaussianScore(center_hd, center_refine, 100, 100));
      // width score:
      //   larger width diff => 0
      //   smaller width diff => 1
      double width_score =
          static_cast<double>(Get1dGaussianScore(width_hd, width_refine, 100));

      // normalized area score
      // larger area => 1
      double area_score = 1.0 *
          refined_bboxes[col]->region.rectified_roi.area() /
          (*max_area_refined_bbox)->region.rectified_roi.area();

      // when numerator=1， denominator is very small,
      // converting to int might reduce to same value, here uses 1000
      // + 0.05 to prevent score = 0
      // score * weight h
      score_matrix[row][col] =
          (0.05 + 0.4 * refined_bboxes[col]->region.detect_score +
              0.2 * distance_score + 0.2 * width_score + 0.2 * area_score);
    }
  }

  HungarianOptimizer munkres(score_matrix);
  std::vector<int> hd_index;
  std::vector<int> refined_index;
  munkres.maximize(&hd_index, &refined_index);
  for (size_t i = 0; i < hdmap_bboxes.size(); ++i) {
    hdmap_bboxes[i]->region.is_selected = false;
  }
  for (size_t i = 0; i < hd_index.size(); ++i) {
    if (hd_index[i] < 0 ||
        static_cast<size_t>(hd_index[i]) >= hdmap_bboxes.size() ||
        refined_index[i] < 0 ||
        static_cast<size_t>(refined_index[i]) >= refined_bboxes.size() ||
        hdmap_bboxes[hd_index[i]]->region.is_selected) {
      continue;
    }
    if (score_matrix[hd_index[i]][refined_index[i]] > 0) {
      refined_bboxes[refined_index[i]]->region.is_selected = true;
      hdmap_bboxes[hd_index[i]]->region.is_selected = true;
      selected_bboxes->push_back(refined_bboxes[refined_index[i]]);
    }
  }
  for (size_t i = 0; i < hdmap_bboxes.size(); ++i) {
    if (!hdmap_bboxes[i]->region.is_selected) {
      selected_bboxes->push_back(hdmap_bboxes[i]);
    }
  }
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
