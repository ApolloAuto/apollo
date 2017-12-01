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
#include "modules/perception/traffic_light/rectify/munkres.h"
#include "modules/perception/traffic_light/base/utils.h"

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

  cv::Mat_<int> cost_matrix(hdmap_bboxes.size(), refined_bboxes.size());
  for (int row = 0; row < hdmap_bboxes.size(); ++row) {
    cv::Point2f center_hd = GetCenter(hdmap_bboxes[row]->region.rectified_roi);
    auto width_hd = hdmap_bboxes[row]->region.rectified_roi.width;
    for (int col = 0; col < refined_bboxes.size(); ++col) {
      cv::Point2f
          center_refine = GetCenter(refined_bboxes[col]->region.rectified_roi);
      auto width_refine = refined_bboxes[col]->region.rectified_roi.width;

      // use gaussian score as metrics of distance and width
      // 距离分数，距离越大，越接近 0；距离越小，越接近 1
      double distance_score = static_cast<double>(
          Get2dGaussianScore(center_hd, center_refine, 100, 100));
      // 宽度分数，宽度差异越大，越接近 0；差异越小，越接近 1
      double width_score = static_cast<double>(
          Get1dGaussianScore(width_hd, width_refine, 100));

      // normalized area score
      // 归一化的面积分数，面积越大，越接近 1
      double
          area_score = 1.0 * refined_bboxes[col]->region.rectified_roi.area() /
          (*max_area_refined_bbox)->region.rectified_roi.area();

      // 如果分子取 1，当分母变化小时，转 int 后可能是相同的值，这里取 1000
      // 加 0.05 是防止所有 score 都为 0
      // 每个 score 乘以一个权重h
      cost_matrix[row][col] = (int) (1000.0 /
          (0.05 + 0.4 * refined_bboxes[col]->region.detect_score +
              0.2 * distance_score +
              0.2 * width_score +
              0.2 * area_score));
    }
  }
  std::cout << cost_matrix << std::endl;
  Munkres munkres;
  munkres.Solve(cost_matrix);
  for (int row = 0; row < hdmap_bboxes.size(); ++row) {
    bool find = false;
    for (int col = 0; col < refined_bboxes.size(); ++col) {
      if (cost_matrix(row, col) == 0) {
        refined_bboxes[col]->region.is_selected = true;
        selected_bboxes->push_back(refined_bboxes[col]);
        find = true;
        break;
      }
    }
    if (!find) {
      hdmap_bboxes[row]->region.is_selected = false;
      selected_bboxes->push_back(hdmap_bboxes[row]);
    }
  }

}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
