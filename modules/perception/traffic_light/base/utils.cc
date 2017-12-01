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

#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

uint64_t TimestampDouble2Int64(double ts) {
  uint64_t result = (uint64_t) (ts * 1e6);
  return result * 1000;
}

void ClearBox(cv::Rect *rect) {
  rect->x = 0;
  rect->y = 0;
  rect->width = 0;
  rect->height = 0;
}
bool BoxIsValid(const cv::Rect &box, const cv::Size &size) {
  if (box.width <= 0 || box.height <= 0
      || box.x >= size.width || box.y >= size.height
      || box.x + box.width - 1 < 0 || box.y + box.height - 1 < 0) {
    return false;
  }
  return true;
}
cv::Rect RefinedBox(const cv::Rect inbox, const cv::Size &size) {
  cv::Rect box(inbox);
  int cols = size.width;
  int rows = size.height;
  if (box.x < 0) {
    box.width += box.x;
    box.x = 0;
  }
  if (box.y < 0) {
    box.height += box.y;
    box.y = 0;
  }
  if (box.x >= cols) {
    box.x = box.width = 0;
  }
  if (box.y >= rows) {
    box.y = box.height = 0;
  }
  box.width = (box.width + box.x < cols) ? box.width : cols - box.x - 1;
  box.height = (box.height + box.y < rows) ? box.height : rows - box.y - 1;

  if (box.width < 0) {
    box.width = 0;
  }
  if (box.height < 0) {
    box.height = 0;
  }
  return box;
}

cv::Point2f GetCenter(const cv::Rect &box) {
  return cv::Point2f(box.x + box.width / 2, box.y + box.height / 2);
}
float GetDistance(const cv::Point2f &p1, const cv::Point2f &p2) {
  return std::sqrt(
      (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

float Get2dGaussianScore(const cv::Point2f &p1,
                         const cv::Point2f &p2,
                         float sigma1,
                         float sigma2) {
  return static_cast<float>(std::exp(
      -0.5 * (((p1.x - p2.x) * (p1.x - p2.x)) / (sigma1 * sigma1) +
          ((p1.y - p2.y) * (p1.y - p2.y)) / (sigma2 * sigma2))));
}

float Get1dGaussianScore(float x1, float x2, float sigma) {
  return static_cast<float>(std::exp(
      -0.5 * (x1 - x2) * (x1 - x2) / (sigma * sigma)));
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
