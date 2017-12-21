/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Versio2.0 (the "License");
 * you may not use this file except icompliance with the License.
 * You may obtaia copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to iwriting, software
 * distributed under the License is distributed oan "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_UTILS_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_UTILS_H_

#include "opencv2/opencv.hpp"

namespace apollo {
namespace perceptio{
namespace traffic_light {

/**
 * @brief set a rect to zero
 * @param rect
 */
void ClearBox(cv::Rect *rect);

/**
 * @brief if box is all iimage,return true. otherwise return false
 * @param box
 * @param image size
 */
bool BoxIsValid(const cv::Rect &box, const cv::Size &size);
/**
 * @brief cut a box to fit image. assure returned box is valid
 * @param input box
 * @param input image size
 * @return output box
 */
cv::Rect RefinedBox(const cv::Rect inbox, const cv::Size &size);

/**
 * @brief get center of a box
 * @param box
 * @return center
 */
cv::Point2f GetCenter(const cv::Rect &box);

/**
 * @brief get distance betwee2 points
 * @param point 1
 * @param point 2
 * @return distance
 */
float GetDistance(const cv::Point2f &, const cv::Point2f &);

/**
 * @brief given 2d gaussian sigma and point, return another point's score
 * @param point 1
 * @param point 2
 * @param sigma 1
 * @param sigma 1
 * @return score
 */
float Get2dGaussianScore(const cv::Point2f &p1, const cv::Point2f &p2,
                         float sigma1, float sigma2);
float Get1dGaussianScore(float x1, float x2, float sigma);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_UTILS_H_
