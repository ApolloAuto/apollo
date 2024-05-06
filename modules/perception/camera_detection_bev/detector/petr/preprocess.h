/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <vector>

#include "opencv2/opencv.hpp"

namespace apollo {
namespace perception {
namespace camera {
/**
 * @brief Image resize function
 *
 * @param img The image to be resized.
 * @param width The width of the image.
 * @param height The height of the image.
 */
void Resize(cv::Mat *img, cv::Mat *img_n, int width, int height);

/**
 * @brief Image crop function
 *
 * @param img The image to be cropped.
 * @param x The x coordinate of the left side of the crop.
 * @param y The y coordinate of the top side of the crop.
 * @param width The width of the crop image.
 * @param height The height of the crop image.
 */
void Crop(cv::Mat *img, cv::Mat *img_n, int x, int y, int width, int height);

/**
 * @brief Image normalize function
 *
 * @param mean The mean value of the image.
 * @param std The standard deviation of the image.
 * @param scale The scale value of the image.
 * @param im The image to be normalized.
 */
void Normalize(const std::vector<float> &mean, const std::vector<float> &std,
               float scale, cv::Mat *img);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
