/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace apollo {
namespace perception {

// Color: cv::Scalar(B, G, R);

// Basic Colors
cv::Scalar black_color = cv::Scalar(0, 0, 0);
cv::Scalar white_color = cv::Scalar(255, 255, 255);
cv::Scalar magenta_color = cv::Scalar(255, 0, 255);
cv::Scalar purple_color = cv::Scalar(128, 0, 128);
cv::Scalar teal_color = cv::Scalar(128, 128, 0);
cv::Scalar violet_color = cv::Scalar(238, 130, 238);
cv::Scalar pink_color = cv::Scalar(203, 192, 255);
cv::Scalar beige_color = cv::Scalar(220, 245, 245);
cv::Scalar ivory_color = cv::Scalar(240, 255, 255);

// Blue class
cv::Scalar azure_color = cv::Scalar(255, 255, 240);
cv::Scalar cyan_color = cv::Scalar(255, 255, 0);
cv::Scalar sky_blue_color = cv::Scalar(235, 206, 135);
cv::Scalar deep_sky_blue_color = cv::Scalar(255, 191, 0);
cv::Scalar dodger_blue_color = cv::Scalar(255, 144, 30);
cv::Scalar blue_color = cv::Scalar(255, 0, 0);
cv::Scalar medium_blue_color = cv::Scalar(205, 0, 0);
cv::Scalar dark_blue_color = cv::Scalar(139, 0, 0);
cv::Scalar navy_color = cv::Scalar(128, 0, 0);

// Green class
cv::Scalar dark_green_color = cv::Scalar(0, 100, 0);
cv::Scalar lime_color = cv::Scalar(0, 255, 0);
cv::Scalar light_green_color = cv::Scalar(144, 238, 144);
cv::Scalar olive_color = cv::Scalar(0, 128, 128);
cv::Scalar green_color = cv::Scalar(0, 128, 0);

// Red class
cv::Scalar red_color = cv::Scalar(0, 0, 255);
cv::Scalar coral_color = cv::Scalar(80, 127, 255);
cv::Scalar salmon_color = cv::Scalar(144, 128, 250);
cv::Scalar orange_color = cv::Scalar(0, 165, 255);
cv::Scalar yellow_color = cv::Scalar(0, 255, 255);
cv::Scalar maroon_color = cv::Scalar(0, 0, 128);

// Gray class
cv::Scalar light_gray_color = cv::Scalar(100, 100, 100);
cv::Scalar gray_color = cv::Scalar(128, 128, 128);
cv::Scalar dark_gray_color = cv::Scalar(170, 170, 170);
cv::Scalar silver_color = cv::Scalar(192, 192, 192);
}  // namespace perception
}  // namespace apollo
