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
#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/core/i_rand.h"
#include "modules/perception/common/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace camera {

// void get_vanishing_row_from_pitch(const float k_mat[9], int image_width,
//                                   int image_height, float cam_pitch,
//                                   int *vanishing_row);

// bool get_pitch_from_vanishing_row(const float k_mat[9], int image_width,
//                                   int image_height, float *cam_pitch);

bool draw_vanishing_row_on_image(const cv::Scalar &color, int vanishing_row,
                                 cv::Mat *image);

// bool draw_distance_line_on_image(const float k_mat[9], const cv::Scalar
// &color,
//                                  float cam_height, float cam_pitch,
//                                  float step_z_line, int nr_lines,
//                                  int vanishing_row, cv::Mat *image);

// void sample_pts_given_sp_ep(const float sp[2], const float ep[2],
//                             int sample_pts, int width, int height,
//                             std::vector<Eigen::Vector2f> *pts);

// void gen_lane_pts(int vanishing_row, int image_width, int image_height,
//                   int nr_lane_pts_left, int nr_lane_pts_right, float
//                   pricipal_x,
//                   float shift_x, std::vector<Eigen::Vector2f> *left_pts,
//                   std::vector<Eigen::Vector2f> *right_pts);

void draw_lane_pts(const std::vector<Eigen::Vector2f> &lane_pts,
                   const cv::Scalar &color, cv::Mat *image);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
