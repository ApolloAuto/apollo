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
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_lane_calibrator_util.h"  // NOLINT
#include "modules/perception/common/i_lib/core/i_basic.h"

namespace apollo {
namespace perception {
namespace camera {

// void get_vanishing_row_from_pitch(const float k_mat[9], int image_width,
//                                   int image_height, float cam_pitch,
//                                   int *vanishing_row) {
//   assert(vanishing_row != nullptr);
//   const float cy = k_mat[5];
//   const float fy = k_mat[4];
//   float y = tan(cam_pitch) * fy + cy;
//   *vanishing_row = common::IRound(y);
// }

// bool get_pitch_from_vanishing_row(const float k_mat[9], int image_width,
//                                   int image_height, int vanishing_row,
//                                   float *cam_pitch) {
//   assert(cam_pitch != nullptr);
//   const float cy = k_mat[5];
//   const float fy = k_mat[4];
//   float y = static_cast<float>(vanishing_row);
//   *cam_pitch = atan2(y - cy, fy);
// }

bool draw_vanishing_row_on_image(const cv::Scalar &color, int vanishing_row,
                                 cv::Mat *image) {
  assert(image != nullptr);
  int h = image->rows;
  int w = image->cols;
  assert(h > 0);
  assert(w > 0);

  if (vanishing_row <= 0 || vanishing_row > h - 1) {
    return false;
  }

  int xl = 0;
  int yl = vanishing_row;
  int xr = w - 1;
  int yr = vanishing_row;
  cv::line(*image, cvPoint(xl, yl), cvPoint(xr, yr), color, 1);
  return true;
}

// bool draw_distance_line_on_image(const float k_mat[9], const cv::Scalar
// &color,
//                                  float cam_height, float cam_pitch,
//                                  float step_z_line, int nr_lines,
//                                  int vanishing_row, cv::Mat *image) {
//   int h = image->rows;
//   int w = image->cols;
//   assert(step_z_line > 0.0f);
//   assert(h > 0);
//   assert(w > 0);
//   float sin_pitch = sin(cam_pitch);
//   float cos_pitch = cos(cam_pitch);
//   float ground_plane[4] = {0, cos_pitch, -sin_pitch, -cam_height};
//   float pt[3] = {0};
//   float pixel[3] = {0};
//   for (int i = 0; i < nr_lines; ++i) {
//     pt[2] += step_z_line;
//     pt[1] = (-ground_plane[3] - ground_plane[2] * pt[2]) *
//             common::i_rec(ground_plane[1]);

//     //  project onto image
//     common::i_project_through_intrinsic(k_mat, pt, pixel);
//     common::i_scale3(pixel, common::i_rec(pixel[2]));
//     int y = common::IRound(pixel[1]);
//     if (y > vanishing_row) {
//       cv::line(*image, cvPoint(0, y), cvPoint(w - 1, y), color, 1);
//     }
//   }
//   return true;
// }

// void sample_pts_given_sp_ep(const float sp[2], const float ep[2],
//                             int sample_pts, int width, int height,
//                             std::vector<Eigen::Vector2f> *pts) {
//   pts->clear();
//   assert(sample_pts > 1);
//   if (sample_pts == 2) {
//     Eigen::Vector2f pt1;
//     pt1 << sp[0], sp[1];
//     Eigen::Vector2f pt2;
//     pt2 << ep[0], ep[1];
//     pts->push_back(pt1);
//     pts->push_back(pt2);
//   }
//   float dx = (ep[0] - sp[0]) / (sample_pts - 1);
//   float dy = (ep[1] - sp[1]) / (sample_pts - 1);
//   float offset_x = 0.0f;
//   float offset_y = 0.0f;
//   for (int i = 0; i < sample_pts; ++i) {
//     float x = sp[0] + offset_x;
//     float y = sp[1] + offset_y;
//     offset_x += dx;
//     offset_y += dy;

//     if (x < 0.0f || common::IRound(x) > width - 1 || y < 0.0f ||
//         common::IRound(y) > height - 1) {
//       continue;
//     }

//     Eigen::Vector2f pt;
//     pt << x, y;
//     pts->push_back(pt);
//   }
// }

// void gen_lane_pts(int vanishing_row, int image_width, int image_height,
//                   int nr_lane_pts_left, int nr_lane_pts_right, float
//                   pricipal_x,
//                   float shift_x, std::vector<Eigen::Vector2f> *left_pts,
//                   std::vector<Eigen::Vector2f> *right_pts) {
//   assert(shift_x * 2 < image_width);
//   float v_point[2] = {pricipal_x, static_cast<float>(vanishing_row)};
//   float left_bottom[2] = {shift_x, static_cast<float>(image_height - 2)};
//   float right_bottom[2] = {static_cast<float>(image_width - shift_x),
//                            static_cast<float>(image_height - 2)};

//   float offset = pricipal_x - image_width / 2;
//   left_bottom[0] += offset;
//   right_bottom[0] += offset;

//   sample_pts_given_sp_ep(left_bottom, v_point, nr_lane_pts_left, image_width,
//                          image_height, left_pts);
//   sample_pts_given_sp_ep(right_bottom, v_point, nr_lane_pts_right,
//   image_width,
//                          image_height, right_pts);
// }

void draw_lane_pts(const std::vector<Eigen::Vector2f> &lane_pts,
                   const cv::Scalar &color, cv::Mat *image) {
  if (image->rows <= 0 || image->cols <= 0) {
    return;
  }
  for (auto &pt : lane_pts) {
    int xc = common::IRound(pt(0));
    int yc = common::IRound(pt(1));
    cv::circle(*image, cvPoint(xc, yc), 3, color);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
