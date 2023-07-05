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

#include <algorithm>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/core/i_rand.h"
#include "modules/perception/common/i_lib/geometry/i_util.h"

namespace adu {
namespace perception {
namespace obstacle {

// void change_suffix(std::string file_path, std::string suffix,
//                    std::string *file_path_changed);

bool load_filename(std::string path, std::string suffix,
                   std::vector<std::string> *name_list);

// bool load_ref_camera_p_mat(const std::string &filename, float p_mat[12]);
bool load_ref_camera_k_mat(const std::string &filename, float k_mat[9], int *w,
                           int *h);

// void draw_2d_bbox(cv::Mat *image, float left, float top, float right,
//                   float bottom, const cv::Scalar &color);

// void draw_2d_face(cv::Mat *image, const float corners_2d[16],
//                   const int idx_points[4], const cv::Scalar &color);

void write_text_on_image(cv::Mat *image, float left, float top,
                         const char *text, const CvFont &font,
                         const cv::Scalar &color);

// void add_noise_to_vector_radius(float *x, int n, float radius,
//                                 bool set_seed = false);

// void add_noise_to_vector_ratio(float *x, int n, float ratio,
//                                bool set_seed = false);

}  // namespace obstacle
}  // namespace perception
}  // namespace adu
