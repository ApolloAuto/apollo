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
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_app_util.h"

#include <fstream>

#include "modules/perception/common/i_lib/core/i_basic.h"

namespace apollo {
namespace perception {
namespace obstacle {

// void change_suffix(boost::filesystem::path file_path, std::string suffix,
//                    std::string *file_path_changed) {
//   boost::filesystem::path file_name_tmp = file_path;
//   file_name_tmp.replace_extension(suffix);
//   *file_path_changed = file_name_tmp.string();
// }

// void change_suffix(std::string file_path, std::string suffix,
//                    std::string *file_path_changed) {
//   boost::filesystem::path path = file_path;
//   change_suffix(path, suffix, file_path_changed);
// }

bool load_filename(std::string path, std::string suffix,
                   std::vector<std::string> *name_list) {
  assert(name_list != nullptr);
  name_list->clear();
  boost::filesystem::directory_iterator end_itr;
  boost::filesystem::directory_iterator iter(path);
  boost::filesystem::path file;
  while (iter != end_itr) {
    file = *iter;
    if (!suffix.empty()) {
      std::string extension = file.extension().string();
      std::transform(extension.begin(), extension.end(), extension.begin(),
                     ::tolower);
      if (extension != suffix) {
        continue;
      }
    }
    std::string filename = file.string();
    name_list->push_back(filename);
    iter++;
  }
  if (name_list->size() == 0) {
    return false;
  }
  std::sort(name_list->begin(), name_list->end());  // in dictionary order
  return true;
}

// bool load_ref_camera_p_mat(const std::string &filename, float p_mat[12]) {
//   std::fstream fin(filename);
//   if (!fin.is_open()) {
//     std::cerr << "Fail to load the camera p matrix: " << filename <<
//     std::endl;
//     return false;
//   }
//   fin >> p_mat[0] >> p_mat[1] >> p_mat[2] >> p_mat[3] >> p_mat[4] >> p_mat[5]
//   >>
//       p_mat[6] >> p_mat[7] >> p_mat[8] >> p_mat[9] >> p_mat[10] >> p_mat[11];
//   fin.close();
//   return true;
// }

bool load_ref_camera_k_mat(const std::string &filename, float k_mat[9], int *w,
                           int *h) {
  std::fstream fin(filename);
  if (!fin.is_open()) {
    std::cerr << "Fail to load the camera k matrix: " << filename << std::endl;
    return false;
  }
  float wh_flt[2] = {0};
  fin >> wh_flt[0] >> wh_flt[1];
  *w = common::IRound(wh_flt[0]);
  *h = common::IRound(wh_flt[1]);
  fin >> k_mat[0] >> k_mat[1] >> k_mat[2] >> k_mat[3] >> k_mat[4] >> k_mat[5] >>
      k_mat[6] >> k_mat[7] >> k_mat[8];
  fin.close();
  return true;
}

// void draw_2d_bbox(cv::Mat *image, float left, float top, float right,
//                   float bottom, const cv::Scalar &color) {
//   cv::rectangle(*image, cvPoint(common::IRound(left), common::IRound(top)),
//                 cvPoint(common::IRound(right),
//                 common::IRound(bottom)), color, 2,
//                 1,
//                 0);
// }

// void draw_2d_face(cv::Mat *image, const float corners_2d[16],
//                   const int idx_points[4], const cv::Scalar &color) {
//   for (int i = 0; i < 4; ++i) {
//     int i_cur2 = idx_points[i] << 1;
//     int i_next2 = idx_points[((i + 1) % 4)] << 1;
//     cv::line(*image, cvPoint(common::IRound(corners_2d[i_cur2]),
//                              common::IRound(corners_2d[i_cur2 + 1])),
//              cvPoint(common::IRound(corners_2d[i_next2]),
//                      common::IRound(corners_2d[i_next2 + 1])),
//              color, 2, 8, 0);
//   }
// }

void write_text_on_image(cv::Mat *image, float left, float top,
                         const char *text, const CvFont &font,
                         const cv::Scalar &color) {
  IplImage ipl_img = *image;
  cvPutText(&ipl_img, text, cvPoint(common::IRound(left), common::IRound(top)),
            &font, color);
}

// void add_noise_to_vector_radius(float *x, int n, float radius, bool set_seed)
// {
//   unsigned int seed = time(NULL);
//   for (int i = 0; i < n; ++i) {
//     float dx = radius * (rand_r(&seed) / RAND_MAX - 0.5f) * 2;
//     /*
//     std::cout << "|i, dx: " << i << ", " << dx;
//     */
//     x[i] += dx;
//   }
//   // std::cout << std::endl;
// }

// void add_noise_to_vector_ratio(float *x, int n, float ratio, bool set_seed) {
//   unsigned int seed = time(NULL);
//   for (int i = 0; i < n; ++i) {
//     float radius = fabs(x[i]) * ratio;
//     float dx = radius * (rand_r(&seed) / RAND_MAX - 0.5f) * 2;
//     /*
//     std::cout << "|i, dx: " << i << ", " << dx;
//     */
//     x[i] += dx;
//   }
//   // std::cout << std::endl;
// }

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
