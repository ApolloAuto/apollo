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
#include "modules/perception/lane_detection/common/common_functions.h"

#include <algorithm>

namespace apollo {
namespace perception {
namespace camera {

/** DisjointSet **/
// add a new element, which is a subset by itself;
int DisjointSet::Add() {
  int cur_size = static_cast<int>(disjoint_array_.size());
  disjoint_array_.push_back(cur_size);
  ++subset_num_;
  return cur_size;
}

int DisjointSet::Find(int x) {
  if (disjoint_array_[x] == x) {
    return x;
  }

  int y = x;
  while (y != disjoint_array_[y]) {
    y = disjoint_array_[y];
  }
  while (true) {
    const int z = disjoint_array_[x];
    if (z == x) {
      break;
    }
    disjoint_array_[x] = y;
    x = z;
  }
  return y;
}

// point the x and y to smaller root of the two
void DisjointSet::Unite(int x, int y) {
  if (x == y) {
    return;
  }
  int x_root = Find(x);
  int y_root = Find(y);
  if (x_root == y_root) {
    return;
  } else if (x_root < y_root) {
    disjoint_array_[y_root] = x_root;
  } else {
    disjoint_array_[x_root] = y_root;
  }
  --subset_num_;
}

/** ConnectedComponent **/
void ConnectedComponent::AddPixel(int x, int y) {
  base::Point2DI point;
  point.x = x;
  point.y = y;
  pixels_.push_back(point);
  bbox_.xmin = std::min(x, bbox_.xmin);
  bbox_.xmax = std::max(x, bbox_.xmax);
  bbox_.ymin = std::min(y, bbox_.ymin);
  bbox_.ymax = std::max(y, bbox_.ymax);
  pixel_count_++;
}

bool FindCC(const std::vector<unsigned char>& src, int width, int height,
            const base::RectI& roi, std::vector<ConnectedComponent>* cc) {
  if (src.empty()) {
    AERROR << "input image is empty";
    return false;
  }

  cc->clear();
  int x_min = roi.x;
  int y_min = roi.y;
  int x_max = x_min + roi.width - 1;
  int y_max = y_min + roi.height - 1;
  if (x_min < 0) {
    AERROR << "x_min is less than zero: " << x_min;
    return false;
  }
  if (y_min < 0) {
    AERROR << "y_min is less than zero: " << y_min;
    return false;
  }
  if (x_max >= width) {
    AERROR << "x_max is larger than image width: " << x_max << "|" << width;
    return false;
  }
  if (y_max >= height) {
    AERROR << "y_max is larger than image height: " << y_max << "|" << height;
    return false;
  }
  if (roi.width <= 1 && roi.height <= 1) {
    AERROR << "too small roi range";
    return false;
  }

  size_t total_pix = static_cast<size_t>(roi.width * roi.height);
  std::vector<int> frame_label(total_pix);
  DisjointSet labels(total_pix);
  std::vector<int> root_map;
  root_map.reserve(total_pix);

  int x = 0;
  int y = 0;
  int left_val = 0;
  int up_val = 0;
  int cur_idx = 0;
  int left_idx = 0;
  int up_idx = 0;

  // first loop logic
  for (y = y_min; y <= y_max; y++) {
    int row_start = y * width;
    for (x = x_min; x <= x_max; x++, cur_idx++) {
      left_idx = cur_idx - 1;
      up_idx = cur_idx - width;

      if (x == x_min) {
        left_val = 0;
      } else {
        left_val = src[row_start + x - 1];
      }

      if (y == y_min) {
        up_val = 0;
      } else {
        up_val = src[row_start - width + x];
      }

      if (src[row_start + x] > 0) {
        if (left_val == 0 && up_val == 0) {
          // current pixel is foreground and has no connected neighbors
          frame_label[cur_idx] = labels.Add();
          root_map.push_back(-1);
        } else if (left_val != 0 && up_val == 0) {
          // current pixel is foreground and has left neighbor connected
          frame_label[cur_idx] = frame_label[left_idx];
        } else if (left_val == 0 && up_val != 0) {
          // current pixel is foreground and has up neighbor connect
          frame_label[cur_idx] = frame_label[up_idx];
        } else {
          // current pixel is foreground
          // and is connected to left and up neighbors
          frame_label[cur_idx] = (frame_label[left_idx] > frame_label[up_idx])
                                     ? frame_label[up_idx]
                                     : frame_label[left_idx];
          labels.Unite(frame_label[left_idx], frame_label[up_idx]);
        }
      } else {
        frame_label[cur_idx] = -1;
      }
    }  //  end for x
  }    //  end for y
  AINFO << "subset number = " << labels.Size();

  // second loop logic
  cur_idx = 0;
  int curt_label = 0;
  int cc_count = 0;
  for (y = y_min; y <= y_max; y++) {
    for (x = x_min; x <= x_max; x++, cur_idx++) {
      curt_label = frame_label[cur_idx];
      if (curt_label >= 0 && curt_label < static_cast<int>(labels.Num())) {
        curt_label = labels.Find(curt_label);
        if (curt_label >= static_cast<int>(root_map.size())) {
          AERROR << "curt_label should be smaller than root_map.size() "
                 << curt_label << " vs. " << root_map.size();
          return false;
        }
        if (root_map.at(curt_label) != -1) {
          (*cc)[root_map.at(curt_label)].AddPixel(x, y);
        } else {
          cc->push_back(ConnectedComponent(x, y));
          root_map.at(curt_label) = cc_count++;
        }
      }
    }  // end for x
  }    // end for y
  AINFO << "cc number = " << cc_count;

  return true;
}

bool ImagePoint2Camera(const base::Point2DF& img_point, float pitch_angle,
                       float camera_ground_height,
                       const Eigen::Matrix3f& intrinsic_params_inverse,
                       Eigen::Vector3d* camera_point) {
  Eigen::MatrixXf pt_m(3, 1);
  pt_m << img_point.x, img_point.y, 1;
  const Eigen::MatrixXf& org_camera_point = intrinsic_params_inverse * pt_m;
  //
  float cos_pitch = static_cast<float>(cos(pitch_angle));
  float sin_pitch = static_cast<float>(sin(pitch_angle));
  Eigen::Matrix3f pitch_matrix;
  pitch_matrix << 1, 0, 0, 0, cos_pitch, sin_pitch, 0, -sin_pitch, cos_pitch;
  const Eigen::MatrixXf& rotate_point = pitch_matrix * org_camera_point;
  if (fabs(rotate_point(1, 0)) < lane_eps_value) {
    return false;
  }
  float scale = camera_ground_height / rotate_point(1, 0);
  (*camera_point)(0) = scale * org_camera_point(0, 0);
  (*camera_point)(1) = scale * org_camera_point(1, 0);
  (*camera_point)(2) = scale * org_camera_point(2, 0);
  return true;
}

bool CameraPoint2Image(const Eigen::Vector3d& camera_point,
                       const Eigen::Matrix3f& intrinsic_params,
                       base::Point2DF* img_point) {
  Eigen::Vector3f camera_point3f;
  camera_point3f(0, 0) = static_cast<float>(camera_point(0, 0));
  camera_point3f(1, 0) = static_cast<float>(camera_point(1, 0));
  camera_point3f(2, 0) = static_cast<float>(camera_point(2, 0));
  Eigen::MatrixXf img_point3f = intrinsic_params * camera_point3f;
  if (fabs(img_point3f(2, 0)) < lane_eps_value) {
    return false;
  }
  img_point->x = img_point3f(0, 0) / img_point3f(2, 0);
  img_point->y = img_point3f(1, 0) / img_point3f(2, 0);
  return true;
}
bool ComparePoint2DY(const base::Point2DF& point1,
                     const base::Point2DF& point2) {
  return point1.y < point2.y;
}

bool FindKSmallValue(const float* distance, int dim, int k, int* index) {
  if (dim < k) {
    AWARN << "dim is smaller than k";
    return false;
  }
  if (k <= 0) {
    AWARN << "k is smaller than 0";
    return false;
  }
  std::vector<float> small_value(k);
  //  sort the small value vector
  QuickSort(index, distance, k);
  for (int i = 0; i < k; i++) {
    small_value[i] = distance[index[i]];
  }

  for (int i = k; i < dim; i++) {
    float max_value = small_value[k - 1];
    if (distance[i] >= max_value) {
      continue;
    }
    int locate_index = -1;
    if (distance[i] < small_value[0]) {
      locate_index = 0;
    } else {
      for (int j = 0; j < k - 1; j++) {
        if (distance[i] >= small_value[j] &&
            distance[i] <= small_value[j + 1]) {
          locate_index = j + 1;
          break;
        }  //  if
      }    //  for
    }      //  else
    if (locate_index == -1) {
      return false;
    }
    for (int j = k - 2; j >= locate_index; j--) {
      small_value[j + 1] = small_value[j];
      index[j + 1] = index[j];
    }
    small_value[locate_index] = distance[i];
    index[locate_index] = i;
  }
  return true;
}

bool FindKLargeValue(const float* distance, int dim, int k, int* index) {
  if (dim < k) {
    AWARN << "dim is smaller than k";
    return false;
  }
  if (k <= 0) {
    AWARN << "k is smaller than 0";
    return false;
  }
  std::vector<float> large_value(k);
  std::vector<int> large_index(k);
  //  sort the large value vector
  QuickSort(&(large_index[0]), distance, k);
  for (int i = 0; i < k; i++) {
    index[i] = large_index[k - 1 - i];
    large_value[i] = distance[index[i]];
  }

  for (int i = k; i < dim; i++) {
    float min_value = large_value[k - 1];
    if (distance[i] <= min_value) {
      continue;
    }
    int locate_index = -1;
    if (distance[i] > large_value[0]) {
      locate_index = 0;
    } else {
      for (int j = 0; j < k - 1; j++) {
        if (distance[i] <= large_value[j] &&
            distance[i] >= large_value[j + 1]) {
          locate_index = j + 1;
          break;
        }  //  if
      }    //  for
    }      //  else
    if (locate_index == -1) {
      return false;
    }
    for (int j = k - 2; j >= locate_index; j--) {
      large_value[j + 1] = large_value[j];
      index[j + 1] = index[j];
    }
    large_value[locate_index] = distance[i];
    index[locate_index] = i;
  }
  return true;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
