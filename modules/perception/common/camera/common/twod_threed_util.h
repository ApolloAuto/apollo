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
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/i_lib/core/i_constant.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_line.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace camera {

/*auxiliary data structure*/
template <typename T>
struct LineSegment2D {
  T pt_start[2] = {0};  // left
  T pt_end[2] = {0};    // right

  LineSegment2D(const T &x_start, const T &y_start, const T &x_end,
                const T &y_end) {
    pt_start[0] = x_start;
    pt_start[1] = y_start;
    pt_end[0] = x_end;
    pt_end[1] = y_end;
  }
  LineSegment2D(const LineSegment2D &ls) {
    this->pt_start[0] = ls.pt_start[0];
    this->pt_start[1] = ls.pt_start[1];
    this->pt_end[0] = ls.pt_end[0];
    this->pt_end[1] = ls.pt_end[1];
  }
  void operator=(const LineSegment2D &ls) {
    this->pt_start[0] = ls.pt_start[0];
    this->pt_start[1] = ls.pt_start[1];
    this->pt_end[0] = ls.pt_end[0];
    this->pt_end[1] = ls.pt_end[1];
  }
};
/*end of auxiliary data structure*/

/*angle util*/
template <typename T>
T CalAngleDiff(const T a1, const T a2) {
  T diff1 = a1 - a2;
  diff1 = diff1 < 0 ? -diff1 : diff1;
  T diff2 = a1 - a2 + algorithm::Constant<T>::PI() * 2;
  diff2 = diff2 < 0 ? -diff2 : diff2;
  T diff3 = a1 - a2 - algorithm::Constant<T>::PI() * 2;
  diff3 = diff3 < 0 ? -diff3 : diff3;
  return std::min<T>(std::min<T>(diff1, diff2), diff3);
}

template <typename T>
inline T GetSharpAngle(const T &a, const T &b) {
  /*a and b are in [-pi, pi)*/
  const T pi = algorithm::Constant<T>::PI();
  T angle = std::abs(a - b);
  if (angle > pi) {
    angle -= pi;
  }
  angle = std::min(angle, pi - angle);
  return angle;
}
/*end of angle util*/

/*other util*/
template <typename T>
T GetJaccardIndex(const T *bbox_ref, const T &x_min, const T &y_min,
                  const T &x_max, const T &y_max) {
  T overlap_x = std::min(bbox_ref[2], x_max) - std::max(bbox_ref[0], x_min);
  T overlap_y = std::min(bbox_ref[3], y_max) - std::max(bbox_ref[1], y_min);
  if (overlap_y <= (T)0 || overlap_x <= (T)0) {
    return (T)0;
  }
  T inter_area = overlap_x * overlap_y;
  T union_area = (y_max - y_min) * (x_max - x_min) +
                 (bbox_ref[3] - bbox_ref[1]) * (bbox_ref[2] - bbox_ref[0]) -
                 inter_area;
  return inter_area * algorithm::IRec(union_area);
}

template <typename T>
void GenRotMatrix(const T &ry, T *rot) {
  rot[0] = static_cast<T>(cos(ry));
  rot[2] = static_cast<T>(sin(ry));
  rot[4] = static_cast<T>(1.0f);
  rot[6] = static_cast<T>(-sin(ry));
  rot[8] = static_cast<T>(cos(ry));
  rot[1] = rot[3] = rot[5] = rot[7] = static_cast<T>(0);
}

template <typename T>
void GenCorners(T h, T w, T l, T *x_cor, T *y_cor, T *z_cor) {
  T half_w = static_cast<T>(w * 0.5f);
  T half_l = static_cast<T>(l * 0.5f);
  y_cor[0] = y_cor[1] = y_cor[2] = y_cor[3] = (T)0;
  y_cor[4] = y_cor[5] = y_cor[6] = y_cor[7] = -h;
  x_cor[0] = x_cor[4] = half_l;
  z_cor[0] = z_cor[4] = half_w;
  x_cor[1] = x_cor[5] = half_l;
  z_cor[1] = z_cor[5] = -half_w;
  x_cor[2] = x_cor[6] = -half_l;
  z_cor[2] = z_cor[6] = -half_w;
  x_cor[3] = x_cor[7] = -half_l;
  z_cor[3] = z_cor[7] = half_w;
}

template <typename T>
bool Occlude(const T *bbox1, const T &h1, const T *bbox2, const T &h2) {
  T overlap_x = std::min(bbox1[2], bbox2[2]) - std::max(bbox1[0], bbox2[0]);
  T overlap_y = std::min(bbox1[3], bbox2[3]) - std::max(bbox1[1], bbox2[1]);
  if (overlap_y <= (T)0 || overlap_x <= (T)0) {
    return false;
  }
  const T HEIGHT_BIG_MOT = static_cast<T>(3.0f);
  const T OCC_RATIO = (T)0.1f;
  T bh1 = bbox1[3] - bbox1[1];
  T bh2 = bbox2[3] - bbox2[1];
  T inter_area = overlap_x * overlap_y;
  T area = bh2 * (bbox2[2] - bbox2[0]);
  T occ_ratio = inter_area * algorithm::IRec(area);
  T thres_occ_ratio = h2 < HEIGHT_BIG_MOT ? 2 * OCC_RATIO : OCC_RATIO;
  bool occ = occ_ratio > thres_occ_ratio &&
             h1 * algorithm::IRec(bh1) < h2 * algorithm::IRec(bh2);
  return occ;
}

template <typename T>
void GetBboxFromPts(const T *pts, const int &n, T *bbox) {
  bbox[0] = bbox[2] = pts[0];
  bbox[1] = bbox[3] = pts[1];
  int i2 = 2;
  for (int i = 1; i < n; ++i) {
    T x = pts[i2];
    T y = pts[i2 + 1];
    bbox[0] = std::min(bbox[0], x);
    bbox[1] = std::min(bbox[1], y);
    bbox[2] = std::max(bbox[2], x);
    bbox[3] = std::max(bbox[3], y);
    i2 += 2;
  }
}

template <typename T>
int GetMinIndexVec(const T *vec, const int &n) {
  CHECK_GT(n, 0);
  int index = 0;
  T min_val = vec[0];
  for (int i = 1; i < n; ++i) {
    if (vec[i] < min_val) {
      min_val = vec[i];
      index = i;
    }
  }
  return index;
}

template <typename T>
T GetScoreViaRotDimensionCenter(const T *k_mat, int width, int height,
                                const T *bbox, const T *rot, const T *hwl,
                                const T *location, const bool &check_truncation,
                                T *bbox_res = nullptr, T *bbox_near = nullptr) {
  T h = hwl[0];
  T w = hwl[1];
  T l = hwl[2];
  T x_corners[8] = {0};
  T y_corners[8] = {0};
  T z_corners[8] = {0};
  GenCorners(h, w, l, x_corners, y_corners, z_corners);

  T x_min = std::numeric_limits<float>::max();
  T x_max = -std::numeric_limits<float>::max();
  T y_min = std::numeric_limits<float>::max();
  T y_max = -std::numeric_limits<float>::max();
  T x_proj[3];
  T x_box[3];
  T pts_proj[16];
  for (int i = 0; i < 8; ++i) {
    x_box[0] = x_corners[i];
    x_box[1] = y_corners[i];
    x_box[2] = z_corners[i];
    algorithm::IProjectThroughKRt(k_mat, rot, location, x_box, x_proj);
    x_proj[0] *= algorithm::IRec(x_proj[2]);
    x_proj[1] *= algorithm::IRec(x_proj[2]);

    if (bbox_near != nullptr) {
      int i2 = i * 2;
      pts_proj[i2] = x_proj[0];
      pts_proj[i2 + 1] = x_proj[1];
    }
    x_min = std::min(x_min, x_proj[0]);
    x_max = std::max(x_max, x_proj[0]);
    y_min = std::min(y_min, x_proj[1]);
    y_max = std::max(y_max, x_proj[1]);
    if (check_truncation) {
      x_min = std::min(std::max(x_min, (T)0), (T)(width - 1));
      x_max = std::min(std::max(x_max, (T)0), (T)(width - 1));
      y_min = std::min(std::max(y_min, (T)0), (T)(height - 1));
      y_max = std::min(std::max(y_max, (T)0), (T)(height - 1));
    }
  }
  if (bbox_res != nullptr) {
    bbox_res[0] = x_min;
    bbox_res[1] = y_min;
    bbox_res[2] = x_max;
    bbox_res[3] = y_max;
  }
  if (bbox_near != nullptr) {
    T bbox_front[4] = {0};
    T bbox_rear[4] = {0};
    std::swap(pts_proj[4], pts_proj[8]);
    std::swap(pts_proj[5], pts_proj[9]);
    std::swap(pts_proj[6], pts_proj[10]);
    std::swap(pts_proj[7], pts_proj[11]);
    GetBboxFromPts(pts_proj, 4, bbox_front);
    GetBboxFromPts(pts_proj + 8, 4, bbox_rear);
    if (bbox_rear[3] - bbox_rear[1] > bbox_front[3] - bbox_front[1]) {
      memcpy(bbox_near, bbox_rear, sizeof(T) * 4);
    } else {
      memcpy(bbox_near, bbox_front, sizeof(T) * 4);
    }
  }
  return GetJaccardIndex(bbox, x_min, y_min, x_max, y_max);
}

template <typename T>
bool CheckXY(const T &x, const T &y, int width, int height) {
  T max_x = static_cast<T>(width - 1);
  T max_y = static_cast<T>(height - 1);
  return x <= max_x && x >= (T)0 && y <= max_y && y >= (T)0;
}

template <typename T>
void UpdateOffsetZ(T x_start, T z_start, T x_end, T z_end,
                   const std::pair<T, T> &range, T *z_offset) {
  ACHECK(range.first < range.second);
  if (x_start > x_end) {
    std::swap(x_start, x_end);
    std::swap(z_start, z_end);
  }

  T x_check_l = std::max(x_start, range.first);
  T x_check_r = std::min(x_end, range.second);
  T overlap_x = x_check_r - x_check_l;
  if (overlap_x < 1e-6) {
    return;
  }

  T dz_divide_dx = (z_end - z_start) * algorithm::IRec(x_end - x_start);
  T z_check_l = z_start + (x_check_l - x_start) * dz_divide_dx;
  T z_check_r = z_start + (x_check_r - x_start) * dz_divide_dx;
  T z_nearest = std::min(z_check_l, z_check_r);
  if (z_nearest < *z_offset) {
    *z_offset = z_nearest;
  }
}

template <typename T>
void GetDxDzForCenterFromGroundLineSeg(const LineSegment2D<T> &ls,
                                       const T *plane, const T *pts_c,
                                       const T *k_mat, int width, int height,
                                       T ratio_x_over_z, T *dx_dz,
                                       bool check_lowerbound = true) {
  const T Z_RESOLUTION = static_cast<T>(1e-1);
  const T Z_UNSTABLE_RATIO = static_cast<T>(0.3f);

  dx_dz[0] = dx_dz[1] = (T)0;
  ACHECK(ls.pt_start[0] < ls.pt_end[0]);
  if (!CheckXY(ls.pt_start[0], ls.pt_start[1], width, height) ||
      !CheckXY(ls.pt_end[0], ls.pt_end[1], width, height)) {
    return;
  }
  T pt_of_line_seg_l[3] = {0};
  T pt_of_line_seg_r[3] = {0};
  bool is_front_l = algorithm::IBackprojectPlaneIntersectionCanonical(
      ls.pt_start, k_mat, plane, pt_of_line_seg_l);
  bool is_front_r = algorithm::IBackprojectPlaneIntersectionCanonical(
      ls.pt_end, k_mat, plane, pt_of_line_seg_r);
  if (!is_front_l || !is_front_r) {
    return;
  }
  ADEBUG << "l&r on-ground points: " << pt_of_line_seg_l[0] << ", "
         << pt_of_line_seg_l[1] << ", " << pt_of_line_seg_l[2] << " | "
         << pt_of_line_seg_r[0] << ", " << pt_of_line_seg_r[1] << ", "
         << pt_of_line_seg_r[2];

  // get transform
  T rot[9] = {0};
  T t[3] = {0};
  T pos[3] = {pt_of_line_seg_l[0], pt_of_line_seg_l[1], pt_of_line_seg_l[2]};
  T v[3] = {pt_of_line_seg_r[0] - pt_of_line_seg_l[0],
            pt_of_line_seg_r[1] - pt_of_line_seg_l[1],
            pt_of_line_seg_r[2] - pt_of_line_seg_l[2]};
  T ry = static_cast<T>(-atan2(v[2], v[0]));
  GenRotMatrix(ry, rot);
  algorithm::ITranspose3x3(rot);
  algorithm::IScale3(pos, (T)-1);
  algorithm::IMultAx3x3(rot, pos, t);
  ADEBUG << "ry: " << ry;

  T l[3] = {0};
  T pt1[3] = {0};
  T pt2[3] = {ratio_x_over_z, (T)0, (T)1};
  T pt1_local[3] = {0};
  T pt2_local[3] = {0};

  // transform to local birdview coordinates
  std::pair<T, T> range;
  range.first = 0;
  range.second = algorithm::ISqrt(
      algorithm::ISqr(v[0]) + algorithm::ISqr(v[2]));
  T pts_local[12] = {0};
  algorithm::IProjectThroughExtrinsic(rot, t, pts_c, pts_local);
  algorithm::IProjectThroughExtrinsic(rot, t, pts_c + 3, pts_local + 3);
  algorithm::IProjectThroughExtrinsic(rot, t, pts_c + 6, pts_local + 6);
  algorithm::IProjectThroughExtrinsic(rot, t, pts_c + 9, pts_local + 9);

  algorithm::IProjectThroughExtrinsic(rot, t, pt1, pt1_local);
  algorithm::IProjectThroughExtrinsic(rot, t, pt2, pt2_local);
  T x[2] = {pt1_local[0], pt1_local[2]};
  T xp[2] = {pt2_local[0], pt2_local[2]};
  algorithm::ILineFit2d(x, xp, l);

  T zs[4] = {pts_local[2], pts_local[5], pts_local[8], pts_local[11]};
  T z_offset = static_cast<T>(0);
  bool all_positive = zs[0] > 0.0f && zs[1] > 0.f && zs[2] > 0.f && zs[3] > 0.f;
  if (all_positive) {
    z_offset = std::min(zs[0], std::min(zs[1], std::min(zs[2], zs[3])));
  } else {
    z_offset = std::max(zs[0], std::max(zs[1], std::max(zs[2], zs[3])));
    T xs[4] = {pts_local[0], pts_local[3], pts_local[6], pts_local[9]};
    // 1 -> 2
    UpdateOffsetZ(xs[0], zs[0], xs[1], zs[1], range, &z_offset);
    // 2 -> 3
    UpdateOffsetZ(xs[1], zs[1], xs[2], zs[2], range, &z_offset);
    // 3 -> 4
    UpdateOffsetZ(xs[2], zs[2], xs[3], zs[3], range, &z_offset);
    // 4 -> 1
    UpdateOffsetZ(xs[3], zs[3], xs[0], zs[0], range, &z_offset);
  }
  if (z_offset < Z_RESOLUTION && z_offset > -Z_RESOLUTION) {
    return;
  }

  // judge & convert back to camera coordinates
  if (z_offset > static_cast<T>(0.0f) && check_lowerbound) {
    return;
  }

  T center_c[3] = {0};
  center_c[0] = (pts_c[0] + pts_c[3] + pts_c[6] + pts_c[9]) / 4;
  center_c[2] = (pts_c[2] + pts_c[5] + pts_c[8] + pts_c[11]) / 4;
  T z_avg = center_c[2];

  T dz_local = -z_offset;
  T dx_local =
      -l[1] * dz_local * algorithm::IRec(l[0]); /*enforce to be along the ray*/

  T center_local[3] = {0};
  algorithm::IProjectThroughExtrinsic(rot, t, center_c, center_local);
  center_local[0] += dx_local;
  center_local[1] = 0;
  center_local[2] += dz_local;
  T center_local_to_c[3] = {0};
  algorithm::ISub3(t, center_local);
  algorithm::IMultAtx3x3(rot, center_local, center_local_to_c);
  dx_dz[0] = center_local_to_c[0] - center_c[0];
  dx_dz[1] = center_local_to_c[2] - center_c[2];

  T dz_max = z_avg * Z_UNSTABLE_RATIO;
  if (z_offset > static_cast<T>(0.0f) && std::abs(dx_dz[1]) > dz_max) {
    dx_dz[0] = dx_dz[1] = 0;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
