/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

// brief: image space to ego-car vehicle space projection

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_PROJECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_PROJECTOR_H_

#include <algorithm>
#include <limits>
#include <vector>

#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"

namespace apollo {
namespace perception {

template <typename T>
class Projector {
 public:
  Projector();

  bool Init(const cv::Rect &roi, const T &max_distance = 200.0,
            bool visualize = false);

  bool UvToXy(const T &u, const T &v, Eigen::Matrix<T, 2, 1> *p) const;

  bool UvToXy(const int &u, const int &v, Eigen::Matrix<T, 2, 1> *p) const {
    return UvToXy(static_cast<T>(u), static_cast<T>(v), p);
  }

  bool is_init() const { return is_init_; }

  bool IsValidUv(const int &x, const int &y) const {
    return IsValidUv(static_cast<T>(x), static_cast<T>(y));
  }

  bool IsValidUv(const T &x, const T &y) const;

  bool IsValidXy(const T &x, const T &y) const {
    return x >= xy_xmin_ && x <= xy_xmax_ && y >= xy_ymin_ && y <= xy_ymax_;
  }

  bool is_vis() const { return is_vis_; }
  T x_step() const { return x_step_; }
  T y_step() const { return y_step_; }
  T xy_image_xmin() const { return xy_image_xmin_; }
  T xy_image_xmax() const { return xy_image_xmax_; }
  T xy_image_ymin() const { return xy_image_ymin_; }
  T xy_image_ymax() const { return xy_image_ymax_; }

  bool IsValidXyInXyImage(const T &x, const T &y) const {
    return x >= xy_image_xmin_ && x <= xy_image_xmax_ && y >= xy_image_ymin_ &&
           y <= xy_image_ymax_;
  }

  int xy_image_cols() const { return xy_image_cols_; }

  int xy_image_rows() const { return xy_image_rows_; }

  bool XyToXyImagePoint(const T &x, const T &y, cv::Point *p) const;
  bool XyToXyImagePoint(const Eigen::Matrix<T, 2, 1> &pos, cv::Point *p) const {
    return XyToXyImagePoint(pos.x(), pos.y(), p);
  }

  bool UvToXyImagePoint(const T &u, const T &v, cv::Point *p) const;

  bool UvToXyImagePoint(const int &u, const int &v, cv::Point *p) const {
    return UvToXyImagePoint(static_cast<T>(u), static_cast<T>(v), p);
  }

 protected:
  bool Project(const T &u, const T &v, Eigen::Matrix<T, 2, 1> *xy_point);

 private:
  Eigen::Matrix<T, 3, 3> trans_mat_;

  // for visualizing IPM image
  std::vector<Eigen::Matrix<T, 2, 1>> xy_grid_;
  std::vector<cv::Point> uv_2_xy_image_;
  T x_step_;
  T y_step_;
  T xy_image_xmin_;
  T xy_image_xmax_;
  int xy_image_cols_;
  T xy_image_ymin_;
  T xy_image_ymax_;
  int xy_image_rows_;

  // ROI limits in IPM space
  T xy_xmin_;
  T xy_ymin_;
  T xy_xmax_;
  T xy_ymax_;

  // ROI bounds in image space
  T uv_xmin_;
  T uv_ymin_;
  T uv_xmax_;
  T uv_ymax_;

  T uv_roi_x_step_;   // the horizontal direction sampling step of ROI in image
                      // space
  T uv_roi_y_step_;   // the vertical direction sampling step of ROI in image
                      // space
  int uv_roi_cols_;   // the column number of ROI in image space
  int uv_roi_rows_;   // the row number of ROI in image space
  int uv_roi_count_;  // the count of ROI sample points

  std::vector<bool> valid_xy_;
  std::vector<Eigen::Matrix<T, 2, 1>> uv_2_xy_;

  bool is_init_;
  bool is_vis_;
};

template <typename T>
Projector<T>::Projector() {
  xy_xmin_ = std::numeric_limits<T>::max();
  xy_ymin_ = std::numeric_limits<T>::max();
  xy_xmax_ = -std::numeric_limits<T>::max();
  xy_ymax_ = -std::numeric_limits<T>::max();

  uv_xmin_ = static_cast<T>(0);
  uv_ymin_ = static_cast<T>(0);
  uv_xmax_ = static_cast<T>(0);
  uv_ymax_ = static_cast<T>(0);

  uv_roi_x_step_ = static_cast<T>(1);
  uv_roi_y_step_ = static_cast<T>(1);

  uv_roi_cols_ = static_cast<int>((uv_xmax_ - uv_xmin_) / uv_roi_x_step_) + 1;
  uv_roi_rows_ = static_cast<int>((uv_ymax_ - uv_ymin_) / uv_roi_y_step_) + 1;
  uv_roi_count_ = uv_roi_rows_ * uv_roi_cols_;

  is_vis_ = false;
  x_step_ = static_cast<T>(0.1);
  y_step_ = static_cast<T>(0.1);

  is_init_ = false;
}

template <typename T>
bool Projector<T>::Init(const cv::Rect &roi, const T &max_distance,
                        bool visualize) {
  AINFO << "Initialize projector ...";

  // read transformation matrix from calibration config manager
  CalibrationConfigManager *calibration_config_manager =
      Singleton<CalibrationConfigManager>::get();

  const CameraCalibrationPtr camera_calibration =
      calibration_config_manager->get_camera_calibration();

  trans_mat_ = camera_calibration->get_camera2car_homography_mat().cast<T>();

  // set ROI
  uv_xmin_ = static_cast<T>(roi.x);
  if (uv_xmin_ < 0) {
    AERROR << "invalid ROI x min: " << roi.x;
    return false;
  }

  uv_ymin_ = static_cast<T>(roi.y);
  if (uv_ymin_ < 0) {
    AERROR << "invalid ROI y min: " << roi.y;
    return false;
  }

  uv_xmax_ = static_cast<T>(roi.x + roi.width - 1);
  if (uv_xmax_ < 0) {
    AERROR << "invalid ROI x max: " << roi.x + roi.width - 1;
    return false;
  }

  uv_ymax_ = static_cast<T>(roi.y + roi.height - 1);
  if (uv_ymax_ < 0) {
    AERROR << "invalid ROI y max: " << roi.y + roi.height - 1;
    return false;
  }

  AINFO << "ROI in image space: "
        << "x_min=" << uv_xmin_ << ", "
        << "x_max=" << uv_xmax_ << ", "
        << "y_min=" << uv_ymin_ << ", "
        << "y_max=" << uv_ymax_ << ".";
  AINFO << "ROI width = " << roi.width << ", height = " << roi.height;

  uv_roi_cols_ = static_cast<int>((uv_xmax_ - uv_xmin_) / uv_roi_x_step_) + 1;
  uv_roi_rows_ = static_cast<int>((uv_ymax_ - uv_ymin_) / uv_roi_y_step_) + 1;
  uv_roi_count_ = uv_roi_rows_ * uv_roi_cols_;
  AINFO << "sampling step_x (u) = " << uv_roi_x_step_ << ", "
        << "sampling step_y (v) = " << uv_roi_y_step_;
  AINFO << "ROI #columns = " << uv_roi_cols_ << ", "
        << "#rows = " << uv_roi_rows_;

  // do point-wise transform of ROI from image space to ground space
  valid_xy_.assign(uv_roi_count_, true);
  uv_2_xy_.resize(uv_roi_count_);
  int count_fail_points = 0;
  int count_too_far_points = 0;
  T v = uv_ymin_;
  for (int i = 0; i < uv_roi_rows_; ++i) {
    T u = uv_xmin_;
    for (int j = 0; j < uv_roi_cols_; ++j) {
      // point coordinate in image space (x, y)
      int id = i * uv_roi_cols_ + j;

      // transform point from image space to ego-car ground space
      if (!Project(u, v, &(uv_2_xy_[id]))) {
        valid_xy_[id] = false;
        ++count_fail_points;
        continue;
      }

      // ignore points which is too far away from origin
      if (uv_2_xy_[id].x() > max_distance || uv_2_xy_[id].y() > max_distance) {
        valid_xy_[id] = false;
        ++count_too_far_points;
        continue;
      }

      //
      if (uv_2_xy_[id].x() < 0) {
        valid_xy_[id] = false;
        ++count_too_far_points;
        continue;
      }

      // update xy limits
      xy_xmin_ = std::min(xy_xmin_, uv_2_xy_[id].x());
      xy_ymin_ = std::min(xy_ymin_, uv_2_xy_[id].y());
      xy_xmax_ = std::max(xy_xmax_, uv_2_xy_[id].x());
      xy_ymax_ = std::max(xy_ymax_, uv_2_xy_[id].y());

      u += uv_roi_x_step_;
    }
    v += uv_roi_y_step_;
  }
  AINFO << "#failed points = " << count_fail_points << " (" << uv_roi_count_
        << ").";
  AINFO << "#too far points = " << count_too_far_points << " (" << uv_roi_count_
        << ").";
  AINFO << " xy limits: "
        << "x_min=" << xy_xmin_ << ", "
        << "x_max=" << xy_xmax_ << ", "
        << "y_min=" << xy_ymin_ << ", "
        << "y_max=" << xy_ymax_;

  is_vis_ = visualize;
  if (is_vis_) {
    // build a 2D grid in ground plane and
    // find the index mapping for each valid point of image space
    xy_image_xmin_ = std::min(std::floor(xy_xmin_), static_cast<T>(0));
    xy_image_xmin_ = std::max(xy_image_xmin_, -max_distance);
    xy_image_xmax_ = std::max(std::ceil(xy_xmax_), static_cast<T>(0));
    xy_image_xmax_ = std::min(xy_image_xmax_, max_distance);
    xy_image_ymin_ = std::min(std::floor(xy_ymin_), static_cast<T>(0));
    xy_image_ymin_ = std::max(xy_image_ymin_, -max_distance);
    xy_image_ymax_ = std::max(std::ceil(xy_ymax_), static_cast<T>(0));
    xy_image_ymax_ = std::min(xy_image_ymax_, max_distance);

    x_step_ = max_distance / static_cast<T>(1000);
    y_step_ = max_distance / static_cast<T>(1000);

    xy_image_cols_ = static_cast<int>(std::ceil(
                         (xy_image_xmax_ - xy_image_xmin_) / x_step_)) +
                     1;
    xy_image_rows_ = static_cast<int>(std::ceil(
                         (xy_image_ymax_ - xy_image_ymin_) / y_step_)) +
                     1;
    AINFO << "xy_image_xmin = " << xy_image_xmin_
          << ", xy_image_xmax = " << xy_image_xmax_
          << ", xy_image_ymin = " << xy_image_ymin_
          << ", xy_image_ymax = " << xy_image_ymax_;
    AINFO << "xy_image: step_x=" << x_step_ << ", "
          << "step_y=" << y_step_;
    AINFO << "xy_image: #cols = " << xy_image_cols_
          << ", #rows = " << xy_image_rows_;

    xy_grid_.resize(xy_image_rows_ * xy_image_cols_);
    T y = xy_image_ymax_;
    for (int i = 0; i < xy_image_rows_; i++) {
      T x = xy_image_xmin_;
      for (int j = 0; j < xy_image_cols_; j++) {
        xy_grid_[i * xy_image_cols_ + j].x() = x;
        xy_grid_[i * xy_image_cols_ + j].y() = y;
        x += x_step_;
      }
      y = y - y_step_;
    }

    uv_2_xy_image_.resize(uv_roi_count_);
    T v = uv_ymin_;
    for (int i = 0; i < uv_roi_rows_; ++i) {
      T u = uv_xmin_;
      for (int j = 0; j < uv_roi_cols_; ++j) {
        int id = i * uv_roi_cols_ + j;
        if (valid_xy_[id]) {
          XyToXyImagePoint(static_cast<T>(uv_2_xy_[id].x()),
                           static_cast<T>(uv_2_xy_[id].y()),
                           &uv_2_xy_image_[id]);
        } else {
          uv_2_xy_image_[id].x = -1;
          uv_2_xy_image_[id].y = -1;
        }
        u += uv_roi_x_step_;
      }
      v += uv_roi_y_step_;
    }
  }

  is_init_ = true;

  AINFO << "succ. to initialize projector.";
  return true;
}

template <typename T>
bool Projector<T>::XyToXyImagePoint(const T &x, const T &y,
                                    cv::Point *p) const {
  if (!IsValidXyInXyImage(x, y)) {
    return false;
  }

  int j = static_cast<int>(std::round((x - xy_image_xmin_) / x_step_));
  if (j < 0) {
    return false;
  }
  if (j >= xy_image_cols_) {
    return false;
  }

  int i = static_cast<int>(std::round((y - xy_image_ymin_) / y_step_));
  if (i < 0) {
    return false;
  }
  if (i >= xy_image_rows_) {
    return false;
  }

  i = (xy_image_rows_ - 1) - i;

  p->x = j;
  p->y = i;

  return true;
}

template <typename T>
bool Projector<T>::IsValidUv(const T &x, const T &y) const {
  if (!(x >= uv_xmin_ && x <= uv_xmax_ && y >= uv_ymin_ && y <= uv_ymax_)) {
    AINFO << "image point "
          << "(" << x << ", " << y << ")"
          << " is not in ROI: "
          << "x_min=" << uv_xmin_ << ", "
          << "x_max=" << uv_xmax_ << ", "
          << "y_min=" << uv_ymin_ << ", "
          << "y_max=" << uv_ymax_ << ". ";
    return false;
  }
  return true;
}

template <typename T>
bool Projector<T>::UvToXy(const T &u, const T &v,
                          Eigen::Matrix<T, 2, 1> *p) const {
  if (p == nullptr) {
    AERROR << "point pointer is null.";
    return false;
  }
  if (!is_init_) {
    AERROR << "projector is not initialized.";
    return false;
  }

  if (!IsValidUv(u, v)) {
    return false;
  }

  int id = static_cast<int>(std::round((v - uv_ymin_) / uv_roi_y_step_)) *
               uv_roi_cols_ +
           static_cast<int>(std::round((u - uv_xmin_) / uv_roi_x_step_));
  if (id < 0 || id >= uv_roi_count_) {
    AERROR << "pixel id is not valid: " << id;
    return false;
  }

  if (!valid_xy_.at(id)) {
    AINFO << "image point "
          << "(" << u << ", " << v << ")"
          << " is not valid for transformation.";
    return false;
  }

  p->x() = uv_2_xy_.at(id).x();
  p->y() = uv_2_xy_.at(id).y();
  return true;
}

template <typename T>
bool Projector<T>::UvToXyImagePoint(const T &u, const T &v,
                                    cv::Point *p) const {
  if (p == nullptr) {
    AERROR << "point pointer is null.";
    return false;
  }
  if (!is_init_) {
    AERROR << "projector is not initialized.";
    return false;
  }

  if (!IsValidUv(u, v)) {
    AINFO << "image point "
          << "(" << u << ", " << v << ")"
          << " is not in ROI.";
    return false;
  }

  int id = static_cast<int>(std::round((v - uv_ymin_) / uv_roi_y_step_)) *
               uv_roi_cols_ +
           static_cast<int>(std::round((u - uv_xmin_) / uv_roi_x_step_));
  if (id < 0 || id >= uv_roi_count_) {
    AERROR << "pixel id is not valid: " << id;
    return false;
  }

  if (!valid_xy_.at(id)) {
    AINFO << "image point "
          << "(" << u << ", " << v << ")"
          << " is not valid for transformation.";
    return false;
  }

  p->x = uv_2_xy_image_.at(id).x;
  p->y = uv_2_xy_image_.at(id).y;
  return true;
}

template <typename T>
bool Projector<T>::Project(const T &u, const T &v,
                           Eigen::Matrix<T, 2, 1> *xy_point) {
  if (xy_point == nullptr) {
    AERROR << "xy_point is a null pointer.";
    return false;
  }

  Eigen::Matrix<T, 3, 1> uv_point(u, v, static_cast<T>(1));
  Eigen::Matrix<T, 3, 1> xy_p = trans_mat_ * uv_point;

  T scale = xy_p(2);
  if (std::abs(scale) < 1e-6) {
    AINFO << "Cannot solve point: scale factor is too small (" << scale << "), "
          << " u=" << u << " v=" << v << ".";
    return false;
  }

  (*xy_point) << xy_p(0) / scale, xy_p(1) / scale;

  if (!std::isfinite((*xy_point)(0)) || std::isnan((*xy_point)(0)) ||
      !std::isfinite((*xy_point)(1)) || std::isnan((*xy_point)(1))) {
    AINFO << "xy point is not valid: "
          << " u=" << u << " v=" << v << ".";
    return false;
  }

  return true;
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_PROJECTOR_H_
