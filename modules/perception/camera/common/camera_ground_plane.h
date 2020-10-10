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

#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/geometry/i_line.h"

namespace apollo {
namespace perception {
namespace camera {

// ground conversion utils
void ConvertGround3ToGround4(const float &baseline,
                             const std::vector<float> &k_mat,
                             const std::vector<float> &ground3,
                             std::vector<float> *ground4);

bool ConvertGround4ToGround3(const float &baseline,
                             const std::vector<float> &k_mat,
                             const std::vector<float> &ground4,
                             std::vector<float> *ground3);

void GetGroundPlanePitchHeight(const float &baseline,
                               const std::vector<float> &k_mat,
                               const std::vector<float> &ground3, float *pitch,
                               float *cam_height);

void GetGround3FromPitchHeight(const std::vector<float> &k_mat,
                               const float &baseline, const float &pitch,
                               const float &cam_height,
                               std::vector<float> *ground3);

// @brief: a ground plane tracker using pitch + camera height representation.
// @param [in]: per-frame p(pitch)h(height) and inlier ratio
// @param [in/out]: smoothed plane
class GroundPlaneTracker {
 public:
  explicit GroundPlaneTracker(int track_length);
  ~GroundPlaneTracker() {}

  void Push(const std::vector<float> &ph, const float &inlier_ratio);

  void GetGround(float *pitch, float *cam_height);

  void Restart();

  int GetCurTrackLength() {
    int length = static_cast<int>(pitch_height_inlier_tracks_.size());
    CHECK_GE(length, 0);
    return length;
  }

 private:
  int head_ = 0;
  std::vector<float> pitch_height_inlier_tracks_;  // pitch, height, inlier ...
  std::vector<float> const_weight_temporal_;
  std::vector<float> weight_;
};

// params for ground detector
struct CameraGroundPlaneParams {
  CameraGroundPlaneParams() { SetDefault(); }
  void SetDefault();

  int min_nr_samples;
  int nr_frames_track;
  float max_tilt_angle;
  float max_camera_ground_height;
  float min_inlier_ratio;
  float thres_inlier_plane_fitting;

  void operator=(const CameraGroundPlaneParams &params) {
    this->min_nr_samples = params.min_nr_samples;
    this->nr_frames_track = params.nr_frames_track;
    this->max_tilt_angle = params.max_tilt_angle;
    this->min_inlier_ratio = params.min_inlier_ratio;
    this->thres_inlier_plane_fitting = params.thres_inlier_plane_fitting;
  }
};

// @brief: a ground plane detector using disparity.
// @param [in]: disparity samples (row-disp pairs)
// @param [in/out]: ground plane (a * y + b * disp + c = 0)
class CameraGroundPlaneDetector {
 public:
  CameraGroundPlaneDetector() {
    ground_plane_tracker_ = new GroundPlaneTracker(params_.nr_frames_track);
  }
  ~CameraGroundPlaneDetector() {
    delete ground_plane_tracker_;
    ground_plane_tracker_ = nullptr;
  }

  void Init(const std::vector<float> &k_mat, int width, int height,
            float baseline, int max_nr_samples = 1080) {
    CHECK_EQ(k_mat.size(), 9);
    memcpy(k_mat_, k_mat.data(), sizeof(float) * 9);
    width_ = width;
    height_ = height;
    baseline_ = baseline;
    ss_flt_.resize(max_nr_samples * 2);
    ss_int_.resize(max_nr_samples * 2);
  }

  int get_min_nr_samples() { return params_.min_nr_samples; }

  // main interface: fit the plane from v-d samples
  // input a backup (default) pitch + camera_height
  // can be assigned by {a, b, c, d}: a * X + b * Y + c * Z + d = 0 (cam coor)
  bool DetetGround(float pitch, float camera_height,
                   float *vd, /*samples: v0, d0 ... vj, dj*/
                   int count_vd, const std::vector<float> &plane = {});

  bool GetGroundModel(float *l) const {
    l[0] = l_[0];
    l[1] = l_[1];
    l[2] = l_[2];
    return ground_is_valid_;
  }

 private:
  void FillGroundModel(const std::vector<float> &ground3) {
    CHECK_EQ(ground3.size(), 3);
    l_[0] = ground3[0];
    l_[1] = ground3[1];
    l_[2] = ground3[2];
  }

  bool DetectGroundFromSamples(float *vd, int count_vd, float *inlier_ratio);

 private:
  CameraGroundPlaneParams params_;
  int width_ = 0;
  int height_ = 0;
  float k_mat_[9] = {0};
  float baseline_ = 0.0f;  // if disp is reversed-z, baseline = 1 / focal
  float l_[3] = {0};       // a * y + b * disp + c = 0

  // scratch space
  std::vector<int> ss_int_ = {};
  std::vector<float> ss_flt_ = {};

  bool ground_is_valid_ = false;  // status of ground estimation

  GroundPlaneTracker *ground_plane_tracker_ = nullptr;
};

/*
 fitting utils
*/
template <typename T>
void GroundHypoGenFunc(const T *v, const T *d, T *p) {
  // disp = p0 * y + p1 -> l = {p0, -1, p1}
  T x[2] = {v[0], d[0]};
  T xp[2] = {v[1], d[1]};
  T l[3] = {0};
  common::ILineFit2d(x, xp, l);
  p[0] = -l[0] * common::IRec(l[1]);
  p[1] = -l[2] * common::IRec(l[1]);
}

template <typename T>
void GroundFittingCostFunc(const T *p, const T *v, const T *d, int n,
                           int *nr_inlier,  // NOLINT compatible for i-lib
                           int *inliers,
                           T *cost,  // NOLINT
                           T error_tol) {
  *cost = static_cast<T>(0.0f);
  *nr_inlier = 0;
  const T *refx = v;
  const T *refp = d;
  for (int i = 0; i < n; ++i) {
    T d_proj = refx[0] * p[0] + p[1];
    T proj_err = static_cast<T>(fabs(d_proj - refp[0]));
    if (proj_err < error_tol) {
      inliers[(*nr_inlier)++] = i;
      *cost += proj_err;
    }
    ++refx;
    ++refp;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
