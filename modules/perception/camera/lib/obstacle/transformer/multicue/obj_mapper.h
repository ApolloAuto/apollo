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
#include <map>
#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/common/twod_threed_util.h"

namespace apollo {
namespace perception {
namespace camera {

struct ObjMapperOptions {
  float hwl[3] = {0};
  float bbox[4] = {0};
  float ry = 0.0f;
  bool is_veh = true;
  bool check_dimension = true;
  int type_min_vol_index = 0;
};

// hyper parameters
struct ObjMapperParams {
  ObjMapperParams() { set_default(); }

  void set_default();

  int nr_bins_z;
  int nr_bins_ry;
  int boundary_len;
  int max_nr_iter;

  float small_bbox_height;
  float factor_small;
  float learning_r;
  float reproj_err;
  float rz_ratio;
  float abnormal_h_veh;
  float stable_w;
  float occ_ratio;
  float depth_min;
  float dist_far;
  float eps_mapper;
  float iou_suc;
  float iou_high;
  float angle_resolution_degree;
};

class ObjMapper {
 public:
  ObjMapper() : width_(0), height_(0) {
    memset(k_mat_, 0, sizeof(float) * 9);
    resize_ry_score(params_.nr_bins_ry);
    set_default_variance();
  }

  ~ObjMapper() {}

  void set_default_variance() {
    orientation_variance_(0) = 1.0f;
    orientation_variance_(1) = orientation_variance_(2) = 0.0f;
    position_uncertainty_ << 1, 0, 0, 0, 0, 0, 0, 0, 1;
  }

  void Init(const float *k_mat, int width, int height) {
    memcpy(k_mat_, k_mat, sizeof(float) * 9);
    width_ = width;
    height_ = height;
    object_template_manager_ = ObjectTemplateManager::Instance();
  }

  void resize_ry_score(int size) { ry_score_.resize(size); }

  std::vector<float> get_ry_score() { return ry_score_; }

  std::vector<float> get_ry_score(float ry) {
    FillRyScoreSingleBin(ry);
    return ry_score_;
  }

  Eigen::Vector3d get_orientation_var() { return orientation_variance_; }

  Eigen::Matrix3d get_position_uncertainty() { return position_uncertainty_; }

  // main interface, center is the bottom-face center ("center" for short)
  bool Solve3dBbox(const ObjMapperOptions &options, float center[3],
                   float hwl[3], float *ry);

 private:
  bool Solve3dBboxGivenOneFullBboxDimensionOrientation(const float *bbox,
                                                       const float *hwl,
                                                       float *ry,
                                                       float *center);

  bool SolveCenterFromNearestVerticalEdge(const float *bbox, const float *hwl,
                                          float ry, float *center,
                                          float *center_2d) const;

  void UpdateCenterViaBackProjectZ(const float *bbox, const float *hwl,
                                   const float *center_2d,
                                   float *center) const {
    float z = center[2];
    common::IBackprojectCanonical(center_2d, k_mat_, z, center);
    center[1] += hwl[0] / 2;
  }

  float GetProjectionScore(float ry, const float *bbox, const float *hwl,
                           const float *center, bool check_truncation = false,
                           float *bbox_res = nullptr) const {
    float rot[9] = {0};
    GenRotMatrix(ry, rot);
    float *bbox_near = nullptr;
    float score = GetScoreViaRotDimensionCenter(
        &k_mat_[0], width_, height_, bbox, rot, hwl, center, check_truncation,
        bbox_res, bbox_near);
    return score;
  }

  void PostRefineZ(const float *bbox, const float *hwl, const float *center2d,
                   float ry, float *center, float dz = 0.2f, float rz = 1.0f,
                   bool fix_proj = true) const {
    float z = center[2];
    float score_best = 0.0f;
    float x[2] = {center2d[0], center2d[1]};
    float center_best[3] = {center[0], center[1], center[2]};
    for (float offset = -rz; offset <= rz; offset += dz) {
      float z_test = std::max(z + offset, params_.depth_min);
      float center_test[3] = {center[0], center[1], z_test};
      if (fix_proj) {
        common::IBackprojectCanonical(x, k_mat_, z_test, center_test);
        center_test[1] += hwl[0] / 2;
      }
      float score_test = GetProjectionScore(ry, bbox, hwl, center_test);
      if (score_best < score_test) {
        score_best = score_test;
        memcpy(center_best, center_test, sizeof(float) * 3);
      }
    }
    memcpy(center, center_best, sizeof(float) * 3);
  }

  void PostRefineOrientation(const float *bbox, const float *hwl,
                             const float *center, float *ry);

  void GetCenter(const float *bbox, const float &z_ref, const float &ry,
                 const float *hwl, float *center,
                 float *x /*init outside*/) const;

  void FillRyScoreSingleBin(const float &ry) {
    int nr_bins_ry = static_cast<int>(ry_score_.size());
    memset(ry_score_.data(), 0, sizeof(float) * nr_bins_ry);
    const float PI = common::Constant<float>::PI();
    int index = static_cast<int>(
        std::floor((ry + PI) * static_cast<float>(nr_bins_ry) / (2.0f * PI)));
    ry_score_[index % nr_bins_ry] = 1.0f;
  }

 private:
  // canonical camera: p_matrix = k_matrix [I 0]
  float k_mat_[9] = {0};
  int width_ = 0;
  int height_ = 0;
  std::vector<float> ry_score_ = {};
  ObjMapperParams params_;

  // per-dimension variance of the orientation [yaw, pitch, roll]
  Eigen::Vector3d orientation_variance_;

  // covariance matrix for position uncertainty
  Eigen::Matrix3d position_uncertainty_;

  // DISALLOW_COPY_AND_ASSIGN(ObjMapper);

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
