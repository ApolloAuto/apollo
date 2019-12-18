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
#include "modules/perception/camera/lib/obstacle/transformer/multicue/obj_mapper.h"

namespace apollo {
namespace perception {
namespace camera {

void ObjMapperParams::set_default() {
  nr_bins_z = 15;
  nr_bins_ry = 36;
  boundary_len = 20;
  max_nr_iter = 10;

  small_bbox_height = 50.0f;
  factor_small = 0.6f;
  learning_r = 0.7f;
  reproj_err = 4 * sqrtf(2.0f);
  rz_ratio = 0.1f;
  abnormal_h_veh = 0.8f;
  stable_w = 0.5f;
  occ_ratio = 0.9f;
  depth_min = 0.1f;
  dist_far = 100.0f;
  eps_mapper = 1e-5f;

  iou_suc = 0.5f;
  iou_high = 0.7f;
  angle_resolution_degree = 10.0f;
}

bool ObjMapper::SolveCenterFromNearestVerticalEdge(const float *bbox,
                                                   const float *hwl, float ry,
                                                   float *center,
                                                   float *center_2d) const {
  center[0] = center[1] = center[2] = 0.0f;
  float height_bbox = bbox[3] - bbox[1];
  float width_bbox = bbox[2] - bbox[0];
  if (width_bbox <= 0.0f || height_bbox <= 0.0f) {
    AERROR << "width or height of bbox is 0";
    return false;
  }

  if (common::IRound(bbox[3]) >= height_ - 1) {
    height_bbox /= params_.occ_ratio;
  }

  float f = (k_mat_[0] + k_mat_[4]) / 2;
  float depth = f * hwl[0] * common::IRec(height_bbox);

  // compensate from the nearest vertical edge to center
  const float PI = common::Constant<float>::PI();
  float theta_bbox = static_cast<float>(atan(hwl[1] * common::IRec(hwl[2])));
  float radius_bbox =
      common::ISqrt(common::ISqr(hwl[2] / 2) + common::ISqr(hwl[1] / 2));

  float abs_ry = fabsf(ry);
  float theta_z = std::min(abs_ry, PI - abs_ry) + theta_bbox;
  theta_z = std::min(theta_z, PI - theta_z);
  depth += static_cast<float>(fabs(radius_bbox * sin(theta_z)));

  // back-project to solve center
  center_2d[0] = (bbox[0] + bbox[2]) / 2;
  center_2d[1] = (bbox[1] + bbox[3]) / 2;
  if (hwl[1] > params_.stable_w) {
    GetCenter(bbox, depth, ry, hwl, center, center_2d);
  } else {
    center[2] = depth;
    UpdateCenterViaBackProjectZ(bbox, hwl, center_2d, center);
  }

  return center[2] > params_.depth_min;
}

bool ObjMapper::Solve3dBboxGivenOneFullBboxDimensionOrientation(
    const float *bbox, const float *hwl, float *ry, float *center) {
  const float PI = common::Constant<float>::PI();
  const float PI_HALF = PI / 2;
  const float small_angle_diff =
      common::IDegreeToRadians(params_.angle_resolution_degree);
  float center_2d[2] = {0};
  bool success =
      SolveCenterFromNearestVerticalEdge(bbox, hwl, *ry, center, center_2d);
  float min_x = static_cast<float>(params_.boundary_len);
  float min_y = static_cast<float>(params_.boundary_len);
  float max_x = static_cast<float>(width_ - params_.boundary_len);
  float max_y = static_cast<float>(height_ - params_.boundary_len);
  bool truncated = bbox[0] <= min_x || bbox[2] >= max_x || bbox[1] <= min_y ||
                   bbox[3] >= max_y;
  float dist_rough = sqrtf(common::ISqr(center[0]) + common::ISqr(center[2]));
  bool ry_pred_is_not_reliable = dist_rough > params_.dist_far &&
                                 bbox[3] - bbox[1] < params_.small_bbox_height;
  if (ry_pred_is_not_reliable || std::abs(*ry - PI_HALF) < small_angle_diff ||
      std::abs(*ry + PI_HALF) < small_angle_diff) {
    *ry = *ry > 0.0f ? PI_HALF : -PI_HALF;
  }
  if (!truncated) {
    PostRefineOrientation(bbox, hwl, center, ry);
    success =
        SolveCenterFromNearestVerticalEdge(bbox, hwl, *ry, center, center_2d);
    PostRefineZ(bbox, hwl, center_2d, *ry, center);
  } else {
    FillRyScoreSingleBin(*ry);
  }
  return success &&
         GetProjectionScore(*ry, bbox, hwl, center, true) > params_.iou_suc;
}

bool ObjMapper::Solve3dBbox(const ObjMapperOptions &options, float center[3],
                            float hwl[3], float *ry) {
  // set default value for variance
  set_default_variance();
  float var_yaw = 0.0f;
  float var_z = 0.0f;

  // get input from options
  memcpy(hwl, options.hwl, sizeof(float) * 3);
  float bbox[4] = {0};
  memcpy(bbox, options.bbox, sizeof(float) * 4);
  *ry = options.ry;
  bool check_dimension = options.check_dimension;
  int type_min_vol_index = options.type_min_vol_index;

  // check input hwl insanity
  if (options.is_veh && check_dimension) {
    assert(type_min_vol_index >= 0);
    const std::vector<float> &kVehHwl = object_template_manager_->VehHwl();
    const float *tmplt_with_min_vol = &kVehHwl[type_min_vol_index];
    float min_tmplt_vol =
        tmplt_with_min_vol[0] * tmplt_with_min_vol[1] * tmplt_with_min_vol[2];
    float shrink_ratio_vol = common::ISqr(sqrtf(params_.iou_high));
    shrink_ratio_vol *= shrink_ratio_vol;
    // float shrink_ratio_vol = sqrt(params_.iou_high);
    if (hwl[0] < params_.abnormal_h_veh ||
        hwl[0] * hwl[1] * hwl[2] < min_tmplt_vol * shrink_ratio_vol) {
      memcpy(hwl, tmplt_with_min_vol, sizeof(float) * 3);
    } else {
      float hwl_tmplt[3] = {hwl[0], hwl[1], hwl[2]};
      int tmplt_index = -1;
      float score = object_template_manager_->VehObjHwlBySearchTemplates(
          hwl_tmplt, &tmplt_index);
      float thres_min_score = shrink_ratio_vol;

      const int kNrDimPerTmplt = object_template_manager_->NrDimPerTmplt();
      bool search_success = score > thres_min_score;
      bool is_same_type = (type_min_vol_index / kNrDimPerTmplt) == tmplt_index;
      const std::map<TemplateIndex, int> &kLookUpTableMinVolumeIndex =
          object_template_manager_->LookUpTableMinVolumeIndex();
      bool is_car_pred =
          type_min_vol_index ==
          kLookUpTableMinVolumeIndex.at(TemplateIndex::CAR_MIN_VOLUME_INDEX);

      bool hwl_is_reliable = search_success && is_same_type;
      if (hwl_is_reliable) {
        memcpy(hwl, hwl_tmplt, sizeof(float) * 3);
      } else if (is_car_pred) {
        const float *tmplt_with_median_vol =
            tmplt_with_min_vol + kNrDimPerTmplt;
        memcpy(hwl, tmplt_with_median_vol, sizeof(float) * 3);
      }
    }
  }

  // call 3d solver
  bool success =
      Solve3dBboxGivenOneFullBboxDimensionOrientation(bbox, hwl, ry, center);

  // calculate variance for yaw & z
  float yaw_score_mean =
      common::IMean(ry_score_.data(), static_cast<int>(ry_score_.size()));
  float yaw_score_sdv = common::ISdv(ry_score_.data(), yaw_score_mean,
                                     static_cast<int>(ry_score_.size()));
  var_yaw = common::ISqrt(common::IRec(yaw_score_sdv + params_.eps_mapper));

  float z = center[2];
  float rz = z * params_.rz_ratio;
  float nr_bins_z = static_cast<float>(params_.nr_bins_z);
  std::vector<float> buffer(static_cast<size_t>(2 * nr_bins_z), 0);
  float *score_z = buffer.data();
  float dz = 2 * rz / nr_bins_z;
  float z_start = std::max(z - rz, params_.depth_min);
  float z_end = z + rz;
  int count_z_test = 0;
  for (float z_test = z_start; z_test <= z_end; z_test += dz) {
    float center_test[3] = {center[0], center[1], center[2]};
    float sf = z_test * common::IRec(center_test[2]);
    common::IScale3(center_test, sf);
    float score_test = GetProjectionScore(*ry, bbox, hwl, center_test);
    score_z[count_z_test++] = score_test;
  }
  float z_score_mean = common::IMean(score_z, count_z_test);
  float z_score_sdv = common::ISdv(score_z, z_score_mean, count_z_test);
  var_z = common::ISqr(common::IRec(z_score_sdv + params_.eps_mapper));

  // fill the position_uncertainty_ and orientation_variance_
  orientation_variance_(0) = var_yaw;
  float bbox_cx = (bbox[0] + bbox[2]) / 2;
  float focal = (k_mat_[0] + k_mat_[4]) / 2;
  float sf_z_to_x = fabsf(bbox_cx - k_mat_[2]) * common::IRec(focal);
  float var_x = var_z * common::ISqr(sf_z_to_x);
  float var_xz = sf_z_to_x * var_z;
  position_uncertainty_(0, 0) = var_x;
  position_uncertainty_(2, 2) = var_z;
  position_uncertainty_(0, 2) = position_uncertainty_(2, 0) = var_xz;
  return success;
}

void ObjMapper::PostRefineOrientation(const float *bbox, const float *hwl,
                                      const float *center, float *ry) {
  const int kNrBinsRy = static_cast<int>(ry_score_.size());
  const float PI = common::Constant<float>::PI();
  const float PI_HALF = PI * 0.5f;
  const float D_RY = 2 * PI / static_cast<float>(kNrBinsRy);

  float ry_test = -PI;
  float ry_best = -PI;
  float score_best = 0.0f;
  float score_cur = GetProjectionScore(*ry, bbox, hwl, center, true);
  int count_bin = 0;
  while (ry_test < PI - params_.eps_mapper) {
    if (CalAngleDiff(ry_test, *ry) > PI_HALF) {
      ry_test += D_RY;
      ry_score_[count_bin++ % kNrBinsRy] = 0.0f;
      continue;
    }

    float score_test = GetProjectionScore(ry_test, bbox, hwl, center, true);
    if (score_test > score_best) {
      score_best = score_test;
      ry_best = ry_test;
    }
    ry_test += D_RY;
    ry_score_[count_bin++ % kNrBinsRy] = score_test;
  }
  common::IUnitize(ry_score_.data(), kNrBinsRy);
  if (score_best > params_.iou_high && score_cur < params_.iou_suc) {
    *ry = ry_best;
  }

  float bbox_res[4] = {0};
  float score_final =
      GetProjectionScore(*ry, bbox, hwl, center, true, bbox_res);
  if (score_final > params_.iou_high) {
    return;
  } else if (bbox[2] - bbox[0] <
             (bbox_res[2] - bbox_res[0]) * params_.factor_small) {
    *ry = *ry > 0 ? PI_HALF : -PI_HALF;
    FillRyScoreSingleBin(*ry);
  } else if ((bbox[2] - bbox[0]) * params_.factor_small >
             bbox_res[2] - bbox_res[0]) {
    *ry = 0.0f;
    FillRyScoreSingleBin(*ry);
  }
}

void ObjMapper::GetCenter(const float *bbox, const float &z_ref,
                          const float &ry, const float *hwl, float *center,
                          float *x) const {
  float x_target[2] = {(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2};
  const float kMinCost = params_.reproj_err;
  const float EPS_COST_DELTA = params_.eps_mapper;
  const float LR = params_.learning_r;
  const int MAX_ITERATION = params_.max_nr_iter;

  float cost_pre = 2.0f * static_cast<float>(width_);
  float cost_delta = 0.0f;
  float center_test[3] = {0};
  float rot[9] = {0};
  GenRotMatrix(ry, rot);
  int iter = 1;
  bool stop = false;
  float h = hwl[0];
  float w = hwl[1];
  float l = hwl[2];
  float x_corners[8] = {0};
  float y_corners[8] = {0};
  float z_corners[8] = {0};
  GenCorners(h, w, l, x_corners, y_corners, z_corners);

  float x_max_flt = static_cast<float>(width_ - 1);
  float y_max_flt = static_cast<float>(height_ - 1);

  // std::cout << "start to iteratively search the center..." << std::endl;
  while (!stop) {
    common::IBackprojectCanonical(x, k_mat_, z_ref, center_test);
    center_test[1] += hwl[0] / 2;
    float x_min = FLT_MAX;
    float x_max = -FLT_MAX;
    float y_min = FLT_MAX;
    float y_max = -FLT_MAX;
    float x_proj[3] = {0};

    for (int i = 0; i < 8; ++i) {
      // bbox from x_proj
      float x_box[3] = {x_corners[i], y_corners[i], z_corners[i]};
      common::IProjectThroughKRt(k_mat_, rot, center_test, x_box, x_proj);
      x_proj[0] *= common::IRec(x_proj[2]);
      x_proj[1] *= common::IRec(x_proj[2]);
      x_min = std::min(x_min, x_proj[0]);
      x_max = std::max(x_max, x_proj[0]);
      y_min = std::min(y_min, x_proj[1]);
      y_max = std::max(y_max, x_proj[1]);

      // truncation processing
      x_min = std::min(std::max(x_min, 0.0f), x_max_flt);
      x_max = std::min(std::max(x_max, 0.0f), x_max_flt);
      y_min = std::min(std::max(y_min, 0.0f), y_max_flt);
      y_max = std::min(std::max(y_max, 0.0f), y_max_flt);
    }
    float x_cur[2] = {(x_min + x_max) / 2, (y_min + y_max) / 2};
    float cost_cur = common::ISqrt(common::ISqr(x_cur[0] - x_target[0]) +
                                   common::ISqr(x_cur[1] - x_target[1]));

    if (cost_cur >= cost_pre) {
      stop = true;
    } else {
      memcpy(center, center_test, sizeof(float) * 3);
      cost_delta = (cost_pre - cost_cur) / cost_pre;
      cost_pre = cost_cur;
      x[0] += (x_target[0] - x_cur[0]) * LR;
      x[1] += (x_target[1] - x_cur[1]) * LR;
      ++iter;
      stop = iter >= MAX_ITERATION || cost_delta < EPS_COST_DELTA ||
             cost_pre < kMinCost;
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
