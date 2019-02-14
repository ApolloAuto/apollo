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
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/obj_postprocessor.h"

// TODO(Xun & Yucheng): code completion
namespace apollo {
namespace perception {
namespace camera {

void ObjPostProcessorParams::set_default() {
  max_nr_iter = 5;
  sampling_ratio_low = 0.1f;
  weight_iou = 3.0f;
  learning_r = 0.2f;
  learning_r_decay = 0.9f;
  dist_far = 15.0f;
  shrink_ratio_iou = 0.9f;
  iou_good = 0.5f;
}

bool ObjPostProcessor::PostProcessObjWithGround(
    const ObjPostProcessorOptions &options, float center[3], float hwl[3],
    float *ry) {
  memcpy(hwl, options.hwl, sizeof(float) * 3);
  float bbox[4] = {0};
  memcpy(bbox, options.bbox, sizeof(float) * 4);
  *ry = options.ry;

  // soft constraints
  bool adjust_soft =
      AdjustCenterWithGround(bbox, hwl, *ry, options.plane, center);
  if (center[2] > params_.dist_far) {
    return adjust_soft;
  }

  // hard constraints
  bool adjust_hard = PostRefineCenterWithGroundBoundary(
      bbox, hwl, *ry, options.plane, options.line_segs, center,
      options.check_lowerbound);

  return adjust_soft || adjust_hard;
}

bool ObjPostProcessor::PostProcessObjWithDispmap(
    const ObjPostProcessorOptions &options, float center[3], float hwl[3],
    float *ry) {
  return true;
}

bool ObjPostProcessor::AdjustCenterWithGround(const float *bbox,
                                              const float *hwl, float ry,
                                              const float *plane,
                                              float *center) const {
  float iou_ini = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_ini < params_.iou_good) {  // ini pos is not good enough
    return false;
  }
  const float MIN_COST = hwl[2] * params_.sampling_ratio_low;
  const float EPS_COST_DELTA = 1e-1f;
  const float WEIGHT_IOU = params_.weight_iou;
  const int MAX_ITERATION = params_.max_nr_iter;

  float lr = params_.learning_r;
  float cost_pre = FLT_MAX;
  float cost_delta = 0.0f;
  float center_input[3] = {center[0], center[1], center[2]};
  float center_test[3] = {0};
  float x[3] = {0};
  int iter = 1;
  bool stop = false;

  // std::cout << "start to update the center..." << std::endl;
  while (!stop) {
    common::IProjectThroughIntrinsic(k_mat_, center, x);
    x[0] *= common::IRec(x[2]);
    x[1] *= common::IRec(x[2]);
    bool in_front = common::IBackprojectPlaneIntersectionCanonical(
        x, k_mat_, plane, center_test);
    if (!in_front) {
      memcpy(center, center_input, sizeof(float) * 3);
      return false;
    }
    float iou_cur = GetProjectionScore(ry, bbox, hwl, center);
    float iou_test = GetProjectionScore(ry, bbox, hwl, center_test);
    float dist = common::ISqrt(common::ISqr(center[0] - center_test[0]) +
                               common::ISqr(center[2] - center_test[2]));
    float cost_cur = dist + WEIGHT_IOU * (1.0f - (iou_cur + iou_test) / 2);
    // std::cout << "cost___ " << cost_cur << "@" << iter << std::endl;
    if (cost_cur >= cost_pre) {
      stop = true;
    } else {
      cost_delta = (cost_pre - cost_cur) / cost_pre;
      cost_pre = cost_cur;
      center[0] += (center_test[0] - center[0]) * lr;
      center[2] += (center_test[2] - center[2]) * lr;
      ++iter;
      stop = iter >= MAX_ITERATION || cost_delta < EPS_COST_DELTA ||
             cost_pre < MIN_COST;
    }
    lr *= params_.learning_r_decay;
  }
  float iou_res = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_res < iou_ini * params_.shrink_ratio_iou) {
    memcpy(center, center_input, sizeof(float) * 3);
    return false;
  }
  return true;
}

bool ObjPostProcessor::PostRefineCenterWithGroundBoundary(
    const float *bbox, const float *hwl, float ry, const float *plane,
    const std::vector<LineSegment2D<float>> &line_seg_limits, float *center,
    bool check_lowerbound) const {
  bool truncated_on_bottom =
      bbox[3] >= static_cast<float>(height_) -
                     (bbox[3] - bbox[1]) * params_.sampling_ratio_low;
  if (truncated_on_bottom) {
    return false;
  }

  float iou_before = GetProjectionScore(ry, bbox, hwl, center);
  int nr_line_segs = static_cast<int>(line_seg_limits.size());
  float depth_pts[4] = {0};
  float pts_c[12] = {0};
  int x_pts[4] = {0};

  GetDepthXPair(bbox, hwl, center, ry, depth_pts, x_pts, &pts_c[0]);

  float dxdz_acc[2] = {0};
  float ratio_x_over_z = center[0] * common::IRec(center[2]);
  for (int i = 0; i < nr_line_segs; ++i) {
    float dxdz[2] = {0};
    GetDxDzForCenterFromGroundLineSeg(line_seg_limits[i], plane, pts_c, k_mat_,
                                      width_, height_, ratio_x_over_z, dxdz,
                                      check_lowerbound);
    dxdz_acc[0] += dxdz[0];
    dxdz_acc[1] += dxdz[1];
  }
  center[0] += dxdz_acc[0];
  center[2] += dxdz_acc[1];

  float iou_after = GetProjectionScore(ry, bbox, hwl, center);
  if (iou_after < iou_before * params_.shrink_ratio_iou) {
    center[0] -= dxdz_acc[0];
    center[2] -= dxdz_acc[1];
    return false;
  }
  return true;
}

int ObjPostProcessor::GetDepthXPair(const float *bbox, const float *hwl,
                                    const float *center, float ry,
                                    float *depth_pts, int *x_pts,
                                    float *pts_c) const {
  int y_min = height_;
  float w_half = hwl[1] / 2;
  float l_half = hwl[2] / 2;
  float x_cor[4] = {l_half, l_half, -l_half, -l_half};
  float z_cor[4] = {w_half, -w_half, -w_half, w_half};
  float pts[12] = {x_cor[0], 0.0f, z_cor[0], x_cor[1], 0.0f, z_cor[1],
                   x_cor[2], 0.0f, z_cor[2], x_cor[3], 0.0f, z_cor[3]};
  float rot[9] = {0};
  GenRotMatrix(ry, rot);
  float pt_proj[3] = {0};
  float pt_c[3] = {0};
  float *pt = pts;
  bool save_pts_c = pts_c != nullptr;
  for (int i = 0; i < 4; ++i) {
    common::IProjectThroughExtrinsic(rot, center, pt, pt_c);
    common::IProjectThroughIntrinsic(k_mat_, pt_c, pt_proj);
    depth_pts[i] = pt_c[2];
    x_pts[i] = common::IRound(pt_proj[0] * common::IRec(pt_proj[2]));
    int y_proj = common::IRound(pt_proj[1] * common::IRec(pt_proj[2]));
    if (y_proj < y_min) {
      y_min = y_proj;
    }
    if (save_pts_c) {
      int i3 = i * 3;
      pts_c[i3] = pt_c[0];
      pts_c[i3 + 1] = pt_c[1];
      pts_c[i3 + 2] = pt_c[2];
    }
    pt += 3;
  }
  return y_min;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
