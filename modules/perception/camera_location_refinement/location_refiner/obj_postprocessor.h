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

#ifdef LOCAL_REPO
// todo(daohu527): need delete?
// local repo header path
#include "modules/perception/common/noncopyable.h"
#include "modules/perception/lib/common/twod_threed_util.h"
#else
// perception-camera header path
#include "modules/perception/common/camera/common/twod_threed_util.h"
#endif

namespace apollo {
namespace perception {
namespace camera {

struct ObjPostProcessorOptions {
  // ground + line segments
  float plane[4] = {0};
  std::vector<LineSegment2D<float>> line_segs = {};
  bool check_lowerbound = true;

  // disparity
  float *disp_map = nullptr;
  float baseline = 0.0f;

  float hwl[3] = {0};
  float bbox[4] = {0};
  float ry = 0.0f;
};

// hyper parameters
struct ObjPostProcessorParams {
  ObjPostProcessorParams() { set_default(); }
  void set_default();
  int max_nr_iter;
  float sampling_ratio_low;
  float weight_iou;
  float learning_r;
  float learning_r_decay;
  float dist_far;
  float shrink_ratio_iou;
  float iou_good;
};

class ObjPostProcessor {
 public:
  ObjPostProcessor() = default;
  virtual ~ObjPostProcessor() = default;
  /**
   * @brief process 2d 3d object
   *
   * @param k_mat 3x3 matrix
   * @param width image width
   * @param height image height
   */
  void Init(const float *k_mat, int width, int height) {
    memcpy(k_mat_, k_mat, sizeof(float) * 9);
    width_ = width;
    height_ = height;
  }
  /**
   * @brief init params
   *
   * @param options options for processing
   * @param center center of the object
   * @param hwl the hwl of the object
   * @param ry ry of the object
   * @return true
   * @return false
   */
  bool PostProcessObjWithGround(const ObjPostProcessorOptions &options,
                                float center[3], float hwl[3], float *ry);
  /**
   * @brief none
   *
   * @param options
   * @param center center of the object
   * @param hwl the hwl of the object
   * @param ry ry of the object
   * @return true
   * @return false
   */
  bool PostProcessObjWithDispmap(const ObjPostProcessorOptions &options,
                                 float center[3], float hwl[3], float *ry);

 private:
  float GetProjectionScore(float ry, const float *bbox, const float *hwl,
                           const float *center,
                           const bool &check_truncation = true) const {
    float rot[9] = {0};
    GenRotMatrix(ry, rot);
    float *bbox_null_1 = nullptr;
    float *bbox_null_2 = nullptr;
    return GetScoreViaRotDimensionCenter(&k_mat_[0], width_, height_, bbox, rot,
                                         hwl, center, check_truncation,
                                         bbox_null_1, bbox_null_2);
  }

  // refinement using ground-3d bbox consistency
  bool AdjustCenterWithGround(const float *bbox, const float *hwl, float ry,
                              const float *plane, float *center) const;

  // adjustment using on-ground line segments
  bool PostRefineCenterWithGroundBoundary(
      const float *bbox, const float *hwl, float ry, const float *plane,
      const std::vector<LineSegment2D<float>> &line_seg_limits, float *center,
      bool check_lowerbound) const;

  int GetDepthXPair(const float *bbox, const float *hwl, const float *center,
                    float ry, float *depth_pts, int *x_pts,
                    float *pts_c = nullptr) const;

 private:
  float k_mat_[9] = {0};
  int width_ = 0;
  int height_ = 0;
  ObjPostProcessorParams params_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
