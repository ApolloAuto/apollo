/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "Eigen/Core"

#include "modules/perception/camera_location_estimation/transformer/singlestage/proto/singlestage.pb.h"

#include "modules/perception/camera_location_estimation/interface/base_transformer.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

// hyper parameters
struct TransformerParams {
  TransformerParams() { set_default(); }

  void set_default();

  int max_nr_iter;
  float learning_rate;
  float k_min_cost;
  float eps_cost;
};

class SingleStageTransformer : public BaseTransformer {
 public:
  SingleStageTransformer() = default;
  virtual ~SingleStageTransformer() = default;

  /**
   * @brief Init transformer by setting parameters
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const TransformerInitOptions &options =
                TransformerInitOptions()) override;
  /**
   * @brief transform 2D detections to 3D bounding box
   *
   * @param frame camera frame
   * @return true
   * @return false
   */
  bool Transform(onboard::CameraFrame *frame) override;

  std::string Name() const override { return "SingleStageTransformer"; }

 private:
  void FillResults(float object_center[3], float dimension_hwl[3],
                   float rotation_y, const Eigen::Affine3d &camera2world_pose,
                   float theta_ray, base::ObjectPtr obj);
  float CenterPointFromBbox(const float *bbox, const float *hwl, float ry,
                            float *center, float *center2d, const float *k_mat,
                            int height, int width);
  void ConstraintCenterPoint(const float *bbox, const float &z_ref,
                             const float &ry, const float *hwl,
                             const float *k_mat, float *center, float *x,
                             int height, int width);

 private:
  SinglestageParam singlestage_param_;
  TransformerParams params_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
