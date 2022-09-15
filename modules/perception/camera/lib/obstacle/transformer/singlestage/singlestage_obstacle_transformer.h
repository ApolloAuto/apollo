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

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/common/twod_threed_util.h"
#include "modules/perception/camera/lib/interface/base_obstacle_transformer.h"
#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/pipeline/proto/stage/singlestage.pb.h"
#include "modules/perception/pipeline/stage.h"

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

class SingleStageObstacleTransformer : public BaseObstacleTransformer {
 public:
  SingleStageObstacleTransformer() : BaseObstacleTransformer() {}
  virtual ~SingleStageObstacleTransformer() = default;

  bool Init(const ObstacleTransformerInitOptions &options =
                ObstacleTransformerInitOptions()) override;

  // @brief: transform 2D detections to 3D bounding box
  // @param [in]: frame
  // @param [out]: frame
  bool Transform(const ObstacleTransformerOptions &options,
                 CameraFrame *frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  int MatchTemplates(base::ObjectSubType sub_type, float *dimension_hwl);
  void FillResults(float object_center[3], float dimension_hwl[3],
                   float rotation_y, Eigen::Affine3d camera2world_pose,
                   float theta_ray, base::ObjectPtr obj);
  float CenterPointFromBbox(const float *bbox, const float *hwl,
                            float ry, float *center, float *center2d,
                            const float* k_mat, int height, int width);
  void ConstraintCenterPoint(const float *bbox, const float &z_ref,
                             const float &ry, const float *hwl,
                             const float* k_mat, float *center,
                             float *x, int height, int width);

 private:
  singlestage::SinglestageParam singlestage_param_;
  int image_width_ = 0;
  int image_height_ = 0;
  TransformerParams params_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
