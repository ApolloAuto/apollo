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

#include <memory>
#include <string>

#include "Eigen/Core"

#include "modules/perception/camera_location_estimation/transformer/multicue/proto/multicue.pb.h"

#include "modules/perception/camera_location_estimation/interface/base_transformer.h"
#include "modules/perception/camera_location_estimation/transformer/multicue/obj_mapper.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/camera/common/object_template_manager.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class MultiCueTransformer : public BaseTransformer {
 public:
  MultiCueTransformer() = default;

  virtual ~MultiCueTransformer() = default;
  /**
   * @brief Init for ObjectTemplate and Object map
   *
   * @param options configuration for object template and object map
   * @return true
   * @return false
   */
  bool Init(const TransformerInitOptions &options =
                TransformerInitOptions()) override;

  /**
   * @brief Transform 2D detections to 3D bounding box
   *
   * @param frame input camera frame
   * @return true
   * @return false
   */
  bool Transform(onboard::CameraFrame *frame) override;

  std::string Name() const override { return "MultiCueTransformer"; }

 private:
  void SetObjMapperOptions(base::ObjectPtr obj,
                           const Eigen::Matrix3f &camera_k_matrix,
                           int width_image, int height_image,
                           ObjMapperOptions *obj_mapper_options,
                           float *theta_ray);
  int MatchTemplates(base::ObjectSubType sub_type, float *dimension_hwl);
  void FillResults(float object_center[3], float dimension_hwl[3],
                   float rotation_y, const Eigen::Affine3d &camera2world_pose,
                   float theta_ray, base::ObjectPtr obj);

 private:
  MulticueParam multicue_param_;
  std::unique_ptr<ObjMapper> mapper_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
