/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/perception/traffic_light/base/tl_shared_data.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool MultiCamerasProjection::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  std::string model_name = "MultiCamerasProjection";
  const ModelConfig *model_config = config_manager->GetModelConfig(model_name);
  if (model_config == nullptr) {
    AERROR << "not found model: " << model_name;
    return false;
  }

  // Read camera names from config file
  std::vector<std::string> camera_names;
  std::string single_projection_name;
  if (!model_config->GetValue("camera_names", &camera_names)) {
    AERROR << "camera_names not found." << name();
    return false;
  }
  if (!model_config->GetValue("SingleProjection", &single_projection_name)) {
    AERROR << "SingleProjection not found." << name();
    return false;
  }

  AINFO << "number of camera_names: " << camera_names.size();
  AINFO << "SingleProjection name: " << single_projection_name;

  // Read each camera's config
  std::string camera_extrinsic_file;
  std::string camera_intrinsic_file;
  std::unordered_map<std::string, CameraCoeffient> camera_coeffients;
  for (size_t i = 0; i < camera_names.size(); ++i) {
    const auto &camera_model_name = camera_names[i];
    const ModelConfig *camera_model_config =
        config_manager->GetModelConfig(camera_model_name);
    if (camera_model_config == nullptr) {
      AERROR << "not found camera model: " << camera_model_name;
      return false;
    }

    if (!camera_model_config->GetValue("camera_extrinsic_file",
                                       &camera_extrinsic_file)) {
      AERROR << "camera_extrinsic_file not found." << name();
      return false;
    }
    if (!camera_model_config->GetValue("camera_intrinsic_file",
                                       &camera_intrinsic_file)) {
      AERROR << "camera_intrinsic_file not found." << name();
      return false;
    }

    CameraCoeffient camera_coeffient;
    if (!camera_coeffient.init(camera_model_name, camera_extrinsic_file,
                               camera_intrinsic_file)) {
      AERROR << camera_model_name << " Projection init failed.";
      return false;
    }
    AINFO << "init " << camera_names[i] << " coeffient succeeded.";
    camera_coeffients[camera_names[i]] = camera_coeffient;
    camera_names_.push_back(camera_names[i]);
  }

  projection_.reset(
      BaseProjectionRegisterer::GetInstanceByName(single_projection_name));
  if (projection_ == nullptr) {
    AERROR << "MultiCamerasProjection new projection failed. name:"
           << single_projection_name;
    return false;
  }
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    auto &camera_coeffient = camera_coeffients[camera_names_[i]];
    camera_coeffient.camera_extrinsic =
        camera_coeffient.camera_extrinsic.inverse().eval();

    AINFO << "Lidar to " << camera_names_[i] << " transform: ";
    AINFO << camera_coeffient.camera_extrinsic;
  }
  camera_coeffient_.resize(camera_names_.size());
  camera_coeffient_[kLongFocusIdx] = camera_coeffients["camera_25mm_focus"];
  camera_coeffient_[kShortFocusIdx] = camera_coeffients["camera_6mm_focus"];
  // auto &short_focus_camera_coeffient = camera_coeffients["camera_6mm_focus"];
  // auto &long_focus_camera_coeffient = camera_coeffients["camera_25mm_focus"];
  camera_coeffient_[kLongFocusIdx].camera_extrinsic =
      camera_coeffient_[kLongFocusIdx].camera_extrinsic *
      camera_coeffient_[kShortFocusIdx].camera_extrinsic;
  AINFO << "Lidar to long(25mm): ";
  AINFO << camera_coeffient_[kLongFocusIdx].camera_extrinsic;
  return true;
}

bool MultiCamerasProjection::Project(const CarPose &pose,
                                     const ProjectOption &option,
                                     Light *light) const {
  const Eigen::Matrix4d mpose = pose.pose();
  const apollo::hdmap::Signal &tl_info = light->info;
  bool ret = true;

  auto camera_id = static_cast<int>(option.camera_id);
  if (camera_id < 0 || camera_id >= kCountCameraId) {
    AERROR << "Projection get invalid camera_id: " << camera_id
           << ", check camera parameters file.";
    return false;
  }
  AINFO << "Begin project camera: " << option.camera_id;
  ret =
      projection_->Project(camera_coeffient_[camera_id], mpose, tl_info, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_id: " << camera_id;
    return false;
  }
  return true;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
