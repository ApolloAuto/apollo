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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <gflags/gflags.h>
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool MultiCamerasProjection::init() {
  ConfigManager *config_manager
      = ConfigManager::instance();
  std::string model_name = "MultiCamerasProjection";
  const ModelConfig *model_config = NULL;
  if (!config_manager->GetModelConfig(model_name, &model_config)) {
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
  std::map<std::string, CameraCoeffient> camera_coeffients;
  for (size_t i = 0; i < camera_names.size(); ++i) {
    const auto &camera_model_name = camera_names[i];
    const ModelConfig *camera_model_config = NULL;
    if (!config_manager->GetModelConfig(camera_model_name, &camera_model_config)) {
      AERROR << "not found camera model: " << camera_model_name;
      return false;
    }

    if (!camera_model_config->GetValue("camera_extrinsic_file", &camera_extrinsic_file)) {
      AERROR << "camera_extrinsic_file not found." << name();
      return false;
    }
    if (!camera_model_config->GetValue("camera_intrinsic_file", &camera_intrinsic_file)) {
      AERROR << "camera_intrinsic_file not found." << name();
      return false;
    }

    CameraCoeffient camera_coeffient;
    if (!camera_coeffient.init(camera_model_name, camera_extrinsic_file, camera_intrinsic_file)) {
      AERROR << camera_model_name << " Projection init failed.";
      return false;
    }
    AINFO << "init " << camera_names[i] << " coeffient succeeded.";
    camera_coeffients[camera_names[i]] = camera_coeffient;
    _camera_names.push_back(camera_names[i]);
  }

  _projection.reset(BaseProjectionRegisterer::GetInstanceByName(
      single_projection_name));
  if (_projection == nullptr) {
    AERROR << "MultiCamerasProjection new projection failed. name:"
           << single_projection_name;
    return false;
  }

  // in fact , because of the difference of definition parameters between cn and us,
  // the name is different.
  // for the short focus(2mm, 6mm, 12mm) camera, they are:
  // lidar2camera is camera to lidar
  // lidar2gps is lidar to gps
  // if (FLAGS_traffic_light_projection == "SingleBoundaryBasedProjection")
  for (size_t i = 0; i < _camera_names.size(); ++i) {

    auto &camera_coeffient = camera_coeffients[_camera_names[i]];
    camera_coeffient.camera_extrinsic = camera_coeffient.camera_extrinsic.inverse().eval();

    AINFO << "Lidar to " << _camera_names[i] << " transform: ";
    AINFO << camera_coeffient.camera_extrinsic;
  }

  // for the long focus camera, they are:
  // lidar2camera is long camera to short(6mm) camera
  // lidar2gps is lidar to gps
  _camera_coeffient.resize(_camera_names.size());
  _camera_coeffient[kLongFocusIdx] = camera_coeffients["camera_25mm_focus"];
  _camera_coeffient[kShortFocusIdx] = camera_coeffients["camera_6mm_focus"];
  auto &short_focus_camera_coeffient = camera_coeffients["camera_6mm_focus"];
  auto &long_focus_camera_coeffient = camera_coeffients["camera_25mm_focus"];
  _camera_coeffient[kLongFocusIdx].camera_extrinsic =
      _camera_coeffient[kLongFocusIdx].camera_extrinsic * _camera_coeffient[kShortFocusIdx].camera_extrinsic;
  AINFO << "Lidar to long(25mm): ";
  AINFO << _camera_coeffient[kLongFocusIdx].camera_extrinsic;

  return true;
}

bool MultiCamerasProjection::project(const CarPose &pose,
                                     const ProjectOption &option,
                                     Light *light) const {
  const Eigen::Matrix4d mpose = pose.pose();
  const apollo::hdmap::Signal &tl_info = light->info;
  bool ret = true;

  std::map<int, CameraCoeffient> camera_id_to_coeffient;

  auto camera_id = static_cast<int>(option.camera_id);
  if (camera_id < 0 || camera_id >= kCountCameraId) {
    AERROR << "Projection get invalid camera_id: " << camera_id
           << ", check camera parameters file.";
    return false;
  }
  AINFO << "Begin project camera: " << option.camera_id;
  ret = _projection->project(_camera_coeffient[camera_id], mpose, tl_info, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_id: " << camera_id;
    return false;
  }
  return true;
}

bool MultiCamerasProjection::has_camera(const CameraId &cam_id) const {
  std::map<int, std::string> camera_id_to_camera_name;
  camera_id_to_camera_name[static_cast<int>(CameraId::SHORT_FOCUS)] = "camera_6mm_focus";
  camera_id_to_camera_name[static_cast<int>(CameraId::LONG_FOCUS)] = "camera_25mm_focus";

  auto itr = std::find(_camera_names.begin(), _camera_names.end(),
                       camera_id_to_camera_name[static_cast<int>(cam_id)]);

  // AINFO << "MultiCamerasProjection::has_camera camera_id: " << cam_id
  //         << ", has camera: " << (itr != _camera_names.end());
  return itr != _camera_names.end();
}

} // namespace traffic_light
} // namespace perception
} // namespace apollo
