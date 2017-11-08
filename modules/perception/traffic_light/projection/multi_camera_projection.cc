// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/07/31 
// @file multi_camera_projection.cpp
// @brief 
//
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

#include <gflags/gflags.h>
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool MultiCamerasProjection::init() {
  ConfigManager *config_manager
      = ConfigManager::instance();
  std::string model_name = FLAGS_traffic_light_projection;
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
  std::string lidar2gps_file;
  std::string lidar2camera_file;
  std::string camera_intrinsic_file;

  for (size_t i = 0; i < camera_names.size(); ++i) {
    const auto &camera_model_name = camera_names[i];
    const ModelConfig *camera_model_config = NULL;
    if (!config_manager->GetModelConfig(camera_model_name, &camera_model_config)) {
      AERROR << "not found camera model: " << camera_model_name;
      return false;
    }

    if (!model_config->GetValue("lidar2gps_file", &lidar2gps_file)) {
      AERROR << "lidar2gps_file not found." << name();
      return false;
    }
    if (!model_config->GetValue("lidar2camera_file", &lidar2camera_file)) {
      AERROR << "lidar2camera_file not found." << name();
      return false;
    }
    if (!model_config->GetValue("camera_intrinsic_file", &camera_intrinsic_file)) {
      AERROR << "camera_intrinsic_file not found." << name();
      return false;
    }
    lidar2gps_file = FileUtil::GetAbsolutePath(config_manager->work_root(),
                                               lidar2gps_file);

    lidar2camera_file = FileUtil::GetAbsolutePath(config_manager->work_root(),
                                                  lidar2camera_file);

    camera_intrinsic_file = FileUtil::GetAbsolutePath(config_manager->work_root(),
                                                      camera_intrinsic_file);


    // we can skip wide/narrow camera if their params. file does not exist
    bool skip_camera = (!FileUtil::Exists(lidar2gps_file) ||
        !FileUtil::Exists(lidar2camera_file) ||
        !FileUtil::Exists(camera_intrinsic_file)) &&
        ((camera_model_name == "camera_2mm_focus") ||
            (camera_model_name == "camera_12mm_focus"));
    if (skip_camera) {
      AINFO << "Init " << camera_model_name
            << " failed, continue load other camera coeffient.";
      continue;
    }

    CameraCoeffient camera_coeffient;
    if (!camera_coeffient.init(
        camera_model_name,
        lidar2gps_file,
        lidar2camera_file,
        camera_intrinsic_file)) {
      AERROR << camera_model_name << " Projection init failed.";
      return false;
    }
    AINFO << "init " << camera_names[i] << " coeffient succeeded.";
    _camera_coeffients[camera_names[i]] = camera_coeffient;
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
    if (_camera_names[i] == "camera_25mm_focus") {
      continue;
    }

    auto &camera_coeffient = _camera_coeffients[_camera_names[i]];
    camera_coeffient.gps2camera = camera_coeffient.lidar2gps * camera_coeffient.lidar2camera;
    camera_coeffient.gps2camera = camera_coeffient.gps2camera.inverse();

    AINFO << "GPS to " << _camera_names[i] << ": ";
    AINFO << camera_coeffient.gps2camera;
  }

  // for the long focus camera, they are:
  // lidar2camera is long camera to short(6mm) camera
  // lidar2gps is lidar to gps
  auto &short_focus_camera_coeffient = _camera_coeffients["camera_6mm_focus"];
  auto &long_focus_camera_coeffient = _camera_coeffients["camera_25mm_focus"];
  long_focus_camera_coeffient.gps2camera = short_focus_camera_coeffient.lidar2gps *
      short_focus_camera_coeffient.lidar2camera *
      long_focus_camera_coeffient.lidar2camera;
  long_focus_camera_coeffient.gps2camera = long_focus_camera_coeffient.gps2camera.inverse();
  AINFO << "GPS to long(25mm): ";
  AINFO << long_focus_camera_coeffient.gps2camera;

  return true;
}

bool MultiCamerasProjection::project(const CarPose &pose,
                                     const ProjectOption &option,
                                     Light *light) const {
  const Eigen::Matrix4d mpose = pose.pose();
  const apollo::hdmap::Signal &tl_info = light->info;
  bool ret = true;

  std::map<int, CameraCoeffient> camera_id_to_coeffient;

  camera_id_to_coeffient[static_cast<int>(CameraId::SHORT_FOCUS)] =
      _camera_coeffients.at("camera_6mm_focus");
  camera_id_to_coeffient[static_cast<int>(CameraId::LONG_FOCUS)] =
      _camera_coeffients.at("camera_25mm_focus");

  auto camera_id = static_cast<int>(option.camera_id);
  if (camera_id_to_coeffient.find(camera_id) == camera_id_to_coeffient.end()) {
    AERROR << "Projection get invalid camera_id: " << camera_id
           << ", check camera parameters file.";
    return false;
  }

  ret = _projection->project(camera_id_to_coeffient[camera_id], mpose, tl_info, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_id: " << camera_id;
    return false;
  }
  return true;
}

bool MultiCamerasProjection::has_camera(const CameraId &cam_id) {
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
