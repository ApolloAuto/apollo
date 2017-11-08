// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/26 17:58:00
// @file base_lights_projection.cpp
// @brief 
//
#include "modules/perception/traffic_light/projection/projection.h"

#include <gflags/gflags.h>
#include "modules/common/log.h"

namespace adu {
namespace perception {
namespace traffic_light {

bool TwoCamerasProjection::init() {

  config_manager::ConfigManager *config_manager
      = base::Singleton<config_manager::ConfigManager>::get();
  std::string model_name = FLAGS_traffic_light_projection;
  const config_manager::ModelConfig *model_config = NULL;
  if (!config_manager->get_model_config(model_name, &model_config)) {
    AERROR << "not found model: " << model_name;
    return false;
  }

  std::string lidar2gps_file;
  std::string lidar2camera_file;
  std::string camera_intrinsic_file;
  using config_manager::ConfigRead;

  try {
    lidar2gps_file = ConfigRead<std::string>::read(*model_config,
                                                   "short_lidar2gps_file");
    lidar2gps_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                       lidar2gps_file);

    lidar2camera_file = ConfigRead<std::string>::read(*model_config,
                                                      "short_lidar2camera_file");
    lidar2camera_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                          lidar2camera_file);

    camera_intrinsic_file = ConfigRead<std::string>::read(*model_config,
                                                          "short_camera_intrinsic_file");
    camera_intrinsic_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                              camera_intrinsic_file);

  } catch (const config_manager::ConfigManagerError &e) {
    AERROR << model_name << "Load short focus camera param: " << e.what();
    return false;
  }
  if (!_short_focus_camera_coeffient.init("Short Focus",
                                          lidar2gps_file,
                                          lidar2camera_file,
                                          camera_intrinsic_file)) {
    AERROR << "Short Focus Camera Projection init failed.";
    return false;
  }

  try {
    lidar2gps_file = ConfigRead<std::string>::read(*model_config,
                                                   "long_lidar2gps_file");
    lidar2gps_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                       lidar2gps_file);

    lidar2camera_file = ConfigRead<std::string>::read(*model_config,
                                                      "long_lidar2camera_file");
    lidar2camera_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                          lidar2camera_file);

    camera_intrinsic_file = ConfigRead<std::string>::read(*model_config,
                                                          "long_camera_intrinsic_file");
    camera_intrinsic_file = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                              camera_intrinsic_file);

  } catch (const config_manager::ConfigManagerError &e) {
    AERROR << model_name << "Load long focus camera param: " << e.what();
    return false;
  }
  if (!_long_focus_camera_coeffient.init("Long Focus",
                                         lidar2gps_file,
                                         lidar2camera_file,
                                         camera_intrinsic_file)) {
    AERROR << "Long Focus Camera Projection init failed.";
    return false;
  }

  _projection.reset(BaseProjectionRegisterer::get_instance_by_name(
      FLAGS_traffic_light_projection));
  if (_projection == nullptr) {
    AERROR << "TwoCamerasProjection new projection failed. name:"
           << FLAGS_traffic_light_projection;
    return false;
  }

  if (FLAGS_traffic_light_projection == "SingleBoundaryBasedProjection") {
    // in fact , because of the difference of definition parameters between cn and us,
    // the name is different.
    // for the short focus camera, they are:
    // lidar2camera is camera to lidar
    // lidar2gps is lidar to gps
    _short_focus_camera_coeffient.gps2camera = _short_focus_camera_coeffient.lidar2gps *
        _short_focus_camera_coeffient.lidar2camera;
    _short_focus_camera_coeffient.gps2camera = _short_focus_camera_coeffient.gps2camera
                                                                            .inverse();

    AINFO << "GPS to short: ";
    AINFO << _short_focus_camera_coeffient.gps2camera;
    // for the long focus camera, they are:
    // lidar2camera is long camera to short camera
    // lidar2gps is lidar to gps
    _long_focus_camera_coeffient.gps2camera = _short_focus_camera_coeffient.lidar2gps *
        _short_focus_camera_coeffient.lidar2camera *
        _long_focus_camera_coeffient.lidar2camera;
    _long_focus_camera_coeffient.gps2camera = _long_focus_camera_coeffient.gps2camera.inverse();
    AINFO << "GPS to long: ";
    AINFO << _long_focus_camera_coeffient.gps2camera;
  }
  return true;
}

bool TwoCamerasProjection::project(const CarPose &pose,
                                   const ProjectOption &option,
                                   Light *light) const {

  const Eigen::Matrix4d mpose = pose.get_pose();
  const adu::common::hdmap::Signal &tl_info = light->info;
  bool ret = true;
  switch (option.camera_id) {
    case CameraId::LONG_FOCUS:ret = _projection
          ->project(_long_focus_camera_coeffient, mpose, tl_info, light);
      break;
    case CameraId::SHORT_FOCUS:ret = _projection
          ->project(_short_focus_camera_coeffient, mpose, tl_info, light);
      break;
    default:AERROR << "Projection get unkown camera_id:" << option.camera_id;
      return false;
  }
  if (!ret) {
    XLOG(WARN) << "Projection failed projection the traffic light. "
               << "camera_id:" << option.camera_id;
    return false;
  }
  return true;
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
