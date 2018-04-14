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

#include "modules/perception/lib/config_manager/calibration_config_manager.h"

#include <cmath>

#include "Eigen/Eigen"
#include "gflags/gflags.h"
#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

// @brief load transformation_matrix from file
bool load_transformation_matrix_from_file(
    const std::string &file_name, Eigen::Matrix4d *transformation_matrix) {
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
      AWARN << "Open TransformationMatrix File:" << file_name << " failed.";
      return false;
    }
    if (!config["transform"]) {
      AWARN << "Open TransformationMatrix File:" << file_name
            << " has no transform.";
      return false;
    }
    // fill translation
    if (config["transform"]["translation"]) {
      (*transformation_matrix)(0, 3) =
          config["transform"]["translation"]["x"].as<double>();
      (*transformation_matrix)(1, 3) =
          config["transform"]["translation"]["y"].as<double>();
      (*transformation_matrix)(2, 3) =
          config["transform"]["translation"]["z"].as<double>();
    } else {
      AWARN << "TransformationMatrix File:" << file_name
            << " has no transform:translation.";
      return false;
    }
    // fill rotation
    if (config["transform"]["rotation"]) {
      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      (*transformation_matrix).block<3, 3>(0, 0) = rotation.toRotationMatrix();
    } else {
      AWARN << "TransformationMatrix File:" << file_name
            << " has no transform:rotation.";
      return false;
    }
  } catch (const YAML::Exception &e) {
    AERROR << file_name << " load failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }

  // fill trivial elements
  for (int i = 0; i < 3; i++) {
    (*transformation_matrix)(3, i) = 0.0;
  }
  (*transformation_matrix)(3, 3) = 1.0;
  return true;
}

bool load_matrix4d_from_file(const std::string &file_name,
                             const std::string &key, Eigen::Matrix4d *matrix) {
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
      AWARN << "Open Matrix File:" << file_name << " failed.";
      return false;
    }
    if (!config[key]) {
      AWARN << "Matrix File:" << file_name << " has no key:" << key;
      return false;
    }

    (*matrix)(0, 0) = config[key]["0"]["0"].as<double>();
    (*matrix)(0, 1) = config[key]["0"]["1"].as<double>();
    (*matrix)(0, 2) = config[key]["0"]["2"].as<double>();
    (*matrix)(0, 3) = config[key]["0"]["3"].as<double>();
    (*matrix)(1, 0) = config[key]["1"]["0"].as<double>();
    (*matrix)(1, 1) = config[key]["1"]["1"].as<double>();
    (*matrix)(1, 2) = config[key]["1"]["2"].as<double>();
    (*matrix)(1, 3) = config[key]["1"]["3"].as<double>();
    (*matrix)(2, 0) = config[key]["2"]["0"].as<double>();
    (*matrix)(2, 1) = config[key]["2"]["1"].as<double>();
    (*matrix)(2, 2) = config[key]["2"]["2"].as<double>();
    (*matrix)(2, 3) = config[key]["2"]["3"].as<double>();
    (*matrix)(3, 0) = config[key]["3"]["0"].as<double>();
    (*matrix)(3, 1) = config[key]["3"]["1"].as<double>();
    (*matrix)(3, 2) = config[key]["3"]["2"].as<double>();
    (*matrix)(3, 3) = config[key]["3"]["3"].as<double>();
  } catch (const YAML::Exception &e) {
    AERROR << file_name << " load failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }

  return true;
}

bool CameraCoeffient::init(const std::string &camera_type,
                           const std::string &camera_extrinsic_file_name,
                           const std::string &camera_intrinsic_file_name) {
  camera_type_str = camera_type;

  try {
    if (!init_camera_extrinsic_matrix(camera_extrinsic_file_name)) {
      AERROR << camera_type_str << " init failed. lidar to camera matrix file:"
             << camera_extrinsic_file_name;
      return false;
    }

    if (!init_camera_intrinsic_matrix_and_distort_params(
            camera_intrinsic_file_name)) {
      AERROR << camera_type_str << " init failed. camera intrinsic matrix file:"
             << camera_intrinsic_file_name;
      return false;
    }
  } catch (const YAML::Exception &e) {
    AERROR << camera_type_str << " init failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }
  return true;
}

bool CameraCoeffient::init_camera_extrinsic_matrix(
    const std::string &file_name) {
  if (!load_transformation_matrix_from_file(file_name, &camera_extrinsic) &&
      !load_matrix4d_from_file(file_name, "T", &camera_extrinsic)) {
    AERROR << "Load camera_extrinsic matrix file failed. file:" << file_name;
    return false;
  }
  AINFO << camera_type_str
        << " camera_extrinsic matrix is:" << camera_extrinsic;
  return true;
}

bool CameraCoeffient::init_camera_intrinsic_matrix_and_distort_params(
    const std::string &file_name) {
  YAML::Node config = YAML::LoadFile(file_name);
  if (config["K"]) {
    camera_intrinsic(0, 0) = config["K"][0].as<double>();
    camera_intrinsic(0, 1) = config["K"][1].as<double>();
    camera_intrinsic(0, 2) = config["K"][2].as<double>();
    camera_intrinsic(0, 3) = 0.0;
    camera_intrinsic(1, 0) = config["K"][3].as<double>();
    camera_intrinsic(1, 1) = config["K"][4].as<double>();
    camera_intrinsic(1, 2) = config["K"][5].as<double>();
    camera_intrinsic(1, 3) = 0.0;
    camera_intrinsic(2, 0) = config["K"][6].as<double>();
    camera_intrinsic(2, 1) = config["K"][7].as<double>();
    camera_intrinsic(2, 2) = config["K"][8].as<double>();
    camera_intrinsic(2, 3) = 0.0;
  } else {
    AERROR << "load lidar camera intrinsic failed. file_name:" << file_name;
    return false;
  }

  if (config["D"]) {
    for (size_t i = 0; i < 5; ++i) {
      distort_params[i] = config["D"][i].as<double>();
    }
  } else {
    AERROR << "load camera distortion coeffients failed. file_name:"
           << file_name;
    return false;
  }

  if (config["height"]) {
    image_height = config["height"].as<size_t>();
  } else {
    AERROR << "load image height from camera intrinsic failed. file:"
           << file_name;
    return false;
  }
  if (config["width"]) {
    image_width = config["width"].as<size_t>();
  } else {
    AERROR << "Load image width from camera intrinsic failed. file:"
           << file_name;
    return false;
  }

  AINFO << camera_type_str << " camera intrinsic is:" << camera_intrinsic
        << " distort_params is:" << distort_params << " height:" << image_height
        << " width:" << image_width;
  return true;
}

CalibrationConfigManager::CalibrationConfigManager()
    : camera_calibration_(new CameraCalibration()) {
  work_root_ = FLAGS_work_root;
  camera_extrinsic_path_ = FLAGS_front_camera_extrinsics_file;
  camera_intrinsic_path_ = FLAGS_front_camera_intrinsics_file;
  init();
}

bool CalibrationConfigManager::init() {
  MutexLock lock(&mutex_);
  return init_internal();
}

bool CalibrationConfigManager::reset() {
  MutexLock lock(&mutex_);
  inited_ = false;
  return init_internal();
}

CalibrationConfigManager::~CalibrationConfigManager() {}

bool CalibrationConfigManager::init_internal() {
  if (inited_) {
    return true;
  }

  if (!camera_calibration_->init(camera_intrinsic_path_,
                                 camera_extrinsic_path_)) {
    AERROR << "init intrinsics failure: " << camera_intrinsic_path_ << " "
           << camera_extrinsic_path_;
    return false;
  }

  AINFO << "finish to load Calibration Configs.";

  inited_ = true;
  return inited_;
}

CameraCalibration::CameraCalibration()
    : _camera2car_pose(new Eigen::Matrix<double, 4, 4>()),
      _car2camera_pose(new Eigen::Matrix<double, 4, 4>()),
      undistort_handler_(new ImageGpuPreprocessHandler()),
      camera_model_(new CameraDistort<double>()) {}

CameraCalibration::~CameraCalibration() {}

bool CameraCalibration::init(const std::string &intrinsic_path,
                             const std::string &extrinsic_path) {
  if (!camera_coefficient_.init("", extrinsic_path, intrinsic_path)) {
    AERROR << "init camera coefficient failed";
    return false;
  }

  camera_intrinsic_ = camera_coefficient_.camera_intrinsic;
  image_height_ = camera_coefficient_.image_height;
  image_width_ = camera_coefficient_.image_width;
  *_camera2car_pose = camera_coefficient_.camera_extrinsic;
  *_car2camera_pose = _camera2car_pose->inverse();

  // TODO(later): BUGGY. Crash
  // if (!init_undistortion(intrinsic_path)) {
  //   AERROR << "init undistortion failed";
  //   return false;
  // }

  init_camera_model();
  calculate_homographic();
  AINFO << "Successfully loading intrinsic and extrinsic";
  return true;
}

void CameraCalibration::calculate_homographic() {
  auto camera_intrinsic_inverse = camera_intrinsic_.block(0, 0, 3, 3).inverse();
  auto car2camera_3_4 = (*_car2camera_pose).block(0, 0, 3, 4);
  Eigen::Matrix3d camera_2car_stripped;
  camera_2car_stripped.col(0) = car2camera_3_4.col(0);
  camera_2car_stripped.col(1) = car2camera_3_4.col(1);
  camera_2car_stripped.col(2) = car2camera_3_4.col(3);
  homography_mat_ = camera_2car_stripped.inverse() * camera_intrinsic_inverse;
  homography_mat_inverse_ = homography_mat_.inverse();
}

void CameraCalibration::init_camera_model() {
  camera_model_->set(camera_intrinsic_.block(0, 0, 3, 3), image_width_,
                     image_height_);
}

bool CameraCalibration::init_undistortion(const std::string &intrinsics_path) {
  AINFO << "Loading intrinsics: " << intrinsics_path;
  int err =
      undistort_handler_->init(intrinsics_path, FLAGS_obs_camera_detector_gpu);

  if (err != 0) {
    AERROR << "Undistortion initialization failed wiht error code: " << err;
    return false;
  }
  undistort_handler_->set_device(FLAGS_obs_camera_detector_gpu);
  return true;
}

}  // namespace perception
}  // namespace apollo
