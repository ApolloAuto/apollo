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
#include "modules/perception/traffic_light/interface/base_projection.h"

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>

namespace apollo {
namespace perception {
namespace traffic_light {

DEFINE_string(traffic_light_projection,
"",
"the projection enabled for traffic_light");

bool CameraCoeffient::init(const std::string &camera_type,
                           const std::string &lidar2gps_matrix_file_name,
                           const std::string &lidar2camera_matrix_file_name,
                           const std::string &camera_intrinsic_file_name) {

  camera_type_str = camera_type;

  try {
    if (!init_lidar_to_gps_matrix(lidar2gps_matrix_file_name)) {
      AERROR << camera_type_str << " init failed. lidar to gps matrix file:"
             << lidar2gps_matrix_file_name;
      return false;
    }

    if (!init_lidar_to_camera_matrix(lidar2camera_matrix_file_name)) {
      AERROR << camera_type_str << " init failed. lidar to camera matrix file:"
             << lidar2camera_matrix_file_name;
      return false;
    }

    if (!init_camera_intrinsic_matrix_and_distort_params(camera_intrinsic_file_name)) {
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

bool CameraCoeffient::init_lidar_to_gps_matrix(const std::string &file_name) {

  if (!load_transformation_matrix_from_file(file_name, &lidar2gps)) {
    AERROR << "Load lidar2gps matrix file failed. file:" << file_name;
    return false;
  }
  AINFO << camera_type_str << " lidar2gps matrix is:" << lidar2gps;
  return true;
}

bool CameraCoeffient::init_lidar_to_camera_matrix(const std::string &file_name) {

  if (!load_transformation_matrix_from_file(file_name, &lidar2camera) &&
      !load_matrix4d_from_file(file_name, "T", &lidar2camera)) {
    AERROR << "Load lidar2camera matrix file failed. file:" << file_name;
    return false;
  }
  AINFO << camera_type_str << " lidar2camera matrix is:" << lidar2camera;
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
    AERROR << "load camera distortion coeffients failed. file_name:" << file_name;
    return false;
  }

  if (config["height"]) {
    image_height = config["height"].as<size_t>();
  } else {
    AERROR << "load image height from camera intrinsic failed. file:" << file_name;
    return false;
  }
  if (config["width"]) {
    image_width = config["width"].as<size_t>();
  } else {
    AERROR << "Load image width from camera intrinsic failed. file:" << file_name;
    return false;
  }

  AINFO << camera_type_str << " camera intrinsic is:" << camera_intrinsic
        << " distort_params is:" << distort_params
        << " height:" << image_height << " width:" << image_width;
  return true;
}

//@brief load transformation_matrix from file
bool load_transformation_matrix_from_file(const std::string &file_name,
                                          Eigen::Matrix4d *transformation_matrix) {
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
      AWARN << "Open TransformationMatrix File:" << file_name << " failed.";
      return false;
    }
    if (!config["transform"]) {
      AWARN << "Open TransformationMatrix File:" << file_name << " has no transform.";
      return false;
    }
    //fill translation
    if (config["transform"]["translation"]) {
      (*transformation_matrix)(0, 3) = config["transform"]["translation"]["x"].as<double>();
      (*transformation_matrix)(1, 3) = config["transform"]["translation"]["y"].as<double>();
      (*transformation_matrix)(2, 3) = config["transform"]["translation"]["z"].as<double>();
    } else {
      AWARN << "TransformationMatrix File:" << file_name << " has no transform:translation.";
      return false;
    }
    //fill rotation
    if (config["transform"]["rotation"]) {
      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      (*transformation_matrix).block<3, 3>(0, 0) = rotation.toRotationMatrix();
    } else {
      AWARN << "TransformationMatrix File:" << file_name << " has no transform:rotation.";
      return false;
    }
  } catch (const YAML::Exception &e) {
    AERROR << file_name << " load failed. error:" << e.what();
    AERROR << "Please ensure param file is exist or format is correct";
    return false;
  }

  //fill trivial elements
  for (int i = 0; i < 3; i++) {
    (*transformation_matrix)(3, i) = 0.0;
  }
  (*transformation_matrix)(3, 3) = 1.0;
  return true;
}

bool load_matrix4d_from_file(const std::string &file_name, const std::string &key,
                             Eigen::Matrix4d *matrix) {
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

} // namespace traffic_light
} // namespace perception
} // namespace apollo
