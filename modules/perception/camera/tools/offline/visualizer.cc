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
#include "modules/perception/camera/tools/offline/visualizer.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/tools/offline/colormap.h"
#include "modules/perception/camera/tools/offline/keycode.h"

namespace apollo {
namespace perception {
namespace camera {

std::vector<cv::Scalar> colorlistobj = {magenta_color,  // last digit 0
                                        purple_color,   // last digit 1
                                        teal_color,     // last digit 2
                                        violet_color,   // last digit 3
                                        pink_color,     // last digit 4
                                        beige_color,    // last digit 5
                                        ivory_color,    // last digit 6
                                        olive_color,    // last digit 7
                                        maroon_color,   // last digit 8
                                        lime_color};    // last digit 9

std::map<base::LaneLinePositionType, cv::Scalar> colormapline = {
    {base::LaneLinePositionType::UNKNOWN, black_color},
    {base::LaneLinePositionType::FOURTH_LEFT, sky_blue_color},
    {base::LaneLinePositionType::THIRD_LEFT, dodger_blue_color},
    {base::LaneLinePositionType::ADJACENT_LEFT, blue_color},
    {base::LaneLinePositionType::EGO_LEFT, dark_blue_color},
    {base::LaneLinePositionType::EGO_CENTER, light_green_color},
    {base::LaneLinePositionType::EGO_RIGHT, red_color},
    {base::LaneLinePositionType::ADJACENT_RIGHT, coral_color},
    {base::LaneLinePositionType::THIRD_RIGHT, salmon_color},
    {base::LaneLinePositionType::FOURTH_RIGHT, orange_color},
    {base::LaneLinePositionType::OTHER, white_color},
    {base::LaneLinePositionType::CURB_LEFT, cyan_color},
    {base::LaneLinePositionType::CURB_RIGHT, yellow_color}};

Eigen::Matrix3d Camera2CarHomograph(Eigen::Matrix3d intrinsic,
                                    Eigen::Matrix4d extrinsic_camera2lidar,
                                    Eigen::Matrix4d extrinsic_lidar2imu,
                                    double pitch_adj) {
  AINFO << "intrinsic parameter of camera: " << intrinsic;
  AINFO << "extrinsic parameter of camera to lidar: " << extrinsic_camera2lidar;
  AINFO << "extrinsic parameter of lidar to imu: " << extrinsic_lidar2imu;
  // rotate 90 degree around z axis to make x point forward
  Eigen::Matrix4d Rz;
  Rz << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix4d extrinsic_camera2car;
  extrinsic_camera2car = extrinsic_camera2lidar * extrinsic_lidar2imu * Rz;
  // adjust pitch in camera coords
  Eigen::Matrix4d Rx;
  Rx << 1, 0, 0, 0, 0, cos(pitch_adj), -sin(pitch_adj), 0, 0, sin(pitch_adj),
      cos(pitch_adj), 0, 0, 0, 0, 1;
  extrinsic_camera2car = extrinsic_camera2car * Rx;
  AINFO << "extrinsic parameter from camera to car: " << extrinsic_camera2car;

  // compute the homography matrix, such that H [u, v, 1]' ~ [X_l, Y_l, 1]
  Eigen::Matrix3d K = intrinsic;
  Eigen::Matrix3d R = extrinsic_camera2car.block(0, 0, 3, 3);
  Eigen::Vector3d T = extrinsic_camera2car.block(0, 3, 3, 1);
  Eigen::Matrix3d H;

  H.block(0, 0, 3, 2) = (K * R.transpose()).block(0, 0, 3, 2);
  H.block(0, 2, 3, 1) = -K * R.transpose() * T;
  return H.inverse();
}

bool Visualizer::Init(const std::vector<std::string> &camera_names,
                      TransformServer *tf_server) {
  tf_server_ = tf_server;
  if (tf_server_ == nullptr) {
    AERROR << "tf_server is unavailable";
    return false;
  }
  last_timestamp_ = 0;
  small_h_ = static_cast<int>(image_height_ * scale_ratio_);
  small_w_ = static_cast<int>(image_width_ * scale_ratio_);
  world_h_ = 2 * small_h_;

  for (size_t i = 0; i < camera_names.size(); ++i) {
    camera_image_[camera_names[i]] =
        cv::Mat(small_h_, small_w_, CV_8UC3, black_color);
  }
  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, black_color);
  color_cipv_ = white_color;
  virtual_lane_color_ = white_color;
  return true;
}

bool Visualizer::Init_all_info_single_camera(
    const std::vector<std::string> &camera_names,
    const std::string &visual_camera,
    const EigenMap<std::string, Eigen::Matrix3f> &intrinsic_map,
    const EigenMap<std::string, Eigen::Matrix4d> &extrinsic_map,
    const Eigen::Matrix4d &ex_lidar2imu, const double pitch_adj_degree,
    const double yaw_adj_degree, const double roll_adj_degree,
    const int image_height, const int image_width) {
  image_height_ = image_height;
  image_width_ = image_width;
  intrinsic_map_ = intrinsic_map;
  extrinsic_map_ = extrinsic_map;
  ex_lidar2imu_ = ex_lidar2imu;
  camera_names_ = camera_names;

  last_timestamp_ = 0;
  small_h_ = static_cast<int>(image_height_ * scale_ratio_);
  small_w_ = static_cast<int>(image_width_ * scale_ratio_);
  world_h_ = 2 * small_h_;

  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, black_color);
  color_cipv_ = white_color;
  virtual_lane_color_ = green_color;

  draw_range_circle();

  AINFO << "world_h_: " << world_h_;
  AINFO << "wide_pixel_: " << wide_pixel_;
  AINFO << "small_h_: " << small_h_;
  AINFO << "small_w_: " << small_w_;

  visual_camera_ = visual_camera;
  // Set camera specific parameters
  for (auto camera_name : camera_names) {
    camera_image_[camera_name] =
        cv::Mat(small_h_, small_w_, CV_8UC3, black_color);
    camera_image_[camera_name] =
        cv::Mat(small_h_, small_w_, CV_8UC3, black_color);

    // 1. transform camera->lidar
    ex_camera2lidar_[camera_name] = extrinsic_map_.at(camera_name);
    AINFO << "ex_camera2lidar_ = " << extrinsic_map_.at(camera_name);

    AINFO << "ex_lidar2imu_ =" << ex_lidar2imu_;

    // 2. transform camera->lidar->imu
    ex_camera2imu_ = ex_lidar2imu_ * ex_camera2lidar_[camera_name];
    AINFO << "ex_camera2imu_ =" << ex_camera2imu_;

    // intrinsic camera parameter
    K_[camera_name] = intrinsic_map_.at(camera_name).cast<double>();
    AINFO << "intrinsic K_ =" << K_[camera_name];
    // homography_ground2image_.setIdentity();
    // homography_image2ground_.setIdentity();

    // rotate 90 degree around z axis to make x point forward
    // double imu_height = 0;  // imu height should be considred later
    ex_imu2car_ << 0, 1, 0, 0,  // cos(90), sin(90), 0,
        -1, 0, 0, 0,            // -sin(90),  cos(90), 0,
        0, 0, 1, 0,             // 0,              0, 1
        0, 0, 0, 1;

    // 3. transform camera->lidar->imu->car
    ex_camera2car_ = ex_imu2car_ * ex_camera2imu_;

    AINFO << "ex_camera2car_ =" << ex_camera2car_;

    // Adjust angle
    adjust_angles(camera_name, pitch_adj_degree, yaw_adj_degree,
                  roll_adj_degree);

    AINFO << "homography_image2ground_ ="
          << homography_image2ground_[camera_name];
    AINFO << "homography_ground2image_ ="
          << homography_ground2image_[camera_name];

    // compute FOV points
    p_fov_1_.x = 0;
    p_fov_1_.y = static_cast<int>(image_height_ * fov_cut_ratio_);

    p_fov_2_.x = image_width_ - 1;
    p_fov_2_.y = static_cast<int>(image_height_ * fov_cut_ratio_);

    p_fov_3_.x = 0;
    p_fov_3_.y = image_height_ - 1;

    p_fov_4_.x = image_width_ - 1;
    p_fov_4_.y = image_height_ - 1;

    AINFO << "p_fov_1_ =" << p_fov_1_;
    AINFO << "p_fov_2_ =" << p_fov_2_;
    AINFO << "p_fov_3_ =" << p_fov_3_;
    AINFO << "p_fov_4_ =" << p_fov_4_;

    vp1_[camera_name](0) = 1024.0;
    if (K_[camera_name](0, 0) >= 1.0) {
      vp1_[camera_name](1) =
          (image_width_ >> 1) * vp1_[camera_name](0) / K_[camera_name](0, 0);
    } else {
      AWARN << "Focal length (" << K_[camera_name](0, 0)
            << " in pixel) is incorrect. "
            << " Please check camera intrinsic parameters.";
      vp1_[camera_name](1) = vp1_[camera_name](0) * 0.25;
    }

    vp2_[camera_name](0) = vp1_[camera_name](0);
    vp2_[camera_name](1) = -vp1_[camera_name](1);

    AINFO << "vanishing point 1:" << vp1_[camera_name];
    AINFO << "vanishing point 2:" << vp2_[camera_name];

    pitch_adj_degree_[camera_name] = pitch_adj_degree;
    yaw_adj_degree_[camera_name] = yaw_adj_degree;
    roll_adj_degree_[camera_name] = roll_adj_degree;
  }

  reset_key();

  all_camera_recieved_ = 0x0;

  return true;
}

bool Visualizer::adjust_angles(const std::string &camera_name,
                               const double pitch_adj_degree,
                               const double yaw_adj_degree,
                               const double roll_adj_degree) {
  // Convert degree angles to radian angles
  double pitch_adj_radian = pitch_adj_degree * degree_to_radian_factor_;
  double yaw_adj_radian = yaw_adj_degree * degree_to_radian_factor_;
  double roll_adj_radian = roll_adj_degree * degree_to_radian_factor_;

  // We use "right handed ZYX" coordinate system for euler angles
  // adjust pitch yaw roll in camera coords
  // Remember that camera coordinate
  // (Z)----> X
  //  |
  //  |
  //  V
  //  Y
  Eigen::Matrix4d Rx;  // pitch
  Rx << 1, 0, 0, 0, 0, cos(pitch_adj_radian), -sin(pitch_adj_radian), 0, 0,
      sin(pitch_adj_radian), cos(pitch_adj_radian), 0, 0, 0, 0, 1;
  Eigen::Matrix4d Ry;  // yaw
  Ry << cos(yaw_adj_radian), 0, sin(yaw_adj_radian), 0, 0, 1, 0, 0,
      -sin(yaw_adj_radian), 0, cos(yaw_adj_radian), 0, 0, 0, 0, 1;
  Eigen::Matrix4d Rz;  // roll
  Rz << cos(roll_adj_radian), -sin(roll_adj_radian), 0, 0, sin(roll_adj_radian),
      cos(roll_adj_radian), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  adjusted_camera2car_ = ex_camera2car_ * Rz * Ry * Rx;
  AWARN << "adjusted_camera2car_: " << adjusted_camera2car_;

  // Get homography from projection matrix
  // ====
  // Version 1. Direct

  // compute the homography matrix, such that H [u, v, 1]' ~ [X_l, Y_l, 1]
  Eigen::Matrix3d R = adjusted_camera2car_.block(0, 0, 3, 3);
  Eigen::Vector3d T = adjusted_camera2car_.block(0, 3, 3, 1);
  Eigen::Matrix3d H;
  Eigen::Matrix3d H_inv;

  H.block(0, 0, 3, 2) = (K_[camera_name] * R.transpose()).block(0, 0, 3, 2);
  H.block(0, 2, 3, 1) = -K_[camera_name] * R.transpose() * T;
  H_inv = H.inverse();
  homography_ground2image_[camera_name] = H;
  homography_image2ground_[camera_name] = H_inv;

  // Version 2. Conceptual
  // ex_car2camera_ = adjusted_camera2car_.inverse();

  // // compute the homography matrix, such that H [u, v, 1]' ~ [X_l, Y_l, 1]
  // Eigen::Matrix3d R = ex_car2camera_.block(0, 0, 3, 3);
  // Eigen::Vector3d T = ex_car2camera_.block(0, 3, 3, 1);

  // projection_matrix_.setIdentity();
  // projection_matrix_.block(0, 0, 3, 3) = K_ * R;
  // projection_matrix_.block(0, 3, 3, 1) = K_ * T;

  // homography_ground2image_.block(0, 0, 3, 2)
  //   = projection_matrix_.block(0, 0, 3, 2);
  // homography_ground2image_.block(0, 2, 3, 1)
  //   = projection_matrix_.block(0, 3, 3, 1);

  // AINFO << "homography_ground2image_: ";
  // AINFO << homography_ground2image_;

  // homography_image2ground_ = homography_ground2image_.inverse();

  return true;
}

bool Visualizer::SetDirectory(const std::string &path) {
  if (!cyber::common::EnsureDirectory(path)) {
    return false;
  }
  const std::string command = "rm " + path + "/*.jpg";
  int ret = system(command.c_str());
  path_ = path;
  return ret == 0;
}

std::string Visualizer::type_to_string(const base::ObjectType type) {
  switch (type) {
    case base::ObjectType::UNKNOWN:
      return "UNKN";
    case base::ObjectType::UNKNOWN_MOVABLE:
      return "U_MO";
    case base::ObjectType::UNKNOWN_UNMOVABLE:
      return "UNMO";
    case base::ObjectType::PEDESTRIAN:
      return "PED";
    case base::ObjectType::BICYCLE:
      return "CYC";
    case base::ObjectType::VEHICLE:
      return "VEH";
    default:
      break;
  }
  return "WRNG";
}

std::string Visualizer::sub_type_to_string(const base::ObjectSubType type) {
  switch (type) {
    case base::ObjectSubType::UNKNOWN:
      return "UNKN";
    case base::ObjectSubType::UNKNOWN_MOVABLE:
      return "U_MO";
    case base::ObjectSubType::UNKNOWN_UNMOVABLE:
      return "UNMO";
    case base::ObjectSubType::CAR:
      return "CAR";
    case base::ObjectSubType::VAN:
      return "VAN";
    case base::ObjectSubType::TRUCK:
      return "TRUC";
    case base::ObjectSubType::BUS:
      return "BUS";
    case base::ObjectSubType::CYCLIST:
      return "CYC";
    case base::ObjectSubType::MOTORCYCLIST:
      return "MCYC";
    case base::ObjectSubType::TRICYCLIST:
      return "TCYC";
    case base::ObjectSubType::PEDESTRIAN:
      return "PED";
    case base::ObjectSubType::TRAFFICCONE:
      return "CONE";
    default:
      break;
  }
  return "WRNG";
}

bool Visualizer::reset_key() {
  use_class_color_ = true;
  capture_screen_ = false;
  capture_video_ = false;
  show_camera_box2d_ = true;
  show_camera_box3d_ = true;
  show_camera_bdv_ = true;
  show_virtual_egolane_ = true;
  show_radar_pc_ = true;
  show_fusion_ = false;
  show_associate_color_ = false;
  show_type_id_label_ = true;
  show_verbose_ = false;
  show_lane_count_ = 1;
  show_trajectory_ = true;
  show_vp_grid_ = true;  // show vanishing point and ground plane grid
  draw_lane_objects_ = true;
  show_box_ = true;
  show_velocity_ = false;
  show_polygon_ = true;
  show_text_ = false;
  show_help_text_ = false;
  manual_calibration_mode_ = false;
  show_homography_object_ = false;
  return true;
}

double Visualizer::regularize_angle(const double radian_angle) {
  if (radian_angle <= -M_PI) {
    return radian_angle + M_PI * 2.0;
  } else if (radian_angle > M_PI) {
    return radian_angle - M_PI * 2.0;
  }
  return radian_angle;
}

// ZYX Euler angles to quaternion
bool Visualizer::euler_to_quaternion(Eigen::Vector4d *quaternion,
                                     const double pitch_radian,
                                     const double yaw_radian,
                                     const double roll_radian) {
  // // Option 1. ZYX Euler to quortonian
  // double cy = cos(yaw_radian * 0.5);
  // double sy = sin(yaw_radian * 0.5);
  // double cp = cos(pitch_radian * 0.5);
  // double sp = sin(pitch_radian * 0.5);
  // double cr = cos(roll_radian * 0.5);
  // double sr = sin(roll_radian * 0.5);

  // quaternion(0) = sy * cp * cr - cy * sp * sr;  // Q.x
  // quaternion(1) = cy * sp * cr + sy * cp * sr;  // Q.y
  // quaternion(2) = cy * cp * sr - sy * sp * cr;  // Q.z
  // quaternion(3) = cy * cp * cr + sy * sp * sr;  // Q.w

  // AINFO << "fast quaternion(x, y, z, w): ("
  //       << quaternion(0) << ", "
  //       << quaternion(1) << ", "
  //       << quaternion(2) << ", "
  //       << quaternion(3) << ")";

  // Option 2. Rotation matrix to quaternion
  Eigen::Matrix3d Rx;  // pitch
  Rx << 1, 0, 0, 0, cos(pitch_radian), -sin(pitch_radian), 0, sin(pitch_radian),
      cos(pitch_radian);
  Eigen::Matrix3d Ry;  // roll
  Ry << cos(roll_radian), 0, sin(roll_radian), 0, 1, 0, -sin(roll_radian), 0,
      cos(roll_radian);
  Eigen::Matrix3d Rz;  // yaw
  Rz << cos(yaw_radian), -sin(yaw_radian), 0, sin(yaw_radian), cos(yaw_radian),
      0, 0, 0, 1;
  Eigen::Matrix3d R;
  R = Rz * Ry * Rx;
  AINFO << "Rotation matrix R: " << R;
  double qw = 0.5 * sqrt(1.0 + R(0, 0) + R(1, 1) + R(2, 2));
  if (fabs(qw) > 1.0e-6) {
    (*quaternion)(0) = 0.25 * (R(2, 1) - R(1, 2)) / qw;  // Q.x
    (*quaternion)(1) = 0.25 * (R(0, 2) - R(2, 0)) / qw;  // Q.y
    (*quaternion)(2) = 0.25 * (R(1, 0) - R(0, 1)) / qw;  // Q.z
    (*quaternion)(3) = qw;                               // Q.w
    AINFO << "quaternion(x, y, z, w): (" << (*quaternion)(0) << ", "
          << (*quaternion)(1) << ", " << (*quaternion)(2) << ", "
          << (*quaternion)(3) << ")";
  } else {
    double qx = 0.5 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
    if (fabs(qx) < 1.0e-6) {
      AWARN << "quaternion is degenerate qw: " << qw << "qx: " << qx;
      return false;
    }
    (*quaternion)(0) = qx;                               // Q.x
    (*quaternion)(1) = 0.25 * (R(0, 1) + R(1, 0)) / qx;  // Q.y
    (*quaternion)(2) = 0.25 * (R(0, 2) + R(2, 0)) / qx;  // Q.z
    (*quaternion)(3) = 0.25 * (R(2, 1) - R(1, 2)) / qx;  // Q.w
    AINFO << "second quaternion(x, y, z, w): (" << (*quaternion)(0) << ", "
          << (*quaternion)(1) << ", " << (*quaternion)(2) << ", "
          << (*quaternion)(3) << ")";
  }
  return true;
}

bool Visualizer::copy_backup_file(const std::string &filename) {
  static int index = 0;
  // int last_index = 0;
  // std::string files = filename + "*";
  // for (const auto &file : std::filesysfs::directory_iterator(files)) {
  //     AINFO << file.path() << std::endl;
  //     // Extract index
  //     last_index = get_index(file.path());
  // }
  // index = last_index;

  ++index;
  std::string yaml_bak_file = absl::StrCat(filename, "__", index);
  AINFO << "yaml_backup_file: " << yaml_bak_file;

  if (!cyber::common::Copy(filename, yaml_bak_file)) {
    AERROR << "Cannot backup the file: " << filename;
  } else {
    AINFO << "Backup file: " << filename << " saved successfully.";
  }

  return true;
}

bool Visualizer::save_extrinsic_in_yaml(const std::string &camera_name,
                                        const Eigen::Matrix4d &extrinsic,
                                        const Eigen::Vector4d &quaternion,
                                        const double pitch_radian,
                                        const double yaw_radian,
                                        const double roll_radian) {
  std::string yaml_file =
      FLAGS_obs_sensor_intrinsic_path + "/" + camera_name + "_extrinsics.yaml";

  copy_backup_file(yaml_file);

  AINFO << "extrinsic: " << extrinsic;

  // Save data
  // Option 1. Save using streaming
  std::ofstream y_file(yaml_file);

  y_file << "header:\n";
  y_file << "  seq: 0\n";
  y_file << "  stamp:\n";
  y_file << "    secs: 0\n";
  y_file << "    nsecs: 0\n";
  y_file << "  frame_id: velodyne128\n";
  y_file << "child_frame_id: %s\n", camera_name.c_str();
  y_file << "transform:\n";
  y_file << "  translation:\n";
  y_file << "    x: " << extrinsic(0, 3) << "\n";
  y_file << "    y: " << extrinsic(1, 3) << "\n";
  y_file << "    z: " << extrinsic(2, 3) << "\n";
  y_file << "  rotation:\n";
  y_file << "     x: " << quaternion(0) << "\n";
  y_file << "     y: " << quaternion(1) << "\n";
  y_file << "     z: " << quaternion(2) << "\n";
  y_file << "     w: " << quaternion(3) << "\n";
  y_file << "  euler_angles_degree:\n";
  y_file << "     pitch: " << pitch_radian * radian_to_degree_factor_ << "\n";
  y_file << "     yaw: " << yaw_radian * radian_to_degree_factor_ << "\n";
  y_file << "     roll: " << roll_radian * radian_to_degree_factor_ << "\n";
  // Option 2. Use YAML write function.
  // Alert! Couldn't find a library to save YAML node.
  // YAML::Node node = YAML::LoadFile(yaml_file);

  // try{
  //   if (node.IsNull()) {
  //     AINFO << "Load " << yaml_file << " failed! please check!";
  //     return false;
  //   }
  //   // Replace rotation only
  //   node["transform"]["rotation"]["x"].as<double>() = quaternion(0);
  //   node["transform"]["rotation"]["y"].as<double>() = quaternion(1);
  //   node["transform"]["rotation"]["z"].as<double>() = quaternion(2);
  //   node["transform"]["rotation"]["w"].as<double>() = quaternion(3);
  //
  //   node.SaveFile(yaml_file);
  //   if (node.IsNull()) {
  //     AINFO << "Save " << yaml_file << " failed! please check!";
  //     return false;
  //   }
  // } catch (YAML::InvalidNode &in) {
  //   AERROR << "load/save camera extrisic file " << yaml_file
  //          << " with error, YAML::InvalidNode exception";
  //   return false;
  // } catch (YAML::TypedBadConversion<double> &bc) {
  //   AERROR << "load camera extrisic file " << yaml_file
  //          << " with error, YAML::TypedBadConversion exception";
  //   return false;
  // } catch (YAML::Exception &e) {
  //   AERROR << "load camera extrisic file " << yaml_file
  //          << " with error, YAML exception:" << e.what();
  //   return false;
  // }

  return true;
}

bool Visualizer::save_manual_calibration_parameter(
    const std::string &camera_name, const double pitch_adj_degree,
    const double yaw_adj_degree, const double roll_adj_degree) {
  // Convert degree angles to radian angles
  double pitch_adj_radian = pitch_adj_degree * degree_to_radian_factor_;
  double yaw_adj_radian = yaw_adj_degree * degree_to_radian_factor_;
  double roll_adj_radian = roll_adj_degree * degree_to_radian_factor_;

  // Get current angle from extrinsics
  // ex_camera2lidar_ = extrinsic_map_.at(camera_name);
  Eigen::Matrix3d R = ex_camera2lidar_[camera_name].block(0, 0, 3, 3);

  double old_pitch_radian = regularize_angle(atan2(R(2, 1), R(2, 2)));
  double old_roll_radian = regularize_angle(
      atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))));
  double old_yaw_radian = regularize_angle(atan2(R(1, 0), R(0, 0)));
  AINFO << "Old pitch: " << old_pitch_radian * radian_to_degree_factor_;
  AINFO << "Old yaw: " << old_yaw_radian * radian_to_degree_factor_;
  AINFO << "Old roll: " << old_roll_radian * radian_to_degree_factor_;
  AINFO << "Adjusted pitch: " << pitch_adj_degree;
  AINFO << "Adjusted yaw: " << yaw_adj_degree;
  AINFO << "Adjusted roll: " << roll_adj_degree;

  // Convert value here since the coordinate system is different

  // Apply changed angles to each angle
  double new_pitch_radian =
      regularize_angle(old_pitch_radian + pitch_adj_radian);
  double new_yaw_radian = regularize_angle(old_yaw_radian - yaw_adj_radian);
  double new_roll_radian = regularize_angle(old_roll_radian + roll_adj_radian);

  AINFO << "New pitch: " << new_pitch_radian * radian_to_degree_factor_;
  AINFO << "New yaw: " << new_yaw_radian * radian_to_degree_factor_;
  AINFO << "New roll: " << new_roll_radian * radian_to_degree_factor_;

  Eigen::Vector4d quaternion;
  euler_to_quaternion(&quaternion, new_pitch_radian, new_yaw_radian,
                      new_roll_radian);
  AINFO << "Quaternion X: " << quaternion(0) << ", Y: " << quaternion(1)
        << ", Z: " << quaternion(2) << ", W: " << quaternion(3);
  // Save the file
  // Yaw and Roll are swapped.
  save_extrinsic_in_yaml(camera_name, ex_camera2lidar_[camera_name], quaternion,
                         new_pitch_radian, new_yaw_radian, new_roll_radian);

  return true;
}

bool Visualizer::key_handler(const std::string &camera_name, const int key) {
  AINFO << "Pressed Key: " << key;
  if (key <= 0) {
    return false;
  }
  switch (key) {
    case KEY_0:
      show_associate_color_ = !show_associate_color_;
      break;
    case KEY_2:
      show_camera_box2d_ = !show_camera_box2d_;
      break;
    case KEY_3:
      show_camera_box3d_ = !show_camera_box3d_;
      break;
    case KEY_UPPER_A:
    case KEY_LOWER_A:
      capture_video_ = !capture_video_;
      break;
    case KEY_UPPER_B:
    case KEY_LOWER_B:
      show_box_ = (show_box_ + 1) % 2;
      break;
    case KEY_UPPER_C:
    case KEY_LOWER_C:
      use_class_color_ = !use_class_color_;
      break;
    case KEY_UPPER_D:
    case KEY_LOWER_D:
      show_radar_pc_ = !show_radar_pc_;
      break;
    case KEY_UPPER_E:
    case KEY_LOWER_E:
      draw_lane_objects_ = !draw_lane_objects_;
      break;
    case KEY_UPPER_F:
    case KEY_LOWER_F:
      show_fusion_ = !show_fusion_;
      break;
    case KEY_UPPER_G:
    case KEY_LOWER_G:
      show_vp_grid_ = !show_vp_grid_;
      break;
    case KEY_UPPER_H:
    case KEY_LOWER_H:
      show_help_text_ = !show_help_text_;
      break;
    case KEY_UPPER_I:
    case KEY_LOWER_I:
      show_type_id_label_ = !show_type_id_label_;
      break;
    case KEY_UPPER_L:
    case KEY_LOWER_L:
      show_verbose_ = !show_verbose_;
      break;
    case KEY_UPPER_O:
    case KEY_LOWER_O:
      show_camera_bdv_ = !show_camera_bdv_;
      break;
    case KEY_UPPER_Q:
    case KEY_LOWER_Q:
      show_lane_count_ = (show_lane_count_ + 1) % 3;
      break;
    case KEY_UPPER_R:
    case KEY_LOWER_R:
      reset_key();
      break;
    case KEY_UPPER_S:
    case KEY_LOWER_S:
      capture_screen_ = true;
      break;
    case KEY_UPPER_T:
    case KEY_LOWER_T:
      show_trajectory_ = !show_trajectory_;
      break;
    case KEY_UPPER_V:
    case KEY_LOWER_V:
      show_velocity_ = (show_velocity_ + 1) % 2;
      break;
    case KEY_UP_NUM_LOCK_ON:
    case KEY_UP:
      if (manual_calibration_mode_ &&
          pitch_adj_degree_[camera_name] + 0.05 <= max_pitch_degree_) {
        pitch_adj_degree_[camera_name] -= 0.05;
      } else {
        visual_camera_ = camera_names_[0];
      }
      AINFO << "Current pitch: " << pitch_adj_degree_[camera_name];
      break;
    case KEY_DOWN_NUM_LOCK_ON:
    case KEY_DOWN:
      if (manual_calibration_mode_ &&
          pitch_adj_degree_[camera_name] - 0.05 >= min_pitch_degree_) {
        pitch_adj_degree_[camera_name] += 0.05;
      } else {
        visual_camera_ = camera_names_[1];
      }
      AINFO << "Current pitch: " << pitch_adj_degree_[camera_name];
      break;
    case KEY_RIGHT_NUM_LOCK_ON:
    case KEY_RIGHT:
      if (manual_calibration_mode_ &&
          yaw_adj_degree_[camera_name] + 0.05 <= max_yaw_degree_) {
        yaw_adj_degree_[camera_name] -= 0.05;
      }
      AINFO << "Current yaw: " << yaw_adj_degree_[camera_name];
      break;
    case KEY_LEFT_NUM_LOCK_ON:
    case KEY_LEFT:
      if (manual_calibration_mode_ &&
          yaw_adj_degree_[camera_name] - 0.05 >= min_yaw_degree_) {
        yaw_adj_degree_[camera_name] += 0.05;
      }
      AINFO << "Current yaw: " << yaw_adj_degree_[camera_name];
      break;
    case KEY_SHIFT_LEFT_NUM_LOCK_ON:
    case KEY_SHIFT_RIGHT:
      if (manual_calibration_mode_ &&
          roll_adj_degree_[camera_name] + 0.05 <= max_roll_degree_) {
        roll_adj_degree_[camera_name] -= 0.05;
      }
      AINFO << "Current roll: " << roll_adj_degree_[camera_name];
      break;
    case KEY_SHIFT_RIGHT_NUM_LOCK_ON:
    case KEY_SHIFT_LEFT:
      if (manual_calibration_mode_ &&
          roll_adj_degree_[camera_name] - 0.05 >= min_roll_degree_) {
        roll_adj_degree_[camera_name] += 0.05;
      }
      AINFO << "Current roll: " << roll_adj_degree_[camera_name];
      break;
    case KEY_CTRL_S_NUM_LOCK_ON:
    case KEY_CTRL_S:
      if (manual_calibration_mode_) {
        save_manual_calibration_parameter(
            visual_camera_, pitch_adj_degree_[camera_name],
            yaw_adj_degree_[camera_name], roll_adj_degree_[camera_name]);
        AINFO << "Saved calibration parameters(pyr): ("
              << pitch_adj_degree_[camera_name] << ", "
              << yaw_adj_degree_[camera_name] << ", "
              << roll_adj_degree_[camera_name] << ")";
      }
      break;
    case KEY_ALT_C_NUM_LOCK_ON:
    case KEY_ALT_C:
      manual_calibration_mode_ = !manual_calibration_mode_;

    default:
      break;
  }

  help_str_ = "H: show help";
  if (show_help_text_) {
    absl::StrAppend(&help_str_, " (ON)\nR: reset matrxi\nB: show box");
    if (show_box_) {
      absl::StrAppend(&help_str_, "(ON)");
    }
    absl::StrAppend(&help_str_, "\nV: show velocity");
    if (show_velocity_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nC: use class color");
    if (use_class_color_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nS: capture screen", "\nA: capture video",
                    "\nI: show type id label");
    if (show_type_id_label_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nQ: show lane");
    if (show_lane_count_ > 0) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nE: draw lane objects");
    if (draw_lane_objects_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nF: show fusion");
    if (show_fusion_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nD: show radar pc");
    if (show_radar_pc_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nT: show trajectory");
    if (show_trajectory_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nO: show camera bdv");
    if (show_camera_bdv_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\n2: show camera box2d");
    if (show_camera_box2d_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\n3: show camera box3d");
    if (show_camera_box3d_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\n0: show associate color");
    if (show_associate_color_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_,
                    "\nG: show vanishing point and ground plane grid");
    if (show_vp_grid_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
    absl::StrAppend(&help_str_, "\nT: show verbose");
    if (show_verbose_) {
      absl::StrAppend(&help_str_, " (ON)");
    }
  }
  switch (key) {
    case KEY_UP_NUM_LOCK_ON:
    case KEY_DOWN_NUM_LOCK_ON:
    case KEY_RIGHT_NUM_LOCK_ON:
    case KEY_LEFT_NUM_LOCK_ON:
    case KEY_SHIFT_LEFT_NUM_LOCK_ON:
    case KEY_SHIFT_RIGHT_NUM_LOCK_ON:
    case KEY_UP:
    case KEY_LEFT:
    case KEY_RIGHT:
    case KEY_DOWN:
    case KEY_SHIFT_LEFT:
    case KEY_SHIFT_RIGHT:
      if (manual_calibration_mode_) {
        adjust_angles(camera_name, pitch_adj_degree_[camera_name],
                      yaw_adj_degree_[camera_name],
                      roll_adj_degree_[camera_name]);
        if (show_help_text_) {
          absl::StrAppend(&help_str_,
                          "\nAdjusted Pitch: ", pitch_adj_degree_[camera_name]);
          absl::StrAppend(&help_str_,
                          "\nAdjusted Yaw: ", yaw_adj_degree_[camera_name]);
          absl::StrAppend(&help_str_,
                          "\nAdjusted Roll: ", roll_adj_degree_[camera_name]);
        }
      }
  }
  return true;
}

// Draw trajectory of each object
bool Visualizer::DrawTrajectories(const base::ObjectPtr &object,
                                  const base::MotionBufferPtr motion_buffer) {
  if (object->drop_num == 0 || motion_buffer == nullptr ||
      motion_buffer->size() == 0) {
    return false;
  }
  std::size_t count = std::min(object->drop_num, motion_buffer->size());

  Eigen::Vector4f start_point;
  start_point << static_cast<float>(object->drops[0](0)),
      static_cast<float>(object->drops[0](1)), 0, 1;
  start_point = (*motion_buffer)[0].motion * start_point;
  cv::circle(world_image_, world_point_to_bigimg(start_point), 3, gray_color);

  for (size_t i = 1; i < count; i++) {
    Eigen::Vector4f end_point;
    end_point << static_cast<float>(object->drops[i](0)),
        static_cast<float>(object->drops[i](1)), 0, 1;
    cv::circle(world_image_, world_point_to_bigimg(end_point), 3, gray_color);
    cv::line(world_image_, world_point_to_bigimg(start_point),
             world_point_to_bigimg(end_point), gray_color,
             trajectory_line_thickness_);
    start_point = end_point;
  }
  return true;
}

void Visualizer::Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  Eigen::Affine3d pose;
  if (!tf_server_->QueryPos(frame.timestamp, &pose)) {
    pose.setIdentity();
  }
  Eigen::Affine3d lidar2novatel;
  if (!tf_server_->QueryTransform("velodyne128", "novatel", &lidar2novatel)) {
    AWARN << "Failed to query transform from lidar to ground.";
    return;
  }
  Eigen::Affine3d lidar2world = pose * lidar2novatel;
  Eigen::Affine3d world2lidar = lidar2world.inverse();
  for (const auto &object : frame.tracked_objects) {
    base::RectF rect(object->camera_supplement.box);
    cv::Rect r(static_cast<int>(rect.x), static_cast<int>(rect.y),
               static_cast<int>(rect.width), static_cast<int>(rect.height));
    cv::Scalar color = colorlistobj[object->track_id % colorlistobj.size()];

    cv::rectangle(image, r, color, 2);
    cv::putText(image, std::to_string(object->track_id),
                cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y)),
                cv::FONT_HERSHEY_DUPLEX, 1, red_color, 2);
    Eigen::Vector3d theta;
    theta << cos(object->theta), sin(object->theta), 0;
    theta = world2lidar.linear() * theta;
    float yaw = static_cast<float>(atan2(theta(1), theta(0)));
    Eigen::Matrix2d rotate;
    rotate << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

    Eigen::Vector3d pos;
    pos << object->center(0), object->center(1), object->center(2);
    pos = world2lidar * pos;
    Eigen::Vector2d pos_2d;
    pos_2d << pos(0), pos(1);
    Eigen::Vector3d v;
    v << object->velocity(0), object->velocity(1), object->velocity(2);
    v = world2lidar.linear() * v;
    Eigen::Vector2d v_2d;
    v_2d << v(0) + pos_2d(0), v(1) + pos_2d(1);
    Eigen::Vector2d p1;
    p1 << object->size(0) * 0.5, object->size(1) * 0.5;
    p1 = rotate * p1 + pos_2d;
    Eigen::Vector2d p2;
    p2 << -object->size(0) * 0.5, object->size(1) * 0.5;
    p2 = rotate * p2 + pos_2d;
    Eigen::Vector2d p3;
    p3 << -object->size(0) * 0.5, -object->size(1) * 0.5;
    p3 = rotate * p3 + pos_2d;
    Eigen::Vector2d p4;
    p4 << object->size(0) * 0.5, -object->size(1) * 0.5;
    p4 = rotate * p4 + pos_2d;

    if (object->b_cipv) {
      cv::line(world_image_, world_point_to_bigimg(p1),
               world_point_to_bigimg(p2), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p2),
               world_point_to_bigimg(p3), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p3),
               world_point_to_bigimg(p4), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p4),
               world_point_to_bigimg(p1), color_cipv_, cipv_line_thickness_);
      // cv::line(world_image_, world_point_to_bigimg(pos_2d),
      //          world_point_to_bigimg(v_2d), color_cipv_,
      //          cipv_line_thickness_);
    }
    cv::line(world_image_, world_point_to_bigimg(p1), world_point_to_bigimg(p2),
             color_cipv_, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p2), world_point_to_bigimg(p3),
             color_cipv_, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p3), world_point_to_bigimg(p4),
             color_cipv_, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p4), world_point_to_bigimg(p1),
             color_cipv_, line_thickness_);
    // cv::line(world_image_, world_point_to_bigimg(pos_2d),
    //          world_point_to_bigimg(v_2d), color_cipv_, line_thickness_);
  }
  last_timestamp_ = frame.timestamp;
  cv::resize(image, camera_image_[frame.data_provider->sensor_name()],
             cv::Size(small_w_, small_h_));
}

void Visualizer::ShowResult(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  std::string camera_name = frame.data_provider->sensor_name();

  if (frame.timestamp - last_timestamp_ > 0.02) {
    draw_selected_image_boundary(small_w_, small_h_,
                                 &(camera_image_[visual_camera_]));
    cv::Mat bigimg(world_h_, small_w_ + wide_pixel_, CV_8UC3);
    camera_image_[camera_name].copyTo(
        bigimg(cv::Rect(0, 0, small_w_, small_h_)));
    camera_image_[camera_name].copyTo(
        bigimg(cv::Rect(0, small_h_, small_w_, small_h_)));
    world_image_.copyTo(bigimg(cv::Rect(small_w_, 0, wide_pixel_, world_h_)));
    if (write_out_img_) {
      char path[1000];
      static int k = 0;
      snprintf(path, sizeof(path), "%s/%06d.jpg", path_.c_str(), k++);
      AINFO << "A snapshot of visualizer saved at " << path;
      cv::imwrite(path, bigimg);
    }

    if (cv_imshow_img_) {
      cv::namedWindow("Apollo Visualizer", CV_WINDOW_NORMAL);
      cv::setWindowProperty("Apollo Visualizer", CV_WND_PROP_FULLSCREEN,
                            CV_WINDOW_FULLSCREEN);
      cv::imshow("Apollo Visualizer", bigimg);
      int key = cvWaitKey(30);
      key_handler(camera_name, key);
    }
    world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, black_color);
    draw_range_circle();
  }

  cv::putText(image, camera_name, cv::Point(10, 50), cv::FONT_HERSHEY_DUPLEX,
              1.3, red_color, 3);
  cv::putText(image, absl::StrCat("frame #: ", frame.frame_id),
              cv::Point(10, 100), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
  Draw2Dand3D(image, frame);
}

void Visualizer::Draw2Dand3D_all_info_single_camera(
    const std::string &camera_name, const cv::Mat &img,
    const CameraFrame &frame, const Eigen::Matrix3d &intrinsic,
    const Eigen::Matrix4d &extrinsic, const Eigen::Affine3d &world2camera,
    const base::MotionBufferPtr motion_buffer) {
  cv::Mat image_2D = img.clone();  // All clone should be replaced with global

  // plot FOV

  // cv::line(img2, p_fov_1_, p_fov_2_, white_color, 2);
  // cv::line(img2, p_fov_1_, p_fov_3_, white_color, 2);
  // cv::line(img2, p_fov_2_, p_fov_4_, white_color, 2);
  // cv::line(world_image_, world_point_to_bigimg(image2ground(p_fov_1_)),
  //          world_point_to_bigimg(image2ground(p_fov_2_)), white_color, 2);
  // cv::line(world_image_, world_point_to_bigimg(image2ground(p_fov_1_)),
  //          world_point_to_bigimg(image2ground(p_fov_3_)), white_color, 2);
  // cv::line(world_image_, world_point_to_bigimg(image2ground(p_fov_2_)),
  //          world_point_to_bigimg(image2ground(p_fov_4_)), white_color, 2);

  // cv::line(img2, p_fov_2_, p_fov_4_, white_color, 2);
  // cv::line(world_image_, world_point_to_bigimg(image2ground(p_fov_1_)),
  //          world_point_to_bigimg(image2ground(p_fov_2_)), white_color, 2);
  // cv::line(world_image_, world_point_to_bigimg(image2ground(p_fov_1_)),

  if (show_vp_grid_) {
    cv::line(image_2D, ground2image(camera_name, vp1_[camera_name]),
             ground2image(camera_name, vp2_[camera_name]), white_color, 2);
  }
  // plot laneline on image and ground plane
  if (show_lane_count_ > 0) {  // Do now show lane line
    for (const auto &object : frame.lane_objects) {
      cv::Scalar lane_color = colormapline[object.pos_type];
      if (show_lane_count_ == 1) {  // show inlier points
        cv::Point p_prev;
        p_prev.x = static_cast<int>(object.curve_image_point_set[0].x);
        p_prev.y = static_cast<int>(object.curve_image_point_set[0].y);
        Eigen::Vector2d p_prev_ground = image2ground(camera_name, p_prev);
        for (unsigned i = 1; i < object.curve_image_point_set.size(); i++) {
          cv::Point p_cur;
          p_cur.x = static_cast<int>(object.curve_image_point_set[i].x);
          p_cur.y = static_cast<int>(object.curve_image_point_set[i].y);
          Eigen::Vector2d p_cur_ground = image2ground(camera_name, p_cur);

          if (p_cur.x >= 0 && p_cur.y >= 0 && p_prev.x >= 0 && p_prev.y >= 0 &&
              p_cur.x < image_width_ && p_cur.y < image_height_ &&
              p_prev.x < image_width_ && p_prev.y < image_height_) {
            cv::line(image_2D, p_prev, p_cur, lane_color, line_thickness_);
          }
          cv::line(world_image_, world_point_to_bigimg(p_prev_ground),
                   world_point_to_bigimg(p_cur_ground), lane_color, 2);
          p_prev = p_cur;
          p_prev_ground = p_cur_ground;
        }
      } else if (show_lane_count_ == 2) {  // Show fitted curve
        base::LaneLineCubicCurve curve_coord = object.curve_car_coord;
        Eigen::Vector2d p_prev_ground;
        float step =
            std::max(std::abs(curve_coord.x_end - curve_coord.x_start) /
                         static_cast<float>(lane_step_num_),
                     3.0f);
        float x = curve_coord.x_start;
        p_prev_ground(0) = x;
        p_prev_ground(1) = curve_coord.a * x * x * x + curve_coord.b * x * x +
                           curve_coord.c * x + curve_coord.d;
        cv::Point p_prev = ground2image(camera_name, p_prev_ground);
        x += step;
        for (unsigned int i = 0; x < curve_coord.x_end && i < lane_step_num_;
             x += step, i++) {
          Eigen::Vector2d p_cur_ground;
          p_cur_ground(0) = x;
          p_cur_ground(1) = curve_coord.a * x * x * x + curve_coord.b * x * x +
                            curve_coord.c * x + curve_coord.d;
          cv::Point p_cur = ground2image(camera_name, p_cur_ground);
          if (p_cur.x >= 0 && p_cur.y >= 0 && p_prev.x >= 0 && p_prev.y >= 0 &&
              p_cur.x < image_width_ && p_cur.y < image_height_ &&
              p_prev.x < image_width_ && p_prev.y < image_height_) {
            cv::line(image_2D, p_prev, p_cur, lane_color, line_thickness_);
          }
          cv::line(world_image_, world_point_to_bigimg(p_prev_ground),
                   world_point_to_bigimg(p_cur_ground), lane_color, 2);
          p_prev = p_cur;
          p_prev_ground = p_cur_ground;
        }
      }
    }
  }

  // Draw objects
  for (const auto &object : frame.tracked_objects) {
    // plot 2D box on image_2D
    base::RectF rect(object->camera_supplement.box);
    cv::Rect r(static_cast<int>(rect.x), static_cast<int>(rect.y),
               static_cast<int>(rect.width), static_cast<int>(rect.height));
    cv::Scalar color = colorlistobj[object->track_id % colorlistobj.size()];

    cv::putText(
        image_2D,
        // type_to_string(object->type) + "->" +
        sub_type_to_string(object->sub_type),
        cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y) + 30),
        cv::FONT_HERSHEY_DUPLEX, 1, red_color, 1);

    // compute 8 vetices in camera coodinates
    Eigen::Vector3d pos;

    ADEBUG << "object->track_id: " << object->track_id;
    // Draw 3D position using homography or direct distance from CNN
    double theta_ray;
    double theta;
    cv::Point c_2D;
    Eigen::Vector2d c_2D_l;
    if (show_homography_object_) {
      pos << object->camera_supplement.local_center(0),
          object->camera_supplement.local_center(1),
          object->camera_supplement.local_center(2);
      ADEBUG << "Camera pos: (" << pos(0) << ", " << pos(1) << ", " << pos(2)
             << ")";
      theta_ray = atan2(pos(0), pos(2));
      theta = object->camera_supplement.alpha + theta_ray;
      // compute obstacle center in lidar ground
      c_2D.x = static_cast<int>(rect.x + rect.width / 2);
      c_2D.y = static_cast<int>(rect.y + rect.height);
      ADEBUG << "Image Footprint c_2D: (" << c_2D.x << ", " << c_2D.y << ")";
      c_2D_l = image2ground(camera_name, c_2D);
      ADEBUG << "Image Footprint position: (" << c_2D_l(0) << ", " << c_2D_l(1)
             << ")";
    } else {
      pos = world2camera * object->center;
      theta_ray = atan2(pos(0), pos(2));
      theta = object->camera_supplement.alpha + theta_ray;
      ADEBUG << "Direct pos: (" << pos(0) << ", " << pos(1) << ", " << pos(2)
             << ")";

      // compute obstacle center in lidar ground
      c_2D_l(0) = pos(2);
      c_2D_l(1) = -pos(0);
      ADEBUG << "Direct center position: (" << c_2D_l(0) << ", " << c_2D_l(1)
             << ")";
    }
    ADEBUG << "theta_ray: " << theta_ray * 180 / M_PI << " degree";
    ADEBUG << "object->camera_supplement.alpha: "
           << object->camera_supplement.alpha * 180 / M_PI << " degree";
    ADEBUG << "theta: " << theta * 180 / M_PI << " = "
           << object->camera_supplement.alpha * 180 / M_PI << " + "
           << theta_ray * 180 / M_PI;

    float distance =
        static_cast<float>(sqrt(c_2D_l(0) * c_2D_l(0) + c_2D_l(1) * c_2D_l(1)));
    char dist_string[100];
    snprintf(dist_string, sizeof(dist_string), "%.1fm", distance);
    // Show distance
    cv::putText(
        image_2D, dist_string,
        cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y - 10)),
        cv::FONT_HERSHEY_DUPLEX, 1, lime_color, 2);

    if (show_camera_box3d_) {
      Eigen::Matrix3d rotate_ry;
      rotate_ry << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0,
          cos(theta);
      std::vector<Eigen::Vector3d> p(8);
      std::vector<Eigen::Vector3d> proj(8);
      std::vector<cv::Point> p_proj(8);
      p[0] << object->size(0) * 0.5, object->size(2) * 0.5,
          object->size(1) * 0.5;
      p[1] << -object->size(0) * 0.5, object->size(2) * 0.5,
          object->size(1) * 0.5;
      p[2] << -object->size(0) * 0.5, object->size(2) * 0.5,
          -object->size(1) * 0.5;
      p[3] << object->size(0) * 0.5, object->size(2) * 0.5,
          -object->size(1) * 0.5;
      p[4] << object->size(0) * 0.5, -object->size(2) * 0.5,
          object->size(1) * 0.5;
      p[5] << -object->size(0) * 0.5, -object->size(2) * 0.5,
          object->size(1) * 0.5;
      p[6] << -object->size(0) * 0.5, -object->size(2) * 0.5,
          -object->size(1) * 0.5;
      p[7] << object->size(0) * 0.5, -object->size(2) * 0.5,
          -object->size(1) * 0.5;

      for (uint i = 0; i < p.size(); i++) {
        proj[i] = intrinsic * (rotate_ry * p[i] + pos);
        if (fabs(p[i](2)) > std::numeric_limits<double>::min()) {
          p_proj[i].x = static_cast<int>(proj[i](0) / proj[i](2));
          p_proj[i].y = static_cast<int>(proj[i](1) / proj[i](2));
        }
      }
      if (object->b_cipv) {
        cv::line(image_2D, p_proj[0], p_proj[1], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[1], p_proj[2], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[2], p_proj[3], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[3], p_proj[0], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[4], p_proj[5], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[5], p_proj[6], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[6], p_proj[7], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[7], p_proj[4], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[0], p_proj[4], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[1], p_proj[5], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[2], p_proj[6], color_cipv_,
                 cipv_line_thickness_);
        cv::line(image_2D, p_proj[3], p_proj[7], color_cipv_,
                 cipv_line_thickness_);
      }

      cv::line(image_2D, p_proj[0], p_proj[1], color, line_thickness_);
      cv::line(image_2D, p_proj[1], p_proj[2], color, line_thickness_);
      cv::line(image_2D, p_proj[2], p_proj[3], color, line_thickness_);
      cv::line(image_2D, p_proj[3], p_proj[0], color, line_thickness_);
      cv::line(image_2D, p_proj[4], p_proj[5], color, line_thickness_);
      cv::line(image_2D, p_proj[5], p_proj[6], color, line_thickness_);
      cv::line(image_2D, p_proj[6], p_proj[7], color, line_thickness_);
      cv::line(image_2D, p_proj[7], p_proj[4], color, line_thickness_);
      cv::line(image_2D, p_proj[0], p_proj[4], color, line_thickness_);
      cv::line(image_2D, p_proj[1], p_proj[5], color, line_thickness_);
      cv::line(image_2D, p_proj[2], p_proj[6], color, line_thickness_);
      cv::line(image_2D, p_proj[3], p_proj[7], color, line_thickness_);
    } else {
      if (object->b_cipv) {
        cv::rectangle(image_2D, r, color_cipv_, cipv_line_thickness_);
      }
      cv::rectangle(image_2D, r, color, 2);
    }

    // plot obstacles on ground plane in lidar coordinates
    Eigen::Matrix2d rotate_rz;
    theta = theta - M_PI_2;
    rotate_rz << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d p1;
    p1 << object->size(0) * 0.5, object->size(1) * 0.5;
    p1 = rotate_rz * p1 + c_2D_l;
    Eigen::Vector2d p2;
    p2 << -object->size(0) * 0.5, object->size(1) * 0.5;
    p2 = rotate_rz * p2 + c_2D_l;
    Eigen::Vector2d p3;
    p3 << -object->size(0) * 0.5, -object->size(1) * 0.5;
    p3 = rotate_rz * p3 + c_2D_l;
    Eigen::Vector2d p4;
    p4 << object->size(0) * 0.5, -object->size(1) * 0.5;
    p4 = rotate_rz * p4 + c_2D_l;

    Eigen::Vector2d pos_2d;
    pos_2d << c_2D_l(0), c_2D_l(1);
    Eigen::Vector3d v;
    v << object->velocity(0), object->velocity(1), object->velocity(2);
    v = world2camera.linear() * v;
    Eigen::Vector2d v_2d;
    v_2d << v(0) + pos_2d(0), v(1) + pos_2d(1);

    AINFO << "v.norm: " << v.norm();
    if (show_trajectory_ && motion_buffer != nullptr &&
        motion_buffer->size() > 0 && v.norm() > speed_limit_) {
      DrawTrajectories(object, motion_buffer);
    }
    if (object->b_cipv) {
      cv::line(world_image_, world_point_to_bigimg(p1),
               world_point_to_bigimg(p2), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p2),
               world_point_to_bigimg(p3), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p3),
               world_point_to_bigimg(p4), color_cipv_, cipv_line_thickness_);
      cv::line(world_image_, world_point_to_bigimg(p4),
               world_point_to_bigimg(p1), color_cipv_, cipv_line_thickness_);
      // cv::line(world_image_, world_point_to_bigimg(pos_2d),
      //          world_point_to_bigimg(v_2d), color_cipv_,
      //          cipv_line_thickness_);
    }
    cv::line(world_image_, world_point_to_bigimg(p1), world_point_to_bigimg(p2),
             color, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p2), world_point_to_bigimg(p3),
             color, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p3), world_point_to_bigimg(p4),
             color, line_thickness_);
    cv::line(world_image_, world_point_to_bigimg(p4), world_point_to_bigimg(p1),
             color, line_thickness_);
    // cv::line(world_image_, world_point_to_bigimg(pos_2d),
    //          world_point_to_bigimg(v_2d), color, line_thickness_);
  }

  // Draw virtual ego lanes
  if (show_virtual_egolane_) {
    EgoLane virtual_egolane_ground;
    virtual_egolane_ground.left_line.line_point.clear();
    virtual_egolane_ground.right_line.line_point.clear();
    CipvOptions cipv_options;
    if (motion_buffer == nullptr || motion_buffer->size() == 0) {
      AWARN << "motion_buffer_ is empty";
      cipv_options.velocity = 5.0f;
      cipv_options.yaw_rate = 0.0f;
    } else {
      cipv_options.velocity = motion_buffer->back().velocity;
      cipv_options.yaw_rate = motion_buffer->back().yaw_rate;
    }
    Cipv::MakeVirtualEgoLaneFromYawRate(
        cipv_options.yaw_rate, cipv_options.velocity,
        kMaxVehicleWidthInMeter * 0.5, &virtual_egolane_ground.left_line,
        &virtual_egolane_ground.right_line);
    // Left ego lane
    Eigen::Vector2d p_prev_ground;
    p_prev_ground(0) = virtual_egolane_ground.left_line.line_point[0](0);
    p_prev_ground(1) = virtual_egolane_ground.left_line.line_point[0](1);
    cv::Point p_prev = ground2image(camera_name, p_prev_ground);
    for (unsigned i = 1; i < virtual_egolane_ground.left_line.line_point.size();
         i++) {
      Eigen::Vector2d p_cur_ground;
      p_cur_ground(0) = virtual_egolane_ground.left_line.line_point[i](0);
      p_cur_ground(1) = virtual_egolane_ground.left_line.line_point[i](1);
      cv::Point p_cur = ground2image(camera_name, p_cur_ground);
      if (p_cur.x >= 0 && p_cur.y >= 0 && p_prev.x >= 0 && p_prev.y >= 0 &&
          p_cur.x < image_width_ && p_cur.y < image_height_ &&
          p_prev.x < image_width_ && p_prev.y < image_height_) {
        cv::line(image_2D, p_prev, p_cur, virtual_lane_color_, line_thickness_);
      }
      cv::line(world_image_, world_point_to_bigimg(p_prev_ground),
               world_point_to_bigimg(p_cur_ground), virtual_lane_color_, 2);
      p_prev = p_cur;
      p_prev_ground = p_cur_ground;
    }

    // Right ego lane
    p_prev_ground(0) = virtual_egolane_ground.right_line.line_point[0](0);
    p_prev_ground(1) = virtual_egolane_ground.right_line.line_point[0](1);
    p_prev = ground2image(camera_name, p_prev_ground);
    for (unsigned i = 1;
         i < virtual_egolane_ground.right_line.line_point.size(); i++) {
      Eigen::Vector2d p_cur_ground;
      p_cur_ground(0) = virtual_egolane_ground.right_line.line_point[i](0);
      p_cur_ground(1) = virtual_egolane_ground.right_line.line_point[i](1);
      cv::Point p_cur = ground2image(camera_name, p_cur_ground);

      if (p_cur.x >= 0 && p_cur.y >= 0 && p_prev.x >= 0 && p_prev.y >= 0 &&
          p_cur.x < image_width_ && p_cur.y < image_height_ &&
          p_prev.x < image_width_ && p_prev.y < image_height_) {
        cv::line(image_2D, p_prev, p_cur, virtual_lane_color_, line_thickness_);
      }
      cv::line(world_image_, world_point_to_bigimg(p_prev_ground),
               world_point_to_bigimg(p_cur_ground), virtual_lane_color_, 2);
      p_prev = p_cur;
      p_prev_ground = p_cur_ground;
    }
  }

  last_timestamp_ = frame.timestamp;
  camera_image_[frame.data_provider->sensor_name()] = image_2D;
  cv::resize(image_2D, camera_image_[frame.data_provider->sensor_name()],
             cv::Size(small_w_, small_h_));
}

void Visualizer::ShowResult_all_info_single_camera(
    const cv::Mat &img, const CameraFrame &frame,
    const base::MotionBufferPtr motion_buffer,
    const Eigen::Affine3d &world2camera) {
  if (frame.timestamp - last_timestamp_ < 0.02) return;

  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, black_color);

  // draw results on visulization panel
  int line_pos = 0;
  cv::Mat image = img.clone();
  std::string camera_name = frame.data_provider->sensor_name();
  if (manual_calibration_mode_) {
    line_pos += 50;
    cv::putText(image,
                "Manual Calibration: Pitch(up/down) Yaw(left/right) "
                "Roll(SH+left/right)",
                cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3,
                red_color, 3);
  }
  line_pos += 50;
  cv::putText(image, camera_name, cv::Point(10, line_pos),
              cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
  line_pos += 50;
  cv::putText(image, absl::StrCat("frame id: ", frame.frame_id),
              cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color,
              3);
  line_pos += 50;
  if (motion_buffer != nullptr) {
    cv::putText(
        image, absl::StrCat("yaw rate: ", motion_buffer->back().yaw_rate),
        cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
    line_pos += 50;
    cv::putText(
        image, absl::StrCat("pitch rate: ", motion_buffer->back().pitch_rate),
        cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
    line_pos += 50;
    cv::putText(
        image, absl::StrCat("roll rate: ", motion_buffer->back().roll_rate),
        cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
    line_pos += 50;
    cv::putText(
        image, absl::StrCat("velocity: ", motion_buffer->back().velocity),
        cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3, red_color, 3);
  }

  // plot predicted vanishing point
  if (frame.pred_vpt.size() > 0) {
    // Option 1. Show both x and y
    // cv::circle(image,
    //            cv::Point(static_cast<int>(frame.pred_vpt[0]),
    //            static_cast<int>(frame.pred_vpt[1])), 5, dark_green_color, 3);
    // Option 2. Show height/2 (x) and y
    cv::circle(image,
               cv::Point(static_cast<int>(image_width_ >> 1),
                         static_cast<int>(frame.pred_vpt[1])),
               5, dark_green_color, 3);
  }

  for (const auto &object : frame.tracked_objects) {
    if (object->b_cipv) {
      line_pos += 50;
      cv::putText(image, absl::StrCat("CIPV: ", object->track_id),
                  cv::Point(10, line_pos), cv::FONT_HERSHEY_DUPLEX, 1.3,
                  red_color, 3);
    }
  }

  if (intrinsic_map_.find(camera_name) != intrinsic_map_.end() &&
      extrinsic_map_.find(camera_name) != extrinsic_map_.end()) {
    Draw2Dand3D_all_info_single_camera(
        camera_name, image, frame,
        intrinsic_map_.at(camera_name).cast<double>(),
        extrinsic_map_.at(camera_name), world2camera, motion_buffer);
  } else {
    AERROR << "Failed to find necessuary intrinsic or extrinsic params.";
  }

  // copy visual results into visualization panel
  if (cv_imshow_img_) {
    if (camera_name == camera_names_[0]) {
      all_camera_recieved_ |= 0x1;
    } else if (camera_name == camera_names_[1]) {
      all_camera_recieved_ |= 0x2;
    }
    if (all_camera_recieved_ == 0x3) {
      if (camera_name == visual_camera_) {
        draw_range_circle();
        draw_selected_image_boundary(small_w_, small_h_,
                                     &(camera_image_[visual_camera_]));
        cv::Mat bigimg(world_h_, small_w_ + wide_pixel_, CV_8UC3);
        camera_image_[camera_names_[0]].copyTo(
            bigimg(cv::Rect(0, 0, small_w_, small_h_)));
        camera_image_[camera_names_[1]].copyTo(
            bigimg(cv::Rect(0, small_h_, small_w_, small_h_)));
        world_image_.copyTo(
            bigimg(cv::Rect(small_w_, 0, wide_pixel_, world_h_)));
        // cv::namedWindow("Apollo Visualizer", CV_WINDOW_NORMAL);
        // cv::setWindowProperty("Apollo Visualizer", CV_WND_PROP_FULLSCREEN,
        //                       CV_WINDOW_FULLSCREEN);
        // cv::imshow("Apollo Visualizer", bigimg);
        // int key = cvWaitKey(30);
        // key_handler(camera_name, key);

        // output visualization panel
        if (write_out_img_) {
          char path[1000];
          static int k = 0;
          snprintf(path, sizeof(path), "%s/%06d.jpg", path_.c_str(), k++);
          AINFO << "snapshot is saved at " << path;
          cv::imwrite(path, bigimg);
        }
        all_camera_recieved_ = 0x0;
      }  // if (camera_name == visual_camera)
    }    // if (all_camera_recieved_ == 0x3)
  }      // if (cv_imshow_img_)
}

void Visualizer::draw_range_circle() {
  cv::circle(world_image_, cv::Point(wide_pixel_ / 2, world_h_), 1 * m2pixel_,
             deep_sky_blue_color, 1);
  for (int i = 20; i < 300; i += 20) {
    cv::circle(world_image_, cv::Point(wide_pixel_ / 2, world_h_), i * m2pixel_,
               deep_sky_blue_color, 2);
  }
  for (int i = 50; i < 300; i += 50) {
    cv::putText(world_image_, std::to_string(i),
                cv::Point(wide_pixel_ / 2, world_h_ - i * m2pixel_),
                cv::FONT_HERSHEY_DUPLEX, 1, red_color, 2);
  }
}

void Visualizer::draw_selected_image_boundary(const int width, int const height,
                                              cv::Mat *image) {
  cv::Rect image_boundary(0, 0, width, height);
  cv::rectangle(*image, image_boundary, light_green_color, 4);
}

cv::Point Visualizer::world_point_to_bigimg(const Eigen::Vector2d &p) {
  cv::Point point;
  point.x = static_cast<int>(-p(1) * m2pixel_ + wide_pixel_ * 0.5);
  point.y = static_cast<int>(world_h_ - p(0) * m2pixel_);
  return point;
}
cv::Point Visualizer::world_point_to_bigimg(const Eigen::Vector4f &p) {
  cv::Point point;
  point.x = (wide_pixel_ >> 1) -
            static_cast<int>(p(1) * static_cast<float>(m2pixel_));
  point.y = world_h_ - static_cast<int>(p(0) * static_cast<float>(m2pixel_));
  return point;
}

Eigen::Vector2d Visualizer::image2ground(const std::string &camera_name,
                                         cv::Point p_img) {
  Eigen::Vector3d p_homo;

  p_homo << p_img.x, p_img.y, 1;
  Eigen::Vector3d p_ground;
  p_ground = homography_image2ground_[camera_name] * p_homo;
  if (fabs(p_ground(2)) > std::numeric_limits<double>::min()) {
    p_ground(0) = p_ground(0) / p_ground(2);
    p_ground(1) = p_ground(1) / p_ground(2);
  } else {
    AWARN << "p_ground(2) too small :" << p_ground(2);
  }
  return p_ground.block(0, 0, 2, 1);
}
cv::Point Visualizer::ground2image(const std::string &camera_name,
                                   Eigen::Vector2d p_ground) {
  Eigen::Vector3d p_homo;

  p_homo << p_ground(0), p_ground(1), 1;
  Eigen::Vector3d p_img;
  p_img = homography_ground2image_[camera_name] * p_homo;
  if (fabs(p_img(2)) > std::numeric_limits<double>::min()) {
    p_img(0) = p_img(0) / p_img(2);
    p_img(1) = p_img(1) / p_img(2);
  }
  return cv::Point(static_cast<int>(p_img(0)), static_cast<int>(p_img(1)));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
