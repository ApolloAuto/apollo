// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/27 15:18:23
// @file test_calibration.cpp
// @brief 
// 
#include <gtest/gtest.h>

#include "module/perception/traffic_light/interface/base_projection.h"
namespace adu {
namespace perception {
namespace traffic_light {

// class TestCalibration : public BaseCalibration {
// public:
//     TestCalibration() = default;
//     ~TestCalibration() = default;

//     virtual bool init() override {
//         return true;
//     }

//     virtual bool calibrate(const CarPose& pose,
//                            const CalibrateOption& option,
//                            Light* light) {
//         return true;
//     }
// };

// REGISTER_CALIBRATION(TestCalibration);

// TEST(TestCalibration, init) {
//     BaseCalibration* calibration = BaseCalibrationRegisterer::get_instance_by_name("TestCalibration");
//     EXPECT_TRUE(calibration != nullptr);
// }

TEST(test_load_transformation_matrix_from_file, non_exist_config_file) {
  std::string file_name = "non-exist";
  Eigen::Matrix4d trans_mat;
  ASSERT_FALSE(load_transformation_matrix_from_file(file_name, &trans_mat));
}

TEST(test_load_transformation_matrix_from_file, no_transform_config_file) {
  std::string file_name = "data/onsemi_obstacle_intrinsics.yaml";
  Eigen::Matrix4d trans_mat;
  ASSERT_FALSE(load_transformation_matrix_from_file(file_name, &trans_mat));
}

TEST(test_load_transformation_matrix_from_file, no_translation_config_file) {
  std::string file_name = "data/onsemi_obstacle_extrinsics_no_translation.yaml";
  Eigen::Matrix4d trans_mat;
  ASSERT_FALSE(load_transformation_matrix_from_file(file_name, &trans_mat));
}

TEST(test_load_transformation_matrix_from_file, no_rotation_config_file) {
  std::string file_name = "data/onsemi_obstacle_extrinsics_no_rotation.yaml";
  Eigen::Matrix4d trans_mat;
  ASSERT_FALSE(load_transformation_matrix_from_file(file_name, &trans_mat));
}

TEST(test_load_matrix4d_from_file, non_exist_config_file) {
  std::string file_name = "non-exist";
  Eigen::Matrix4d trans_mat;
  std::string key = "non-exist";
  ASSERT_FALSE(load_matrix4d_from_file(file_name, key, &trans_mat));
}

TEST(test_load_matrix4d_from_file, invalid_key_config_file) {
  std::string file_name = "data/onsemi_obstacle_intrinsics.yaml";
  Eigen::Matrix4d trans_mat;
  std::string key = "non-exist";
  ASSERT_FALSE(load_matrix4d_from_file(file_name, key, &trans_mat));
}

TEST(test_init_camera_intrinsic_matrix_and_distort_params, no_K) {
  std::string file_name = "data/onsemi_obstacle_intrinsics_no_k.yaml";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_camera_intrinsic_matrix_and_distort_params(file_name));
}

TEST(test_init_camera_intrinsic_matrix_and_distort_params, no_D) {
  std::string file_name = "data/onsemi_obstacle_intrinsics_no_d.yaml";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_camera_intrinsic_matrix_and_distort_params(file_name));
}

TEST(test_init_camera_intrinsic_matrix_and_distort_params, no_height) {
  std::string file_name = "data/onsemi_obstacle_intrinsics_no_height.yaml";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_camera_intrinsic_matrix_and_distort_params(file_name));
}

TEST(test_init_camera_intrinsic_matrix_and_distort_params, no_width) {
  std::string file_name = "data/onsemi_obstacle_intrinsics_no_width.yaml";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_camera_intrinsic_matrix_and_distort_params(file_name));
}

TEST(test_init_lidar_to_camera_matrix, invalid_lidar2camera_file) {
  std::string file_name = "non-exist";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_lidar_to_camera_matrix(file_name));
}

TEST(test_init_lidar_to_gps_matrix, invalid_lidar2gps_file) {
  std::string file_name = "non-exist";
  CameraCoeffient camera_coeff;
  ASSERT_FALSE(camera_coeff.init_lidar_to_gps_matrix(file_name));
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
