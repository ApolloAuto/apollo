/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"  // NOLINT
#include "modules/perception/camera/lib/lane/detector/darkSCNN/darkSCNN_lane_detector.h"
#include "modules/perception/camera/lib/lane/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h"
#include "modules/perception/camera/tools/offline/visualizer.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

// @description: load camera extrinsics from yaml file
static bool LoadExtrinsics(const std::string &yaml_file,
                           Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}

TEST(darkSCNNLanePostprocessor, camera_lane_postprocessor_point_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  // turn on this flag to output visualization results
  bool enable_visualization_ = false;
  //  initialize lane detector
  LaneDetectorInitOptions init_options;
  LaneDetectorOptions detetor_options;
  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/data/";
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();
  //  BaseLaneDetector *detector =
  //      BaseLaneDetectorRegisterer::GetInstanceByName("darkSCNNLaneDetector");
  std::shared_ptr<DarkSCNNLaneDetector> detector(new DarkSCNNLaneDetector);
  AINFO << "detector: " << detector->Name();
  EXPECT_TRUE(detector->Init(init_options));
  //  initialize lane postprocessor
  std::shared_ptr<DarkSCNNLanePostprocessor> lane_postprocessor;
  lane_postprocessor.reset(new DarkSCNNLanePostprocessor);
  LanePostprocessorInitOptions postprocessor_init_options;
  postprocessor_init_options.detect_config_root =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/data/";
  postprocessor_init_options.detect_config_name = "config.pt";
  postprocessor_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/data/";
  postprocessor_init_options.conf_file = "lane_postprocessor_config.pt";
  lane_postprocessor->Init(postprocessor_init_options);
  LanePostprocessorOptions postprocessor_options;
  const std::string result_dir = "./result";

  //  set pitch angle
  float pitch_angle = 0.0f;
  //  set camera_ground_height (unit:meter)
  float camera_ground_height = 1.6f;
  CameraFrame frame;
  frame.camera_k_matrix = model.get_intrinsic_params();
  CalibrationServiceInitOptions calibration_service_init_options;
  calibration_service_init_options.calibrator_working_sensor_name =
      "onsemi_obstacle";
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
  name_intrinsic_map["onsemi_obstacle"] = frame.camera_k_matrix;
  calibration_service_init_options.name_intrinsic_map = name_intrinsic_map;
  calibration_service_init_options.calibrator_method = "LaneLineCalibrator";
  calibration_service_init_options.image_height =
      static_cast<int>(model.get_height());
  calibration_service_init_options.image_width =
      static_cast<int>(model.get_width());
  OnlineCalibrationService calibration_service;
  ACHECK(calibration_service.Init(calibration_service_init_options));

  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  name_camera_ground_height_map["onsemi_obstacle"] = camera_ground_height;
  name_camera_pitch_angle_diff_map["onsemi_obstacle"] = 0;
  calibration_service.SetCameraHeightAndPitch(name_camera_ground_height_map,
                                              name_camera_pitch_angle_diff_map,
                                              pitch_angle);
  frame.calibration_service = &calibration_service;

  //  image data initialized
  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  DataProvider::InitOptions dp_init_options;
  DataProvider data_provider;
  frame.data_provider = &data_provider;
  base::MotionBufferPtr motion_buffer;

  // initialize visualizer and set homography for lane_postprocessor
  Visualizer visualize_;
  double pitch_adj = 0;
  double yaw_adj = 0;
  double roll_adj = 0;
  std::string visual_camera = "onsemi_obstacle";
  EigenMap<std::string, Eigen::Matrix4d> extrinsic_map;
  EigenMap<std::string, Eigen::Matrix3f> intrinsic_map;
  Eigen::Matrix4d ex_camera2lidar;
  Eigen::Matrix4d ex_lidar2imu;
  Eigen::Matrix3d homography_im2car_;

  LoadExtrinsics(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/params/"
      "onsemi_obstacle_extrinsics.yaml",
      &ex_camera2lidar);
  LoadExtrinsics(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/darkSCNN/params/"
      "velodyne128_novatel_extrinsics.yaml",
      &ex_lidar2imu);

  intrinsic_map["onsemi_obstacle"] = frame.camera_k_matrix;
  extrinsic_map["onsemi_obstacle"] = ex_camera2lidar;
  std::vector<std::string> camera_names;
  camera_names[0] = visual_camera;

  EXPECT_TRUE(visualize_.Init_all_info_single_camera(
      camera_names, visual_camera, intrinsic_map, extrinsic_map, ex_lidar2imu,
      pitch_adj, yaw_adj, roll_adj,
      calibration_service_init_options.image_height,
      calibration_service_init_options.image_width));
  homography_im2car_ = visualize_.homography_im2car(visual_camera);
  lane_postprocessor->SetIm2CarHomography(homography_im2car_);
  AINFO << "Initilize visualizer finished!";

  if (enable_visualization_) {
    visualize_.write_out_img_ = true;
    visualize_.cv_imshow_img_ = true;
    visualize_.SetDirectory(result_dir);
  }

  // process each test image
  for (int i = 1; i < 5; i++) {
    std::string impath = cv::format(
        "/apollo/modules/perception/testdata/"
        "camera/lib/lane/postprocessor/"
        "darkSCNN/data/test_%d.jpg",
        i);
    cv::Mat img = cv::imread(impath);
    ACHECK(!img.empty()) << "input image is empty.";
    int size = img.cols * img.rows * img.channels();
    img_gpu_data.reset(new base::SyncedMemory(size, true));
    memcpy(img_gpu_data->mutable_cpu_data(), img.data, size * sizeof(uint8_t));
    dp_init_options.image_height = img.rows;
    dp_init_options.image_width = img.cols;
    dp_init_options.device_id = 0;
    dp_init_options.sensor_name = "onsemi_obstacle";
    frame.data_provider->Init(dp_init_options);
    frame.data_provider->FillImageData(
        img.rows, img.cols, (const uint8_t *)(img_gpu_data->mutable_gpu_data()),
        "bgr8");
    EXPECT_TRUE(detector->Detect(detetor_options, &frame));
    AINFO << "detection finished!";
    //  postprocess
    EXPECT_EQ(lane_postprocessor->Name(), "DarkSCNNLanePostprocessor");
    EXPECT_TRUE(lane_postprocessor->Process2D(postprocessor_options, &frame));
    AINFO << "2D postprocess finished!";
    EXPECT_TRUE(lane_postprocessor->Process3D(postprocessor_options, &frame));
    AINFO << "3D postprocess finished!";

    if (enable_visualization_) {
      frame.frame_id = i;
      frame.timestamp = static_cast<double>(i);
      Eigen::Affine3d world2camera = Eigen::Affine3d::Identity();
      visualize_.ShowResult_all_info_single_camera(img, frame, motion_buffer,
                                                   world2camera);
    }

    // delete detector;
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
