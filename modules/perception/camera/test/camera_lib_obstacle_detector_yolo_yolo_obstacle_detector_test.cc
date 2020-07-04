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
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/yolo_obstacle_detector.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

TEST(YoloCameraDetectorTest, demo_test) {
  inference::CudaUtil::set_device_id(0);
  cv::Mat cv_img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/img/test.jpg");
  ACHECK(!cv_img.empty()) << "image is empty.";

  base::Image8U image(cv_img.rows, cv_img.cols, base::Color::BGR);

  for (int y = 0; y < cv_img.rows; ++y) {
    memcpy(image.mutable_cpu_ptr(y), cv_img.ptr<uint8_t>(y),
           image.width_step());
  }
  CameraFrame frame;
  DataProvider data_provider;
  frame.data_provider = &data_provider;
  if (frame.track_feature_blob == nullptr) {
    frame.track_feature_blob.reset(new base::Blob<float>());
  }
  DataProvider::InitOptions dp_init_options;
  dp_init_options.sensor_name = "onsemi_obstacle";

  dp_init_options.image_height = cv_img.rows;
  dp_init_options.image_width = cv_img.cols;
  dp_init_options.device_id = 0;
  ACHECK(frame.data_provider->Init(dp_init_options));
  ACHECK(frame.data_provider->FillImageData(cv_img.rows, cv_img.cols,
                                            image.gpu_data(), "bgr8"));

  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";

  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_TRUE(detector->Init(init_options));

  ObstacleDetectorOptions options;
  EXPECT_TRUE(detector->Detect(options, &frame));
  EXPECT_FALSE(detector->Detect(options, nullptr));

  // EXPECT_EQ(frame.detected_objects.size(), 8);
  int obj_id = 0;
  for (auto obj : frame.detected_objects) {
    // AINFO << static_cast<int>(obj->type) << "\t"
    //          << static_cast<int>(obj->sub_type);
    auto &box = obj->camera_supplement.box;
    cv::rectangle(
        cv_img,
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
        cv::Scalar(0, 255, 0), 2);
    std::stringstream text;
    text << static_cast<int>(obj->sub_type) << " - " << obj_id++;
    fprintf(stderr,
            "%4d 0 0 %6.3f %8.2f %8.2f %8.2f %8.2f %6.3f %6.3f %6.3f "
            "0 0 0 0 %6.3f %4d %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
            static_cast<int>(obj->sub_type), obj->camera_supplement.alpha,
            obj->camera_supplement.box.xmin, obj->camera_supplement.box.ymin,
            obj->camera_supplement.box.xmax, obj->camera_supplement.box.ymax,
            obj->size[2], obj->size[1], obj->size[0],
            obj->type_probs[static_cast<int>(obj->type)], 0,
            obj->camera_supplement.visible_ratios[0],
            obj->camera_supplement.visible_ratios[1],
            obj->camera_supplement.visible_ratios[2],
            obj->camera_supplement.visible_ratios[3],
            obj->camera_supplement.cut_off_ratios[0],
            obj->camera_supplement.cut_off_ratios[1]);
    cv::putText(
        cv_img, text.str(),
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);
  }
  cv::imwrite("output.jpg", cv_img);
  delete detector;
}
TEST(YoloCameraDetectorTest, config_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "configbak.pt";

  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_FALSE(detector->Init(init_options));
  delete detector;
}
TEST(YoloCameraDetectorTest, inference_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";
  std::string config_path =
      GetAbsolutePath(init_options.root_dir, init_options.conf_file);
  yolo::YoloParam yolo_param;
  cyber::common::GetProtoFromFile(config_path, &yolo_param);
  yolo::YoloParam origin_yolo_param = yolo_param;
  yolo_param.mutable_model_param()->set_model_type("Unknownnet");

  {
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_FALSE(detector->Init(init_options));
  {
    delete detector;
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
}
TEST(YoloCameraDetectorTest, anchor_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";
  std::string config_path =
      GetAbsolutePath(init_options.root_dir, init_options.conf_file);
  yolo::YoloParam yolo_param;
  yolo::YoloParam origin_yolo_param;
  cyber::common::GetProtoFromFile(config_path, &yolo_param);
  origin_yolo_param.CopyFrom(yolo_param);
  yolo_param.mutable_model_param()->set_anchors_file("unknown_anchor.txt");
  {
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_FALSE(detector->Init(init_options));
  {
    delete detector;
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
}
TEST(YoloCameraDetectorTest, type_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";
  std::string config_path =
      GetAbsolutePath(init_options.root_dir, init_options.conf_file);
  yolo::YoloParam yolo_param;
  yolo::YoloParam origin_yolo_param;
  cyber::common::GetProtoFromFile(config_path, &yolo_param);
  origin_yolo_param.CopyFrom(yolo_param);
  yolo_param.mutable_model_param()->set_types_file("config.pt");
  {
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_FALSE(detector->Init(init_options));
  {
    delete detector;
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
}
TEST(YoloCameraDetectorTest, feature_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";
  std::string config_path =
      GetAbsolutePath(init_options.root_dir, init_options.conf_file);
  yolo::YoloParam yolo_param;
  yolo::YoloParam origin_yolo_param;
  cyber::common::GetProtoFromFile(config_path, &yolo_param);
  origin_yolo_param.CopyFrom(yolo_param);
  yolo_param.mutable_model_param()->set_feature_file("unknown.pt");
  {
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/detector/yolo/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_FALSE(detector->Init(init_options));
  {
    delete detector;
    std::string out_str;
    std::ofstream ofs(config_path, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
  }
}

#if 0
TEST(YoloCameraDetectorTest, cameramodel_init_test) {
  ObstacleDetectorInitOptions init_options;
  init_options.root_dir = "/apollo/modules/perception/testdata/"
    "camera/lib/obstacle/detector/yolo/data/";
  init_options.conf_file = "config.pt";

  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  CHECK_EQ(detector->Name(), "YoloObstacleDetector");
  EXPECT_TRUE(detector->Init(init_options));
  delete detector;
}
#endif
}  // namespace camera
}  // namespace perception
}  // namespace apollo
