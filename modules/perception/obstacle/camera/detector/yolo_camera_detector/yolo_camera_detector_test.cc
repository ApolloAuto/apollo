/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include <string>
#include <vector>

#include "caffe/caffe.hpp"
#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/cuda_util/region_output.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"

DECLARE_string(yolo_config_filename);

DEFINE_string(test_dir,
              "/apollo/modules/perception/data/yolo_camera_detector_test/",
              "test data directory");

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::SetProtoToASCIIFile;
using apollo::perception::obstacle::yolo::YoloParam;

class YoloCameraDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_config_manager_path = FLAGS_test_dir + "config_manager.config",
    FLAGS_yolo_config_filename = "config.pt";
    RegisterFactoryYoloCameraDetector();
  }
};

TEST_F(YoloCameraDetectorTest, model_init_test) {
  const std::string yolo_config =
      "/apollo/modules/perception/model/yolo_camera_detector/config.pt";

  YoloParam yolo_param;
  CHECK(GetProtoFromASCIIFile(yolo_config, &yolo_param));
  YoloParam origin_yolo_param = yolo_param;

  CHECK(SetProtoToASCIIFile(yolo_param, yolo_config));
  BaseCameraDetector *camera_detector =
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector");
  CHECK_NOTNULL(camera_detector);
  CHECK(camera_detector->Init());

  // Recover to origin config.
  CHECK(SetProtoToASCIIFile(origin_yolo_param, yolo_config));
}

TEST_F(YoloCameraDetectorTest, yolo_camera_detector_roipooling_test) {
  BaseCameraDetector *camera_detector =
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector");
  CHECK(camera_detector->Init());
  CHECK_EQ(camera_detector->Name(), "YoloCameraDetector");

  const std::string image_file = FLAGS_test_dir + "test.jpg";
  ADEBUG << "test image file: " << image_file;

  cv::Mat frame = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  CHECK_NOTNULL(frame.data);

  CameraDetectorOptions options;
  CHECK_EQ(camera_detector->Detect(frame, options, NULL), false);

  std::vector<std::shared_ptr<VisualObject>> objects;
  CHECK(camera_detector->Detect(frame, options, &objects));
  ADEBUG << "#objects detected = " << objects.size();

  CHECK_EQ(objects.size(), 1);  // Related to current model and threshold

  int obj_idx = 0;
  for (const auto &obj : objects) {
    ADEBUG << "Obj-" << obj_idx++ << ": " << GetObjectName(obj->type)
           << " (feat: " << obj->object_feature.size() << "-D)";
    if (obj->object_feature.size() > 0) {
      float sum_of_squares = 0.0;
      for (const auto &f : obj->object_feature) {
        sum_of_squares += f * f;
      }
    }
  }
}

TEST_F(YoloCameraDetectorTest, multi_task_test) {
  BaseCameraDetector *camera_detector =
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector");
  CHECK(camera_detector->Init());
  CHECK_EQ(camera_detector->Name(), "YoloCameraDetector");

  const std::string image_file = FLAGS_test_dir + "test.jpg";
  ADEBUG << "test image file: " << image_file;

  cv::Mat frame = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  CHECK_NOTNULL(frame.data);

  CameraDetectorOptions options;
  CHECK_EQ(camera_detector->Multitask(frame, options, NULL, NULL), false);

  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat lane_map(frame.rows, frame.cols, CV_32FC1);
  CHECK(camera_detector->Multitask(frame, options, &objects, &lane_map));
  ADEBUG << "#objects detected = " << objects.size();

  CHECK_EQ(objects.size(), 1);  // Related to current model and threshold

  const std::string lane_map_result_file = FLAGS_test_dir + "lane_map.jpg";
  lane_map.convertTo(lane_map, CV_8UC3, 255.0f);
  cv::imwrite(lane_map_result_file, lane_map);
}

}  // namespace perception
}  // namespace apollo
