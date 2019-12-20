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

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/yolo_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/transformer/multicue/multicue_obstacle_transformer.h"  // NOLINT
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(MultiCueObstacleTransformerTest, multicue_obstacle_transformer_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  CHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  inference::CudaUtil::set_device_id(0);
  cv::Mat cv_img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/transformer/multicue/test.jpg");
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
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(cv_img.rows, cv_img.cols, image.gpu_data(),
                                     "bgr8");

  ObstacleDetectorOptions detector_options;
  ObstacleTransformerOptions transformer_options;
  ObstacleDetectorInitOptions detector_init_options;
  ObstacleTransformerInitOptions transformer_init_options;

  detector_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/transformer/multicue/data/";
  detector_init_options.conf_file = "config.pt";

  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/transformer/multicue/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  detector_init_options.base_camera_model = model.get_camera_model();
  auto pinhole =
      static_cast<base::PinholeCameraModel *>(model.get_camera_model().get());
  Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
  frame.camera_k_matrix = intrinsic;

  // transformer
  BaseObstacleTransformer *transformer =
      BaseObstacleTransformerRegisterer::GetInstanceByName(
          "MultiCueObstacleTransformer");
  EXPECT_EQ(transformer->Name(), "MultiCueObstacleTransformer");

  EXPECT_FALSE(transformer->Init(transformer_init_options));

  transformer_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/transformer/multicue/";
  transformer_init_options.conf_file = "config.pt";
  EXPECT_TRUE(transformer->Init(transformer_init_options));

  // blank input objects
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // detection
  BaseObstacleDetector *detector =
      BaseObstacleDetectorRegisterer::GetInstanceByName("YoloObstacleDetector");
  EXPECT_TRUE(detector->Init(detector_init_options));
  EXPECT_TRUE(detector->Detect(detector_options, &frame));

  // transformer
  // real data from detection
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // dummy data
  // null object pointer
  base::ObjectPtr obj = nullptr;
  frame.detected_objects.resize(0);
  frame.detected_objects.push_back(obj);
  EXPECT_FALSE(transformer->Transform(transformer_options, &frame));

  // not null object pointer
  obj.reset(new base::Object);
  frame.detected_objects.resize(0);
  frame.detected_objects.push_back(obj);
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // alpha is out of range
  const float PI = common::Constant<float>::PI();
  obj->camera_supplement.alpha = -2 * PI;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->camera_supplement.alpha = 2 * PI;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // various visual types
  obj->sub_type = base::ObjectSubType::CAR;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::VAN;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::BUS;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::TRUCK;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::CYCLIST;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::TRICYCLIST;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::PEDESTRIAN;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::TRAFFICCONE;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::UNKNOWN_MOVABLE;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::UNKNOWN_UNMOVABLE;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->sub_type = base::ObjectSubType::MAX_OBJECT_TYPE;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // object type is vehicle
  obj->type = base::ObjectType::VEHICLE;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // bounding box is out of image boundaries
  const int width_image = frame.data_provider->src_width();
  const int height_image = frame.data_provider->src_height();
  ObjMapperParams params;
  float min_x = static_cast<float>(params.boundary_len);
  float min_y = static_cast<float>(params.boundary_len);
  float max_x = static_cast<float>(width_image - params.boundary_len);
  float max_y = static_cast<float>(height_image - params.boundary_len);
  obj->size(2) = 1.6182f;
  obj->size(1) = 0.4825f;
  obj->size(0) = 1.4886f;
  obj->camera_supplement.box.xmin = min_x + 1.0f;
  obj->camera_supplement.box.ymin = min_y + 1.0f;
  obj->camera_supplement.box.xmax = max_x - 1.0f;
  obj->camera_supplement.box.ymax = max_y - 1.0f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->camera_supplement.box.xmin = -1.0f;
  obj->camera_supplement.box.ymin = min_y + 1.0f;
  obj->camera_supplement.box.xmax = max_x - 1.0f;
  obj->camera_supplement.box.ymax = max_y - 1.0f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->camera_supplement.box.xmin = min_x + 1.0f;
  obj->camera_supplement.box.ymin = -1.0f;
  obj->camera_supplement.box.xmax = max_x - 1.0f;
  obj->camera_supplement.box.ymax = max_y - 1.0f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->camera_supplement.box.xmin = min_x + 1.0f;
  obj->camera_supplement.box.ymin = min_y + 1.0f;
  obj->camera_supplement.box.xmax = -1.0f;
  obj->camera_supplement.box.ymax = max_y - 1.0f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->camera_supplement.box.xmin = min_x + 1.0f;
  obj->camera_supplement.box.ymin = min_y + 1.0f;
  obj->camera_supplement.box.xmax = max_x - 1.0f;
  obj->camera_supplement.box.ymax = -1.0f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  // abnormal dimension hwl
  obj->camera_supplement.box.xmin = min_x + 1.0f;
  obj->camera_supplement.box.ymin = min_y + 1.0f;
  obj->camera_supplement.box.xmax = max_x - 1.0f;
  obj->camera_supplement.box.ymax = max_y - 1.0f;
  obj->size(2) = -1.0f;
  obj->size(1) = 0.4825f;
  obj->size(0) = 1.4886f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->size(2) = 1.6182f;
  obj->size(1) = -1.0f;
  obj->size(0) = 1.4886f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));
  obj->size(2) = 1.6182f;
  obj->size(1) = 0.4825f;
  obj->size(0) = 1.4886f;
  EXPECT_TRUE(transformer->Transform(transformer_options, &frame));

  delete transformer;
  delete detector;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
