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

#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(RecognizeTest, yellow) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/yellow.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  base::RectI region(918, 241, 18, 54);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_VERTICAL_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_YELLOW == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, red) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/red.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  base::RectI region(1043, 601, 21, 54);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_VERTICAL_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_RED == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, green) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/green.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));
  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);
  base::RectI region(538, 332, 52, 135);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_VERTICAL_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_GREEN == lights[0]->status.color);
}

TEST(RecognizeTest, black) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/black.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  base::RectI region(1209, 201, 25, 64);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_VERTICAL_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_BLACK == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, no_detection) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/black.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  lights[0]->region.is_detected = false;
  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_UNKNOWN_COLOR == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, unknown_class) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/yellow.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  base::RectI region(918, 241, 18, 54);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_UNKNOWN_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_FALSE(recognition->Detect(recognition_options, &frame));
#endif
}

TEST(RecognizeTest, quadrate) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/green.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);

  base::RectI region(544, 425, 40, 44);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id = base::TLDetectionClass::TL_QUADRATE_CLASS;
  lights[0]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_GREEN == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, horizontal) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/horizontal.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);
  base::RectI region(1354, 635, 71, 24);
  lights[0]->region.is_detected = true;
  lights[0]->region.detect_class_id =
      base::TLDetectionClass::TL_HORIZONTAL_CLASS;
  lights[0]->region.detection_roi = region;

  lights.emplace_back(new base::TrafficLight);
  lights[1]->region.is_detected = false;
  lights[1]->region.detect_class_id =
      base::TLDetectionClass::TL_HORIZONTAL_CLASS;
  lights[1]->region.detection_roi = region;

  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  AINFO << "COLOR " << static_cast<int>(lights[0]->status.color);
  ASSERT_TRUE(base::TLColor::TL_GREEN == lights[0]->status.color);
#endif
}

TEST(RecognizeTest, no_light) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions recognition_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/img/horizontal.jpg");
  ASSERT_FALSE(origin_image.data == nullptr);

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/recognition/data/";
  init_options.gpu_id = 0;
  EXPECT_TRUE(recognition->Init(init_options));

  std::vector<base::TrafficLightPtr> lights;
  lights.clear();
  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  ASSERT_TRUE(recognition->Detect(recognition_options, &frame));
  EXPECT_EQ(lights.size(), 0);
#endif
}

TEST(RecognizeTest, bad_config) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  {
    TrafficLightDetectorInitOptions init_options;
    DataProvider::InitOptions dp_init_options;
    init_options.conf_file = "bad_config.pt";
    init_options.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/detector/recognition/data/";
    init_options.gpu_id = 0;
    recognition->Init(init_options);
    init_options.conf_file = "non_exist_config.pt";
    init_options.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/detector/recognition/data/";
    init_options.gpu_id = 0;
    EXPECT_FALSE(recognition->Init(init_options));
  }
}

TEST(RecognizeTest, rgb_config) {
  std::shared_ptr<TrafficLightRecognition> recognition(
      new TrafficLightRecognition);

  {
    TrafficLightDetectorInitOptions init_options;
    DataProvider::InitOptions dp_init_options;
    init_options.conf_file = "rgb_config.pt";
    init_options.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/detector/recognition/data/";
    init_options.gpu_id = 0;
    EXPECT_TRUE(recognition->Init(init_options));
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
