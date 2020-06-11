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

#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(DetectionTest, init_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  {
    std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
    TrafficLightDetectorInitOptions init_options;
    init_options.root_dir = "non-exists";
    init_options.conf_file = "non-exists";
    EXPECT_FALSE(detector->Init(init_options));
  }
  {
    std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
    TrafficLightDetectorInitOptions init_options;
    init_options.conf_file = "rgb_config.pt";
    init_options.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/detector/detection/data/";
    init_options.gpu_id = 0;
    EXPECT_TRUE(detector->Init(init_options));
    EXPECT_NEAR(detector->mean_[0], 122.7717, 1e-4);
    EXPECT_NEAR(detector->mean_[1], 115.9465, 1e-4);
    EXPECT_NEAR(detector->mean_[2], 102.9801, 1e-4);
  }
  {
    std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
    TrafficLightDetectorInitOptions init_options;
    init_options.conf_file = "config_whole_crop.pt";
    init_options.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/detector/detection/data/";
    init_options.gpu_id = 0;
    EXPECT_TRUE(detector->Init(init_options));
  }
}

TEST(DetectionTest, all) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions detetor_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/detection/img/img.png");
  cv::cvtColor(origin_image, origin_image, cv::COLOR_BGR2RGB);
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
      "camera/lib/traffic_light/detector/detection/data/";
  init_options.gpu_id = 0;
  AINFO << "init options ready";
  EXPECT_TRUE(detector->Init(init_options));
  AINFO << "init success";
  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);
  lights[0]->region.projection_roi.x = 1150;
  lights[0]->region.projection_roi.y = 400;
  lights[0]->region.projection_roi.width = 50;
  lights[0]->region.projection_roi.height = 130;

  lights.emplace_back(new base::TrafficLight);
  lights[1]->region.projection_roi.x = 1500;
  lights[1]->region.projection_roi.y = 390;
  lights[1]->region.projection_roi.width = 40;
  lights[1]->region.projection_roi.height = 120;

  std::vector<base::RectI> gt_lights;
  base::RectI light1(1193, 419, 45, 129);
  base::RectI light2(1551, 420, 45, 129);
  gt_lights.push_back(light1);
  gt_lights.push_back(light2);
  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  EXPECT_TRUE(detector->Detect(detetor_options, &frame));

  EXPECT_EQ(lights.size(), gt_lights.size());
  for (int i = 0; i < lights.size(); i++) {
    bool get = false;
    base::RectI region = lights[i]->region.detection_roi;
    for (int j = 0; j < gt_lights.size(); j++) {
      base::RectI _inter = region & gt_lights[j];
      base::RectI _union = region | gt_lights[j];
      float iou = static_cast<float>(_inter.Area()) /
                  static_cast<float>(_union.Area() + 1e-6);
      AINFO << "IOU " << iou;
      if (iou > 0.5) {
        get = true;
        break;
      }
    }
    EXPECT_TRUE(get);
  }
#endif
}

TEST(DetectionTest, no_light) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
  AINFO << detector->Name();
  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions detetor_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/detection/img/img.png");
  cv::cvtColor(origin_image, origin_image, cv::COLOR_BGR2RGB);
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
      "camera/lib/traffic_light/detector/detection/data/";
  init_options.gpu_id = 0;
  AINFO << "init options ready";
  EXPECT_TRUE(detector->Init(init_options));
  AINFO << "init success";
  std::vector<base::TrafficLightPtr> lights;
  lights.clear();
  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  EXPECT_TRUE(detector->Detect(detetor_options, &frame));
#endif
}

TEST(DetectionTest, out_of_img_light) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions detetor_options;
  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/detection/img/img.png");
  cv::cvtColor(origin_image, origin_image, cv::COLOR_BGR2RGB);
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
      "camera/lib/traffic_light/detector/detection/data/";
  init_options.gpu_id = 0;
  AINFO << "init options ready";
  EXPECT_TRUE(detector->Init(init_options));
  AINFO << "init success";
  std::vector<base::TrafficLightPtr> lights;
  lights.emplace_back(new base::TrafficLight);
  lights[0]->region.projection_roi.x = 0;
  lights[0]->region.projection_roi.y = 0;
  lights[0]->region.projection_roi.width = 0;
  lights[0]->region.projection_roi.height = 0;

  lights.emplace_back(new base::TrafficLight);
  lights[1]->region.projection_roi.x = 2000;
  lights[1]->region.projection_roi.y = 3000;
  lights[1]->region.projection_roi.width = 40;
  lights[1]->region.projection_roi.height = 120;

  lights.emplace_back(new base::TrafficLight);
  lights[2]->region.projection_roi.x = 1000;
  lights[2]->region.projection_roi.y = 300;
  lights[2]->region.projection_roi.width = 40;
  lights[2]->region.projection_roi.height = 120;

  std::vector<base::RectI> gt_lights;
  base::RectI light2(1551, 420, 45, 129);
  gt_lights.push_back(light2);
  frame.traffic_lights = lights;

#ifndef CPU_ONLY
  EXPECT_TRUE(detector->Detect(detetor_options, &frame));

  EXPECT_FALSE(lights[0]->region.is_detected);
  EXPECT_FALSE(lights[1]->region.is_detected);
  EXPECT_FALSE(lights[2]->region.is_detected);
#endif
}

TEST(DetectionTest, nms_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  std::shared_ptr<TrafficLightDetection> detector(new TrafficLightDetection);
  TrafficLightDetectorInitOptions init_options;
  TrafficLightDetectorOptions detetor_options;

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/detector/detection/data/";
  init_options.gpu_id = 0;
  AINFO << "init options ready";
  EXPECT_TRUE(detector->Init(init_options));

  {
    std::vector<base::TrafficLightPtr> lights;
    lights.emplace_back(new base::TrafficLight);
    lights[0]->region.detection_roi.x = 100;
    lights[0]->region.detection_roi.y = 100;
    lights[0]->region.detection_roi.width = 40;
    lights[0]->region.detection_roi.height = 80;
    lights[0]->region.detect_score = 0.91;

    lights.emplace_back(new base::TrafficLight);
    lights[1]->region.detection_roi.x = 100;
    lights[1]->region.detection_roi.y = 80;
    lights[1]->region.detection_roi.width = 50;
    lights[1]->region.detection_roi.height = 100;
    lights[1]->region.detect_score = 0.9;

    lights.emplace_back(new base::TrafficLight);
    lights[2]->region.detection_roi.x = 500;
    lights[2]->region.detection_roi.y = 500;
    lights[2]->region.detection_roi.width = 40;
    lights[2]->region.detection_roi.height = 90;
    lights[2]->region.detect_score = 0.8;

    lights.emplace_back(new base::TrafficLight);
    lights[3]->region.detection_roi.x = 500;
    lights[3]->region.detection_roi.y = 500;
    lights[3]->region.detection_roi.width = 40;
    lights[3]->region.detection_roi.height = 85;
    lights[3]->region.detect_score = 0.81;

    detector->ApplyNMS(&lights);
    EXPECT_EQ(lights.size(), 2);
    for (auto light : lights) {
      AINFO << light->region.detection_roi.ToStr();
    }
  }

  {
    std::vector<base::TrafficLightPtr> lights;
    lights.emplace_back(new base::TrafficLight);
    lights[0]->region.detection_roi.x = 100;
    lights[0]->region.detection_roi.y = 100;
    lights[0]->region.detection_roi.width = 40;
    lights[0]->region.detection_roi.height = 80;
    lights[0]->region.detect_score = 0.91;

    detector->ApplyNMS(&lights);
    EXPECT_EQ(lights.size(), 1);
    for (auto light : lights) {
      AINFO << light->region.detection_roi.ToStr();
    }
  }

  {
    std::vector<base::TrafficLightPtr> lights;
    detector->ApplyNMS(&lights);
    EXPECT_EQ(lights.size(), 0);
  }

  {
    std::vector<base::TrafficLightPtr> lights;
    lights.emplace_back(new base::TrafficLight);
    lights[0]->region.detection_roi.x = 100;
    lights[0]->region.detection_roi.y = 100;
    lights[0]->region.detection_roi.width = 40;
    lights[0]->region.detection_roi.height = 80;
    lights[0]->region.detect_score = 0.91;

    lights.emplace_back(new base::TrafficLight);
    lights[1]->region.detection_roi.x = 500;
    lights[1]->region.detection_roi.y = 500;
    lights[1]->region.detection_roi.width = 40;
    lights[1]->region.detection_roi.height = 90;
    lights[1]->region.detect_score = 0.8;

    detector->ApplyNMS(&lights);
    EXPECT_EQ(lights.size(), 2);
    for (auto light : lights) {
      AINFO << light->region.detection_roi.ToStr();
    }
  }

  {
    std::vector<base::TrafficLightPtr> lights;
    for (int i = 0; i < 100; ++i) {
      lights.emplace_back(new base::TrafficLight);
      lights.back()->region.detection_roi.x = 100;
      lights.back()->region.detection_roi.y = 100;
      lights.back()->region.detection_roi.width = 40;
      lights.back()->region.detection_roi.height = 80;
      lights.back()->region.detect_score = 0.9;
    }

    detector->ApplyNMS(&lights);
    EXPECT_EQ(lights.size(), 1);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
