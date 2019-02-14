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

#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/perception/base/point.h"
#include "modules/perception/camera/lib/traffic_light/preprocessor/tl_preprocessor.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}  // namespace common
namespace camera {

class TLPreprocessorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    unsetenv("MODULE_PATH");
    unsetenv("CYBER_PATH");
    FLAGS_obs_sensor_meta_path =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/preprocessor/conf/sensor_meta.config";
    FLAGS_obs_sensor_intrinsic_path =
        "/apollo/modules/perception/testdata/"
        "camera/lib/traffic_light/preprocessor/data/multi_projection";
    preprocessor_ = new TLPreprocessor;
    common::SensorManager::Instance()->Init();
    camera_names_.push_back("onsemi_traffic");
    camera_names_.push_back("onsemi_narrow");
    camera_names_.push_back("onsemi_obstacle");
    camera_names_.push_back("onsemi_wide");
  }

  ~TLPreprocessorTest() { delete preprocessor_; }

  void PrepareTestDataLongFocus(CarPose *pose,
                                std::vector<base::PointXYZID> *boundary) {
    pose->pose_ << 0.970765, -0.000792166, 0.240032, 408528, -0.239319,
        0.0738927, 0.968126, 4.31098e+06, -0.0185036, -0.997266, 0.0715428,
        -2.67629, 0, 0, 0, 1;

    pose->SetCameraPose("onsemi_traffic", pose->pose_);
    pose->SetCameraPose("onsemi_obstacle", pose->pose_);

    if (!boundary) {
      return;
    }

    double x1 = 408575.09159042523;
    double x2 = 408575.89141399995;
    double y1 = 4311145.1306145815;
    double y2 = 4311144.981124;
    double z1 = 1.311;
    double z2 = 2.932;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    base::PointXYZID pt;
    pt.x = x1;
    pt.y = y1;
    pt.z = z1;
    boundary->push_back(pt);

    pt.x = x2;
    pt.y = y2;
    pt.z = z1;
    boundary->push_back(pt);

    pt.x = x2;
    pt.y = y2;
    pt.z = z2;
    boundary->push_back(pt);

    pt.x = x1;
    pt.y = y1;
    pt.z = z2;
    boundary->push_back(pt);

    AINFO << "prepare long focus data done.";
  }

  void PrepareTestDataShortFocus(CarPose *pose,
                                 std::vector<base::PointXYZID> *boundary) {
    pose->pose_ << 0.969254, -0.0103669, 0.245844, 408563, -0.245933,
        -0.00847109, 0.96925, 4.3111e+06, -0.00796562, -0.999911, -0.0107602,
        -2.93461, 0, 0, 0, 1;

    pose->SetCameraPose("onsemi_traffic", pose->pose_);
    pose->SetCameraPose("onsemi_obstacle", pose->pose_);

    if (!boundary) {
      return;
    }

    double x1 = 408575.09159042523;
    double x2 = 408575.89141399995;
    double y1 = 4311145.1306145815;
    double y2 = 4311144.981124;
    double z1 = 1.311;
    double z2 = 2.932;

    /*
      (x1, y1, z1)
      (x2, y2, z1)
      (x2, y2, z2)
      (x1, y1, z2)
     */
    base::PointXYZID pt;
    pt.x = x1;
    pt.y = y1;
    pt.z = z1;
    boundary->push_back(pt);

    pt.x = x2;
    pt.y = y2;
    pt.z = z1;
    boundary->push_back(pt);

    pt.x = x2;
    pt.y = y2;
    pt.z = z2;
    boundary->push_back(pt);

    pt.x = x1;
    pt.y = y1;
    pt.z = z2;
    boundary->push_back(pt);
  }

 protected:
  TLPreprocessor *preprocessor_;
  std::vector<std::string> camera_names_;
};

TEST_F(TLPreprocessorTest, test_set_and_get_camera_is_working_flag) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  ASSERT_TRUE(preprocessor_->Init(init_options));
  preprocessor_->GetAlllightsOutsideFlag();
  std::string camera_name = "invalid";
  bool is_working = false;
  ASSERT_FALSE(preprocessor_->SetCameraWorkingFlag(camera_name, true));
  ASSERT_FALSE(preprocessor_->SetCameraWorkingFlag(camera_name, false));
  ASSERT_FALSE(preprocessor_->GetCameraWorkingFlag(camera_name, &is_working));
  ASSERT_FALSE(is_working);

  camera_name = "";
  is_working = false;
  ASSERT_FALSE(preprocessor_->SetCameraWorkingFlag(camera_name, true));
  ASSERT_FALSE(preprocessor_->SetCameraWorkingFlag(camera_name, false));
  ASSERT_FALSE(preprocessor_->GetCameraWorkingFlag(camera_name, &is_working));
  ASSERT_FALSE(is_working);

  camera_name = "onsemi_narrow";
  is_working = false;
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(camera_name, true));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(camera_name, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(camera_name, true));
  ASSERT_TRUE(preprocessor_->GetCameraWorkingFlag(camera_name, &is_working));
  ASSERT_TRUE(is_working);

  camera_name = "onsemi_traffic";
  is_working = false;
  ASSERT_TRUE(preprocessor_->GetCameraWorkingFlag(camera_name, &is_working));
  ASSERT_FALSE(is_working);
}

TEST_F(TLPreprocessorTest, test_project_lights) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  ASSERT_TRUE(preprocessor_->Init(init_options));

  // empty signals
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights;

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;
    std::string camera_name = "onsemi_traffic";
    ASSERT_TRUE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
  }

  // invalid camera id
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

    std::string camera_name = "non-exist";
    ASSERT_FALSE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
  }

  // long focus project on image
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    preprocessor_->SetCameraWorkingFlag("onsemi_traffic", true);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

    std::string camera_name = "onsemi_traffic";
    ASSERT_TRUE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
    EXPECT_EQ(1, lights_on_image.size());
    EXPECT_EQ(0, lights_outside_image.size());
  }

  // short focus project on image
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    std::string camera_name = "onsemi_obstacle";
    preprocessor_->SetCameraWorkingFlag(camera_name, true);
    PrepareTestDataShortFocus(&pose, &(lights[0]->region.points));

    AINFO << "test cam id " << camera_name;
    ASSERT_TRUE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
    EXPECT_EQ(1, lights_on_image.size());
    EXPECT_EQ(0, lights_outside_image.size());
  }

  // long focus project outside image
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    std::string camera_name = "onsemi_traffic";
    preprocessor_->SetCameraWorkingFlag(camera_name, true);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    pose.c2w_poses_[camera_name](0, 3) =
        pose.c2w_poses_[camera_name](0, 3) + 100000;
    ASSERT_TRUE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
    EXPECT_EQ(0, lights_on_image.size());
    EXPECT_EQ(1, lights_outside_image.size());
  }

  // short focus project outside image
  {
    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);

    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    std::string camera_name = "onsemi_obstacle";
    preprocessor_->SetCameraWorkingFlag(camera_name, true);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    pose.c2w_poses_[camera_name](0, 3) += +100000;
    ASSERT_TRUE(preprocessor_->ProjectLights(
        pose, camera_name, &lights, &lights_on_image, &lights_outside_image));
    EXPECT_EQ(0, lights_on_image.size());
    EXPECT_EQ(1, lights_outside_image.size());
  }
}

TEST_F(TLPreprocessorTest, test_select_camera) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  ASSERT_TRUE(preprocessor_->Init(init_options));
  std::map<std::string, int> image_borders_size = {{"onsemi_traffic", 100},
                                                   {"onsemi_obstacle", 100},
                                                   {"onsemi_narrow", 100},
                                                   {"onsemi_wide", 100}};

  {
    CarPose pose;
    TLPreprocessorOption option;
    option.image_borders_size = &image_borders_size;

    int cam_no = 2;
    std::string selection = "unknown";
    std::vector<base::TrafficLightPtrs> lights_on_image_array(cam_no);
    std::vector<base::TrafficLightPtrs> lights_outside_image_array(cam_no);

    // no lights info
    preprocessor_->SelectCamera(&lights_on_image_array,
                                &lights_outside_image_array, option,
                                &selection);
    EXPECT_EQ("unknown", selection);
  }

  {
    CarPose pose;
    int cam_no = 2;
    std::string selection = "unknown";
    std::vector<base::TrafficLightPtrs> lights_on_image_array(cam_no);
    std::vector<base::TrafficLightPtrs> lights_outside_image_array(cam_no);

    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    std::string camera_name = "onsemi_traffic";
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(camera_name, true));
    ASSERT_TRUE(preprocessor_->ProjectLights(pose, camera_name, &lights,
                                             &lights_on_image_array[0],
                                             &lights_outside_image_array[0]));

    AINFO << "lights on image " << lights_on_image_array[0].size();
    AINFO << "lights outside image " << lights_outside_image_array[0].size();

    TLPreprocessorOption option;
    option.image_borders_size = &image_borders_size;

    preprocessor_->SelectCamera(&lights_on_image_array,
                                &lights_outside_image_array, option,
                                &selection);
    EXPECT_EQ("onsemi_traffic", selection);

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(camera_name, false));
    preprocessor_->SelectCamera(&lights_on_image_array,
                                &lights_outside_image_array, option,
                                &selection);
    EXPECT_EQ("onsemi_traffic", selection);
  }

  // test project on image border region
  {
    CarPose pose;
    int cam_no = 4;
    std::string selection = "";
    std::vector<base::TrafficLightPtrs> lights_on_image_array(cam_no);
    std::vector<base::TrafficLightPtrs> lights_outside_image_array(cam_no);

    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    std::string long_cam_id = "onsemi_traffic";
    std::string short_cam_id = "onsemi_obstacle";
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(short_cam_id, true));
    ASSERT_TRUE(preprocessor_->ProjectLights(pose, long_cam_id, &lights,
                                             &lights_on_image_array[0],
                                             &lights_outside_image_array[0]));

    TLPreprocessorOption option;
    option.image_borders_size = &image_borders_size;
    preprocessor_->SelectCamera(&lights_on_image_array,
                                &lights_outside_image_array, option,
                                &selection);
    EXPECT_EQ(long_cam_id, selection);

    image_borders_size[long_cam_id] = 1000;
    std::string wide_cam_id = "onsemi_wide";
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(wide_cam_id, true));
    preprocessor_->SelectCamera(&lights_on_image_array,
                                &lights_outside_image_array, option,
                                &selection);
    EXPECT_EQ(short_cam_id, selection);
    image_borders_size[long_cam_id] = 100;
    image_borders_size[short_cam_id] = 100;
  }
}

TEST_F(TLPreprocessorTest, test_get_max_min_focal_len_camera_id) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  ASSERT_TRUE(preprocessor_->Init(init_options));

  // test get_max_focal_len_camera_id()
  std::string long_camera_id = "onsemi_traffic";
  std::string narrow_camera_id = "onsemi_narrow";
  std::string short_camera_id = "onsemi_obstacle";
  std::string wide_camera_id = "onsemi_wide";

  EXPECT_EQ(preprocessor_->GetMaxFocalLenWorkingCameraName(), "");

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_camera_id, true));
  EXPECT_EQ(long_camera_id, preprocessor_->GetMaxFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_camera_id, true));
  EXPECT_EQ(narrow_camera_id, preprocessor_->GetMaxFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(short_camera_id, true));
  EXPECT_EQ(short_camera_id, preprocessor_->GetMaxFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(short_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(wide_camera_id, true));
  EXPECT_EQ(wide_camera_id, preprocessor_->GetMaxFocalLenWorkingCameraName());

  // test GetMinFocalLenWorkingCameraName()
  EXPECT_EQ(wide_camera_id, preprocessor_->GetMinFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(wide_camera_id, true));
  EXPECT_EQ(wide_camera_id, preprocessor_->GetMinFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(wide_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(short_camera_id, true));
  EXPECT_EQ(short_camera_id, preprocessor_->GetMinFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(short_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_camera_id, true));
  EXPECT_EQ(narrow_camera_id, preprocessor_->GetMinFocalLenWorkingCameraName());

  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_camera_id, false));
  ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_camera_id, true));
  EXPECT_EQ(long_camera_id, preprocessor_->GetMinFocalLenWorkingCameraName());
}

TEST_F(TLPreprocessorTest, invalid_pose_id) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  CarPose pose;
  Eigen::Matrix4d c2w_pose;
  std::vector<base::TrafficLightPtr> lights(1);
  lights[0].reset(new base::TrafficLight);
  PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
  std::string cam_id = "onsemi_traffic";
  pose.ClearCameraPose(cam_id);

  cam_id = "non-exist";
  pose.ClearCameraPose(cam_id);
  ASSERT_FALSE(pose.GetCameraPose(cam_id, &c2w_pose));
  pose.GetCameraPose(cam_id, &c2w_pose);
}

TEST_F(TLPreprocessorTest, invalid_camera_name) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  CarPose pose;
  Eigen::Matrix4d c2w_pose;
  std::vector<base::TrafficLightPtr> lights(1);
  lights[0].reset(new base::TrafficLight);
  PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;

  init_options.camera_names.clear();
  ASSERT_FALSE(preprocessor_->Init(init_options));

  init_options.camera_names.clear();
  init_options.camera_names.push_back("invalid_sensor");
  ASSERT_FALSE(preprocessor_->Init(init_options));

  init_options.camera_names.clear();
  init_options.camera_names.push_back("velodyne64");
  ASSERT_FALSE(preprocessor_->Init(init_options));
}

TEST_F(TLPreprocessorTest, on_board) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  std::string long_cam_id = "onsemi_traffic";
  std::string narrow_cam_id = "onsemi_narrow";
  std::map<std::string, int> image_border_sizes = {{"onsemi_traffic", 100},
                                                   {"onsemi_obstacle", 100},
                                                   {"onsemi_narrow", 100},
                                                   {"onsemi_wide", 100}};
  ASSERT_TRUE(preprocessor_->Init(init_options));
  // no lights
  {
    TLPreprocessorOption option;
    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    lights.clear();

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_TRUE(preprocessor_->UpdateCameraSelection(pose, option, &lights));
  }

  // long focus project outside image
  {
    TLPreprocessorOption option;
    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    pose.c2w_poses_[long_cam_id](0, 3) += 100000;
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_TRUE(preprocessor_->UpdateCameraSelection(pose, option, &lights));
  }

  // long focus project on image
  {
    TLPreprocessorOption option;
    base::TrafficLightPtrs lights_on_image;
    base::TrafficLightPtrs lights_outside_image;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_TRUE(preprocessor_->UpdateCameraSelection(pose, option, &lights));
  }
  double time = 100;
  for (int i = 0; i < 15; ++i) {
    ASSERT_TRUE(preprocessor_->SyncInformation(time, long_cam_id));
    ASSERT_FALSE(preprocessor_->SyncInformation(time, narrow_cam_id));
    ASSERT_FALSE(preprocessor_->SyncInformation(time, ""));
    time += 100;
  }
  // invalid timestamp
  time = 0;
  for (int i = 0; i < 10; ++i) {
    ASSERT_FALSE(preprocessor_->SyncInformation(time, long_cam_id));
    time += 100;
  }
}

TEST_F(TLPreprocessorTest, UpdateLightsProjectionTest) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  TrafficLightPreprocessorInitOptions init_options;
  init_options.conf_file = "preprocess.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/traffic_light/preprocessor/data/";
  init_options.gpu_id = 0;
  init_options.sync_interval_seconds = 0.5;
  init_options.camera_names = camera_names_;
  std::string long_cam_id = "onsemi_traffic";
  std::string narrow_cam_id = "onsemi_narrow";
  std::map<std::string, int> image_border_sizes = {{"onsemi_traffic", 100},
                                                   {"onsemi_obstacle", 100},
                                                   {"onsemi_narrow", 100},
                                                   {"onsemi_wide", 100}};
  ASSERT_TRUE(preprocessor_->Init(init_options));
  // no lights
  {
    TLPreprocessorOption option;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    lights.clear();

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));
    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(narrow_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_TRUE(preprocessor_->UpdateLightsProjection(
        pose, option, "onsemi_traffic", &lights));
  }
  // invalid camera name
  {
    TLPreprocessorOption option;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_FALSE(preprocessor_->UpdateLightsProjection(pose, option, "invalid",
                                                       &lights));
  }
  // project out of valid region
  {
    TLPreprocessorOption option;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));

    image_border_sizes["onsemi_traffic"] = 1000;
    option.image_borders_size = &image_border_sizes;
    ASSERT_FALSE(preprocessor_->UpdateLightsProjection(
        pose, option, "onsemi_traffic", &lights));
    image_border_sizes["onsemi_traffic"] = 100;
  }
  // project out of image
  {
    TLPreprocessorOption option;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));
    lights[0]->region.points[0].x = 0;
    lights[0]->region.points[0].y = 0;

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_FALSE(preprocessor_->UpdateLightsProjection(
        pose, option, "onsemi_traffic", &lights));
  }
  // project out of valid region
  {
    TLPreprocessorOption option;

    CarPose pose;
    std::vector<base::TrafficLightPtr> lights(1);
    lights[0].reset(new base::TrafficLight);
    PrepareTestDataLongFocus(&pose, &(lights[0]->region.points));

    ASSERT_TRUE(preprocessor_->SetCameraWorkingFlag(long_cam_id, true));

    option.image_borders_size = &image_border_sizes;
    ASSERT_TRUE(preprocessor_->UpdateLightsProjection(
        pose, option, "onsemi_traffic", &lights));
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
