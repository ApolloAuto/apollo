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
#include "modules/perception/common/algorithm/io/io_util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace algorithm {

TEST(CommonIoTest, read_pose_file) {
  const std::string testdata_folder =
      "/apollo/modules/perception/testdata/common/io/params/";
  std::string filename;
  Eigen::Affine3d pose;
  int frame_id = 0;
  double time_stamp = 0;

  EXPECT_FALSE(ReadPoseFile(filename, nullptr, nullptr, nullptr));
  EXPECT_FALSE(ReadPoseFile(filename, &pose, &frame_id, &time_stamp));

  filename = testdata_folder + "pose_file.txt";
  EXPECT_TRUE(ReadPoseFile(filename, &pose, &frame_id, &time_stamp));
}

TEST(CommonIoTest, load_camera_intrinsic) {
  const std::string testdata_folder =
      "/apollo/modules/perception/testdata/common/io/params/";
  std::string yaml_file;
  base::BrownCameraDistortionModel model;

  EXPECT_FALSE(LoadBrownCameraIntrinsic(yaml_file, nullptr));
  EXPECT_FALSE(LoadBrownCameraIntrinsic(yaml_file, &model));

  yaml_file = testdata_folder + "empty.yaml";
  EXPECT_FALSE(LoadBrownCameraIntrinsic(yaml_file, &model));

  yaml_file = testdata_folder + "test.yaml";
  EXPECT_TRUE(LoadBrownCameraIntrinsic(yaml_file, &model));
  EXPECT_EQ(model.width_, 1192);
  EXPECT_EQ(model.height_, 712);
}

TEST(CommonIoTest, load_ocamera_intrinsic) {
  const std::string testdata_folder =
      "/apollo/modules/perception/testdata/common/io/params/";
  std::string yaml_file;
  base::OmnidirectionalCameraDistortionModel model;

  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, nullptr));
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));

  yaml_file = testdata_folder + "empty.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));

  yaml_file = testdata_folder + "test_ocam.yaml";
  EXPECT_TRUE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  EXPECT_EQ(model.width_, 1920);
  EXPECT_EQ(model.height_, 1080);

  yaml_file = testdata_folder + "test_ocam1.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam2.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam3.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam4.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam5.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam6.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam7.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
  yaml_file = testdata_folder + "test_ocam8.yaml";
  EXPECT_FALSE(LoadOmnidirectionalCameraIntrinsics(yaml_file, &model));
}

TEST(CommonIoTest, GetFileList) {
  std::string path = "/apollo/modules/perception/testdata/lib/data";
  std::vector<std::string> files;
  EXPECT_TRUE(GetFileList(path, "", &files));
  EXPECT_FALSE(GetFileList("/not_exist_path", "", &files));
  EXPECT_TRUE(GetFileList("/apollo/modules/perception/testdata/lib/data", "txt",
                          &files));
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
