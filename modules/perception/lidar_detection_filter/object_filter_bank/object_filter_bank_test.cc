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

#include "modules/perception/lidar_detection_filter/object_filter_bank/object_filter_bank.h"

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"

DECLARE_string(work_root);

namespace apollo {
namespace perception {
namespace lidar {

class MockObjectFilter1 : public BaseObjectFilter {
 public:
  MockObjectFilter1() = default;

  virtual ~MockObjectFilter1() = default;

  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override {
    return true;
  }

  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override {
    return false;
  }

  std::string Name() const override { return "MockObjectFilter1"; }
};  // class MockObjectFilter1

class MockObjectFilter2 : public BaseObjectFilter {
 public:
  MockObjectFilter2() = default;

  virtual ~MockObjectFilter2() = default;

  bool Init(const ObjectFilterInitOptions& options =
                ObjectFilterInitOptions()) override {
    return false;
  }

  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame) override {
    return false;
  }

  std::string Name() const override { return "MockObjectFilter2"; }
};  // class MockObjectFilter

TEST(LidarLibObjectFilterBankTest, lidar_lib_object_filter_bank_test) {
  // FIXME(perception): fix missing data files
  return;

  char cyber_path[100] = "CYBER_PATH=";
  putenv(cyber_path);
  char module_path[100] = "MODULE_PATH=";
  putenv(module_path);
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "lidar/lib/object_filter_bank/filter_bank";

  ObjectFilterBank filter_bank;
  EXPECT_EQ(filter_bank.Name(), "ObjectFilterBank");
  EXPECT_TRUE(filter_bank.Init());
  EXPECT_EQ(filter_bank.Size(), 3);
  LidarFrame frame;
  ObjectFilterOptions option;
  EXPECT_TRUE(filter_bank.Filter(option, &frame));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
