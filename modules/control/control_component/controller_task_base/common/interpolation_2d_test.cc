/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/control_component/controller_task_base/common/interpolation_2d.h"

#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/control/control_component/proto/calibration_table.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace control {

class Interpolation2DTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string control_base_dir =
        "/apollo/modules/control/control_component/";
    std::string control_conf_file =
        control_base_dir + "conf/calibration_table.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(control_conf_file,
                                           &calibration_table_));
  }

 protected:
  calibration_table calibration_table_;
};

TEST_F(Interpolation2DTest, normal) {
  Interpolation2D::DataType xyz{std::make_tuple(0.3, 0.2, 0.6),
                                std::make_tuple(10.1, 15.2, 5.5),
                                std::make_tuple(20.2, 10.3, 30.5)};

  Interpolation2D estimator;
  EXPECT_TRUE(estimator.Init(xyz));

  for (unsigned i = 0; i < xyz.size(); i++) {
    EXPECT_DOUBLE_EQ(std::get<2>(xyz[i]),
                     estimator.Interpolate(std::make_pair(
                         std::get<0>(xyz[i]), std::get<1>(xyz[i]))));
  }

  EXPECT_DOUBLE_EQ(4.7000000000000002,
                   estimator.Interpolate(std::make_pair(8.5, 14)));
  EXPECT_DOUBLE_EQ(26.292079207920793,
                   estimator.Interpolate(std::make_pair(18.5, 12)));

  // out of range
  EXPECT_DOUBLE_EQ(0.59999999999999998,
                   estimator.Interpolate(std::make_pair(-5, 12)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(30, 12)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(30, -0.5)));
  EXPECT_DOUBLE_EQ(5.4500000000000002,
                   estimator.Interpolate(std::make_pair(10, -0.5)));
  EXPECT_DOUBLE_EQ(5.4500000000000002,
                   estimator.Interpolate(std::make_pair(10, 40)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(40, 40)));
}

TEST_F(Interpolation2DTest, calibration_table) {
  const auto &calibration_table = calibration_table_;
  AINFO << "Throttle calibration table:" << calibration_table.DebugString();

  Interpolation2D::DataType xyz;

  for (const auto &calibration : calibration_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  Interpolation2D estimator;
  EXPECT_TRUE(estimator.Init(xyz));

  for (const auto &elem : xyz) {
    EXPECT_DOUBLE_EQ(std::get<2>(elem),
                     estimator.Interpolate(
                         std::make_pair(std::get<0>(elem), std::get<1>(elem))));
  }
}

}  // namespace control
}  // namespace apollo
