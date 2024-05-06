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

#include "modules/control/control_component/controller_task_base/common/interpolation_1d.h"

#include <string>
#include <utility>

#include "gmock/gmock.h"

#include "modules/control/controllers/lat_based_lqr_controller/proto/lat_based_lqr_controller_conf.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

using ::testing::ElementsAre;

namespace apollo {
namespace control {

class Interpolation1DTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string controllers_dir = "/apollo/modules/control/controllers/";
    std::string control_conf_file =
        controllers_dir +
        "lat_based_lqr_controller/conf/controller_conf.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(control_conf_file, &control_conf_));
  }

 protected:
  LatBaseLqrControllerConf control_conf_;
};

TEST_F(Interpolation1DTest, normal) {
  Interpolation1D::DataType xy{{0, 0}, {15, 12}, {30, 17}};

  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  constexpr double episilon = 1.0e-10;
  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_NEAR(xy[i].second, estimator.Interpolate(xy[i].first), episilon);
  }

  EXPECT_DOUBLE_EQ(4.7777777777777777, estimator.Interpolate(5));
  EXPECT_DOUBLE_EQ(8.7777777777777786, estimator.Interpolate(10));
  EXPECT_DOUBLE_EQ(14.444444444444445, estimator.Interpolate(20));

  // out of x range
  EXPECT_DOUBLE_EQ(0, estimator.Interpolate(-1));
  EXPECT_DOUBLE_EQ(17, estimator.Interpolate(30));
}

TEST_F(Interpolation1DTest, unordered) {
  Interpolation1D::DataType xy{{15, 12}, {5, 5}, {40, 25}, {30, 17}};

  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, estimator.Interpolate(xy[i].first));
  }
}

TEST_F(Interpolation1DTest, gain_scheduler) {
  const auto& gain_scheduler = control_conf_.lat_err_gain_scheduler();
  AINFO << "Lateral Error Gain Scheduler:" << gain_scheduler.DebugString();

  Interpolation1D::DataType xy;

  for (const auto& scheduler : gain_scheduler.scheduler()) {
    xy.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, estimator.Interpolate(xy[i].first));
  }
}

}  // namespace control
}  // namespace apollo
