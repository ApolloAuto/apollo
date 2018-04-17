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

#include "modules/dreamview/backend/sim_control/sim_control.h"

#include <cmath>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/proto/adapter_config.pb.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/time/time.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingResponse;

namespace apollo {
namespace dreamview {

class SimControlTest : public ::testing::Test {
 public:
  SimControlTest() {
    FLAGS_map_dir = "modules/dreamview/backend/testdata";
    FLAGS_base_map_filename = "garage.bin";

    map_service_.reset(new MapService(false));
    sim_control_.reset(new SimControl(map_service_.get()));

    AdapterManagerConfig config;
    config.set_is_ros(false);
    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(
          apollo::common::adapter::AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(apollo::common::adapter::AdapterConfig::CHASSIS);
    }

    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(
          apollo::common::adapter::AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(
          apollo::common::adapter::AdapterConfig::LOCALIZATION);
    }

    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(
          apollo::common::adapter::AdapterConfig::RECEIVE_ONLY);
      sub_config->set_type(
          apollo::common::adapter::AdapterConfig::PLANNING_TRAJECTORY);
    }

    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(
          apollo::common::adapter::AdapterConfig::RECEIVE_ONLY);
      sub_config->set_type(
          apollo::common::adapter::AdapterConfig::ROUTING_RESPONSE);
    }

    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(
          apollo::common::adapter::AdapterConfig::RECEIVE_ONLY);
      sub_config->set_type(apollo::common::adapter::AdapterConfig::NAVIGATION);
    }

    AdapterManager::Init(config);

    sim_control_->Start();
  }

 protected:
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<SimControl> sim_control_;
};

void SetTrajectory(const std::vector<double> &xs, const std::vector<double> &ys,
                   const std::vector<double> &ss, const std::vector<double> &vs,
                   const std::vector<double> &as,
                   const std::vector<double> &ths,
                   const std::vector<double> &ks, const std::vector<double> &ts,
                   planning::ADCTrajectory *adc_trajectory) {
  for (size_t i = 0; i < xs.size(); ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xs[i]);
    point->mutable_path_point()->set_y(ys[i]);
    point->mutable_path_point()->set_s(ss[i]);
    point->set_v(vs[i]);
    point->set_a(as[i]);
    point->mutable_path_point()->set_theta(ths[i]);
    point->mutable_path_point()->set_kappa(ks[i]);
    point->set_relative_time(ts[i]);
  }
}

TEST_F(SimControlTest, Test) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs(5);
  std::vector<double> ys(5);
  std::vector<double> ss(5);
  std::vector<double> vs(5);
  std::vector<double> as = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ths = {M_PI / 4.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0,
                             M_PI / 4.0};
  std::vector<double> kappa_s = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
  ss[0] = 0.0;
  xs[0] = 0.0;
  ys[0] = 0.0;
  vs[0] = 10.0;
  for (std::size_t i = 1; i < ts.size(); ++i) {
    vs[i] = vs[i - 1] + as[i - 1] * ts[i];
    ss[i] = (vs[i - 1] + 0.5 * vs[i]) * ts[i];
    xs[i] = std::sqrt(ss[i] * ss[i] / 2.0);
    ys[i] = std::sqrt(ss[i] * ss[i] / 2.0);
  }

  SetTrajectory(xs, ys, ss, vs, as, ths, kappa_s, ts, &adc_trajectory);

  const double timestamp = 100.0;
  adc_trajectory.mutable_header()->set_timestamp_sec(timestamp);

  sim_control_->SetStartPoint(adc_trajectory.trajectory_point(0));
  AdapterManager::PublishPlanning(adc_trajectory);

  {
    Clock::SetMode(Clock::MOCK);
    const auto timestamp = apollo::common::time::From(100.01);
    Clock::SetNow(timestamp.time_since_epoch());
    sim_control_->RunOnce();

    const Chassis *chassis = AdapterManager::GetChassis()->GetLatestPublished();
    const LocalizationEstimate *localization =
        AdapterManager::GetLocalization()->GetLatestPublished();

    EXPECT_TRUE(chassis->engine_started());
    EXPECT_EQ(Chassis::COMPLETE_AUTO_DRIVE, chassis->driving_mode());
    EXPECT_EQ(Chassis::GEAR_DRIVE, chassis->gear_location());

    EXPECT_NEAR(chassis->speed_mps(), 10.0, 1e-6);
    EXPECT_NEAR(chassis->throttle_percentage(), 0.0, 1e-6);
    EXPECT_NEAR(chassis->brake_percentage(), 0.0, 1e-6);

    const auto &pose = localization->pose();
    EXPECT_NEAR(pose.position().x(), 0.10606601717803638, 1e-6);
    EXPECT_NEAR(pose.position().y(), 0.10606601717803638, 1e-6);
    EXPECT_NEAR(pose.position().z(), 0.0, 1e-6);

    const double theta = M_PI / 4.0;
    EXPECT_NEAR(pose.heading(), theta, 1e-6);

    const Eigen::Quaternion<double> orientation =
        HeadingToQuaternion<double>(theta);
    EXPECT_NEAR(pose.orientation().qw(), orientation.w(), 1e-6);
    EXPECT_NEAR(pose.orientation().qx(), orientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation().qy(), orientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation().qz(), orientation.z(), 1e-6);

    const double speed = 10.0;
    EXPECT_NEAR(pose.linear_velocity().x(), std::cos(theta) * speed, 1e-6);
    EXPECT_NEAR(pose.linear_velocity().y(), std::sin(theta) * speed, 1e-6);
    EXPECT_NEAR(pose.linear_velocity().z(), 0.0, 1e-6);

    const double curvature = 0.0;
    EXPECT_NEAR(pose.angular_velocity().x(), 0.0, 1e-6);
    EXPECT_NEAR(pose.angular_velocity().y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.angular_velocity().z(), speed * curvature, 1e-6);

    const double acceleration_s = 0.0;
    EXPECT_NEAR(pose.linear_acceleration().x(),
                std::cos(theta) * acceleration_s, 1e-6);
    EXPECT_NEAR(pose.linear_acceleration().y(),
                std::sin(theta) * acceleration_s, 1e-6);
    EXPECT_NEAR(pose.linear_acceleration().z(), 0.0, 1e-6);
  }
}

}  // namespace dreamview
}  // namespace apollo
