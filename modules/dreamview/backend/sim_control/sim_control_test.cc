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

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/math/quaternion.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/time/time.h"

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
  SimControlTest()
      : map_service_("modules/dreamview/backend/testdata/garage.bin"),
        sim_control_(&map_service_) {
    FLAGS_enable_sim_control = false;

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

    AdapterManager::Init(config);
  }

 protected:
  MapService map_service_;
  SimControl sim_control_;
};

void SetTrajectory(const std::vector<double> &xs, const std::vector<double> &ys,
                   const std::vector<double> &vs, const std::vector<double> &as,
                   const std::vector<double> &ths,
                   const std::vector<double> &ks, const std::vector<double> &ts,
                   planning::ADCTrajectory *adc_trajectory) {
  for (size_t i = 0; i < xs.size(); ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xs[i]);
    point->mutable_path_point()->set_y(ys[i]);
    point->set_v(vs[i]);
    point->set_a(as[i]);
    point->mutable_path_point()->set_theta(ths[i]);
    point->mutable_path_point()->set_kappa(ks[i]);
    point->set_relative_time(ts[i]);
  }
}

TEST_F(SimControlTest, Test) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> vs = {40.0, 50.0, 60.0, 70.0, 80.0};
  std::vector<double> as = {40.0, 50.0, 60.0, 70.0, 80.0};
  std::vector<double> ths = {0.2, 0.6, 0.8, 1.0, 1.5};
  std::vector<double> ks = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
  SetTrajectory(xs, ys, vs, as, ths, ks, ts, &adc_trajectory);

  const double timestamp = 100.0;
  adc_trajectory.mutable_header()->set_timestamp_sec(timestamp);

  RoutingResponse routing;
  routing.mutable_routing_request()->mutable_start()->mutable_pose()->set_x(
      1.0);
  routing.mutable_routing_request()->mutable_start()->mutable_pose()->set_y(
      1.0);

  sim_control_.SetStartPoint(routing);

  AdapterManager::AddPlanningCallback(&SimControl::OnPlanning, &sim_control_);
  AdapterManager::GetPlanning()->OnReceive(adc_trajectory);

  {
    Clock::UseSystemClock(false);
    const auto timestamp = apollo::common::time::From(100.01);
    Clock::SetNow(timestamp.time_since_epoch());
    sim_control_.TimerCallback(ros::TimerEvent());

    const Chassis *chassis = AdapterManager::GetChassis()->GetLatestPublished();
    const LocalizationEstimate *localization =
        AdapterManager::GetLocalization()->GetLatestPublished();

    EXPECT_EQ(chassis->engine_started(), true);
    EXPECT_EQ(chassis->driving_mode(), Chassis::COMPLETE_AUTO_DRIVE);
    EXPECT_EQ(chassis->gear_location(), Chassis::GEAR_DRIVE);

    EXPECT_NEAR(chassis->speed_mps(), 41.0, 1e-6);
    EXPECT_NEAR(chassis->throttle_percentage(), 0.0, 1e-6);
    EXPECT_NEAR(chassis->brake_percentage(), 0.0, 1e-6);

    const auto &pose = localization->pose();
    EXPECT_NEAR(pose.position().x(), 1.01, 1e-6);
    EXPECT_NEAR(pose.position().y(), 1.01, 1e-6);
    EXPECT_NEAR(pose.position().z(), 0.0, 1e-6);

    const double theta = 0.24;
    EXPECT_NEAR(pose.heading(), theta, 1e-6);

    const Eigen::Quaternion<double> orientation =
        HeadingToQuaternion<double>(theta);
    EXPECT_NEAR(pose.orientation().qw(), orientation.w(), 1e-6);
    EXPECT_NEAR(pose.orientation().qx(), orientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation().qy(), orientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation().qz(), orientation.z(), 1e-6);

    const double speed = 41.0;
    EXPECT_NEAR(pose.linear_velocity().x(), std::cos(theta) * speed, 1e-6);
    EXPECT_NEAR(pose.linear_velocity().y(), std::sin(theta) * speed, 1e-6);
    EXPECT_NEAR(pose.linear_velocity().z(), 0.0, 1e-6);

    const double curvature = 1.1;
    EXPECT_NEAR(pose.angular_velocity().x(), 0.0, 1e-6);
    EXPECT_NEAR(pose.angular_velocity().y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.angular_velocity().z(), speed * curvature, 1e-6);

    const double acceleration_s = 41.0;
    EXPECT_NEAR(pose.linear_acceleration().x(),
                std::cos(theta) * acceleration_s, 1e-6);
    EXPECT_NEAR(pose.linear_acceleration().y(),
                std::sin(theta) * acceleration_s, 1e-6);
    EXPECT_NEAR(pose.linear_acceleration().z(), 0.0, 1e-6);
  }
}

}  // namespace dreamview
}  // namespace apollo
