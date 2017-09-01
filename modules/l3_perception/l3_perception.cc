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
#include "modules/l3_perception/l3_perception.h"

#include "ros/include/ros/ros.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/l3_perception/l3_perception_gflags.h"

#include <cmath>

namespace apollo {
namespace l3_perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::drivers::Mobileye;
using apollo::drivers::DelphiESR;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;

std::string L3Perception::Name() const {
  return FLAGS_hmi_name;
}

Status L3Perception::Init() {
  AdapterManager::Init(FLAGS_adapter_config_path);

  CHECK(AdapterManager::GetMobileye()) << "Mobileye is not initialized.";
  AdapterManager::AddMobileyeCallback(&L3Perception::OnMobileye, this);
  CHECK(AdapterManager::GetDelphiESR()) << "DelphiESR is not initialized.";
  AdapterManager::AddDelphiESRCallback(&L3Perception::OnDelphiESR, this);
  CHECK(AdapterManager::GetLocalization()) << "Localization is not initialized.";
  AdapterManager::AddLocalizationCallback(&L3Perception::OnLocalization, this);

  return Status::OK();
}

Status L3Perception::Start() {
  const double duration = 1.0 / FLAGS_l3_perception_freq;
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(duration), &L3Perception::OnTimer, this);

  return Status::OK();
}

void L3Perception::Stop() {
  timer_.stop();
}

void L3Perception::OnMobileye(const Mobileye& message) {
  AINFO << "receive Mobileye callback";
  mobileye_.CopyFrom(message);
}

void L3Perception::OnDelphiESR(const DelphiESR& message) {
  AINFO << "receive DelphiESR callback";
  delphi_esr_.CopyFrom(message);
}

void L3Perception::OnLocalization(const LocalizationEstimate& message) {
  AINFO << "receive Localization callback";
  localization_.CopyFrom(message);
}

PerceptionObstacles L3Perception::ConvertToPerceptionObstacles(
    const Mobileye& mobileye, const LocalizationEstimate& localization) {
  PerceptionObstacles obstacles;
  double adc_x = localization.pose().position().x();
  double adc_y = localization.pose().position().y();
  double adc_z = localization.pose().position().z();
  auto adc_quaternion = localization.pose().orientation();
  double adc_vx = localization.pose().linear_velocity().x();
  double adc_vy = localization.pose().linear_velocity().y();
  double adc_velocity = std::sqrt(adc_vx * adc_vx + adc_vy * adc_vy);

  double adc_theta =
      std::atan2(2.0 * adc_quaternion.qw() * adc_quaternion.qz() + 
        adc_quaternion.qx() * adc_quaternion.qy(),
        1.0 - 2.0 * (adc_quaternion.qy() * adc_quaternion.qy() +
                     adc_quaternion.qz() * adc_quaternion.qz())) +
      acos(-1.0) / 2.0;

  for (int index = 0; index < mobileye.details_738().num_obstacles(); ++index) {
    auto* mob = obstacles.add_perception_obstacle();
    const auto& data_739 = mobileye.details_739(index);
    const auto& data_73a = mobileye.details_73a(index);
    int mob_id = data_739.obstacle_id();
    double mob_pos_x = data_739.obstacle_pos_x();
    double mob_pos_y = - data_739.obstacle_pos_y();
    double mob_vel_x = data_739.obstacle_rel_vel_x();
    int mob_type = data_739.obstacle_type();

    double mob_l = data_73a.obstacle_length();
    double mob_w = data_73a.obstacle_width();

    mob_pos_x += 3;

    double converted_x = adc_x + mob_pos_x * std::cos(adc_theta) + mob_pos_y * std::sin(adc_theta);
    double converted_y = adc_y + mob_pos_x * std::sin(adc_theta) - mob_pos_y * std::cos(adc_theta);
    double converted_speed = adc_velocity + mob_vel_x;
    double converted_vx = converted_speed * std::cos(adc_theta);
    double converted_vy = converted_speed * std::sin(adc_theta);

    mob->set_id(mob_id);
    mob->mutable_position()->set_x(converted_x);
    mob->mutable_position()->set_y(converted_y);

    switch (mob_type) {
      case 0:
      case 1: {
          mob->set_type(PerceptionObstacle::VEHICLE); // VEHICLE
          break;
      }
      case 2:
      case 4: {
          mob->set_type(PerceptionObstacle::BICYCLE); // BIKE
          break;
      }
      case 3: {
          mob->set_type(PerceptionObstacle::PEDESTRIAN); // PED
          break;
      }
      default: {
          mob->set_type(PerceptionObstacle::UNKNOWN); // UNKNOWN
          break;
      }
    }

    mob->mutable_velocity()->set_x(converted_vx);
    mob->mutable_velocity()->set_y(converted_vy);
    mob->set_length(mob_l);
    mob->set_width(mob_w);
    mob->set_theta(std::atan2(converted_vy, converted_vx));
    mob->set_height(3.0);

    mob->clear_polygon_point();
    double x0 = converted_x;
    double y0 = converted_y;
    double z0 = 0.0;
    double heading = mob->theta();
    auto* p1_upper = mob->add_polygon_point();
    p1_upper->set_x(x0 + mob_l * std::cos(heading) / 2.0 + mob_w * std::sin(heading) / 2.0);
    p1_upper->set_y(y0 + mob_l * std::sin(heading) / 2.0 - mob_w * std::cos(heading) / 2.0);
    p1_upper->set_z(z0 + mob->height() / 2.0);
    auto* p1_lower = mob->add_polygon_point();
    p1_lower->set_x(x0 + mob_l * std::cos(heading) / 2.0 + mob_w * std::sin(heading) / 2.0);
    p1_lower->set_y(y0 + mob_l * std::sin(heading) / 2.0 - mob_w * std::cos(heading) / 2.0);
    p1_lower->set_z(z0 - mob->height() / 2.0);
    auto* p2_upper = mob->add_polygon_point();
    p2_upper->set_x(x0 + mob_l * std::cos(heading) / 2.0 - mob_w * std::sin(heading) / 2.0);
    p2_upper->set_y(y0 + mob_l * std::sin(heading) / 2.0 + mob_w * std::cos(heading) / 2.0);
    p2_upper->set_z(z0 + mob->height() / 2.0);
    auto* p2_lower = mob->add_polygon_point();
    p2_lower->set_x(x0 + mob_l * std::cos(heading) / 2.0 - mob_w * std::sin(heading) / 2.0);
    p2_lower->set_y(y0 + mob_l * std::sin(heading) / 2.0 + mob_w * std::cos(heading) / 2.0);
    p2_lower->set_z(z0 - mob->height() / 2.0);
    auto* p3_upper = mob->add_polygon_point();
    p3_upper->set_x(x0 - mob_l * std::cos(heading) / 2.0 - mob_w * std::sin(heading) / 2.0);
    p3_upper->set_y(y0 - mob_l * std::sin(heading) / 2.0 + mob_w * std::cos(heading) / 2.0);
    p3_upper->set_z(z0 + mob->height() / 2.0);
    auto* p3_lower = mob->add_polygon_point();
    p3_lower->set_x(x0 - mob_l * std::cos(heading) / 2.0 - mob_w * std::sin(heading) / 2.0);
    p3_lower->set_y(y0 - mob_l * std::sin(heading) / 2.0 + mob_w * std::cos(heading) / 2.0);
    p3_lower->set_z(z0 - mob->height() / 2.0);
    auto* p4_upper = mob->add_polygon_point();
    p4_upper->set_x(x0 - mob_l * std::cos(heading) / 2.0 + mob_w * std::sin(heading) / 2.0);
    p4_upper->set_y(y0 - mob_l * std::sin(heading) / 2.0 - mob_w * std::cos(heading) / 2.0);
    p4_upper->set_z(z0 + mob->height() / 2.0);
    auto* p4_lower = mob->add_polygon_point();
    p4_lower->set_x(x0 - mob_l * std::cos(heading) / 2.0 + mob_w * std::sin(heading) / 2.0);
    p4_lower->set_y(y0 - mob_l * std::sin(heading) / 2.0 - mob_w * std::cos(heading) / 2.0);
    p4_lower->set_z(z0 - mob->height() / 2.0);
  }

  return obstacles;
}

void L3Perception::OnTimer(const ros::TimerEvent&) {
  AINFO << "publish PerceptionObstacles";

  apollo::perception::PerceptionObstacles obstacles;

  if (mobileye_.header().timestamp_sec() >= last_timestamp_) {
    apollo::perception::PerceptionObstacles mobileye_obstacles = ConvertToPerceptionObstacles(mobileye_, localization_);
    obstacles.MergeFrom(mobileye_obstacles);
  }

  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_node_name, &obstacles);
  AdapterManager::PublishPerceptionObstacles(obstacles);
  last_timestamp_ = apollo::common::time::Clock::NowInSecond();
}

}  // namespace l3_perception
}  // namespace apollo
