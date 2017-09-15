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

#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/l3_perception/l3_perception_gflags.h"
#include "modules/l3_perception/l3_perception_util.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace l3_perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::drivers::Mobileye;
using apollo::drivers::DelphiESR;
using apollo::drivers::Esr_track01_500;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;

std::string L3Perception::Name() const { return FLAGS_hmi_name; }

Status L3Perception::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetMobileye()) << "Mobileye is not initialized.";
  AdapterManager::AddMobileyeCallback(&L3Perception::OnMobileye, this);
  CHECK(AdapterManager::GetDelphiESR()) << "DelphiESR is not initialized.";
  AdapterManager::AddDelphiESRCallback(&L3Perception::OnDelphiESR, this);
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";
  AdapterManager::AddLocalizationCallback(&L3Perception::OnLocalization, this);

  return Status::OK();
}

Status L3Perception::Start() {
  const double duration = 1.0 / FLAGS_l3_perception_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &L3Perception::OnTimer, this);

  return Status::OK();
}

void L3Perception::Stop() { timer_.stop(); }

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

  double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < mobileye.details_738().num_obstacles() &&
                      index < mobileye.details_739_size();
       ++index) {
    auto* mob = obstacles.add_perception_obstacle();
    const auto& data_739 = mobileye.details_739(index);
    int mob_id = data_739.obstacle_id();
    double mob_pos_x = data_739.obstacle_pos_x();
    double mob_pos_y = -data_739.obstacle_pos_y();
    double mob_vel_x = data_739.obstacle_rel_vel_x();
    int mob_type = data_739.obstacle_type();

    double mob_l = GetDefaultObjectLength(mob_type);

    double mob_w = 0.0;
    if (mobileye.details_73a_size() <= index) {
      mob_l = GetDefaultObjectWidth(mob_type);
    } else {
      mob_w = mobileye.details_73a(index).obstacle_width();
    }

    // TODO(lizh): calibrate mobileye and make those consts FLAGS
    mob_pos_x += FLAGS_mobileye_pos_adjust;  // offset: imu <-> mobileye
    mob_pos_x += mob_l / 2.0; // make x the middle point of the vehicle.

    double converted_x = adc_x + mob_pos_x * std::cos(adc_theta) +
                         mob_pos_y * std::sin(adc_theta);
    double converted_y = adc_y + mob_pos_x * std::sin(adc_theta) -
                         mob_pos_y * std::cos(adc_theta);
    double converted_speed = adc_velocity + mob_vel_x;
    double converted_vx = converted_speed * std::cos(adc_theta);
    double converted_vy = converted_speed * std::sin(adc_theta);

    mob->set_id(mob_id);
    mob->mutable_position()->set_x(converted_x);
    mob->mutable_position()->set_y(converted_y);

    switch (mob_type) {
      case 0:
      case 1: {
        mob->set_type(PerceptionObstacle::VEHICLE);  // VEHICLE
        break;
      }
      case 2:
      case 4: {
        mob->set_type(PerceptionObstacle::BICYCLE);  // BIKE
        break;
      }
      case 3: {
        mob->set_type(PerceptionObstacle::PEDESTRIAN);  // PED
        break;
      }
      default: {
        mob->set_type(PerceptionObstacle::UNKNOWN);  // UNKNOWN
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
    double mid_x = converted_x;
    double mid_y = converted_y;
    double mid_z = adc_z / 2.0;
    double heading = mob->theta();

    FillPerceptionPolygon(mob, mid_x, mid_y, mid_z, mob_l, mob_w, mob->height(), heading);
  }

  return obstacles;
}

PerceptionObstacles L3Perception::ConvertToPerceptionObstacles(
    const DelphiESR& delphi_esr, const LocalizationEstimate& localization) {
  PerceptionObstacles obstacles;
  double adc_x = localization.pose().position().x();
  double adc_y = localization.pose().position().y();
  double adc_z = localization.pose().position().z();
  auto adc_quaternion = localization.pose().orientation();
  double adc_vx = localization.pose().linear_velocity().x();
  double adc_vy = localization.pose().linear_velocity().y();
  double adc_velocity = std::sqrt(adc_vx * adc_vx + adc_vy * adc_vy);

  double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < delphi_esr.esr_track01_500_size(); ++index) {
    auto* mob = obstacles.add_perception_obstacle();
    const auto& data_500 = delphi_esr.esr_track01_500(index);
    //TODO(lizh): object id
    int mob_id = index;
    double mob_range = data_500.can_tx_track_range();
    double mob_angle = data_500.can_tx_track_angle();
    double mob_pos_x = mob_range * std::cos(mob_angle * L3_PI / 180.0);
    double mob_pos_y = mob_range * std::sin(mob_angle * L3_PI / 180.0);
    double mob_range_vel = data_500.can_tx_track_range_rate();
    double mob_vel_x = mob_range_vel * std::cos(mob_angle * L3_PI / 180.0);
    double mob_vel_y = mob_range_vel * std::sin(mob_angle * L3_PI / 180.0);

    double mob_l = GetDefaultObjectLength(4);

    double mob_w = GetDefaultObjectWidth(4);

    // TODO(lizh): calibrate mobileye and make those consts FLAGS
    mob_pos_x += delphi_esr_adjust;  // offset: imu <-> mobileye
    mob_pos_x += mob_l / 2.0; // make x the middle point of the vehicle.

    double converted_x = adc_x + mob_pos_x * std::cos(adc_theta) +
                         mob_pos_y * std::sin(adc_theta);
    double converted_y = adc_y + mob_pos_x * std::sin(adc_theta) -
                         mob_pos_y * std::cos(adc_theta);
    double converted_speed = std::sqrt((adc_velocity + mob_vel_x) * 
                             (adc_velocity + mob_vel_x) + mob_vel_y * mob_vel_y);
    double converted_vx = converted_speed * std::cos(adc_theta);
    double converted_vy = converted_speed * std::sin(adc_theta);

    mob->set_id(mob_id);
    mob->mutable_position()->set_x(converted_x);
    mob->mutable_position()->set_y(converted_y);

    mob->set_type(PerceptionObstacle::UNKNOWN);  // UNKNOWN

    mob->mutable_velocity()->set_x(converted_vx);
    mob->mutable_velocity()->set_y(converted_vy);
    mob->set_length(mob_l);
    mob->set_width(mob_w);
    mob->set_theta(std::atan2(converted_vy, converted_vx));
    mob->set_height(3.0);

    mob->clear_polygon_point();
    double mid_x = converted_x;
    double mid_y = converted_y;
    double mid_z = adc_z / 2.0;
    double heading = mob->theta();

    FillPerceptionPolygon(mob, mid_x, mid_y, mid_z, mob_l, mob_w, mob->height(), heading);
  }

  return obstacles;
}

bool IsPreserved(const PerceptionObstacle& perception_obstacle, const Esr_track01_500& esr_track01_500, const double& rcs) {
/*
  if (sqrt(perception_obstacle.velocity().x() *
               perception_obstacle.velocity().x() +
           perception_obstacle.velocity().y() *
               perception_obstacle.velocity().y()) < 5.0) {
    return false;
  }
  if (rcs < -1.0) {
    return false;
  }
  if (esr_track01_500.can_tx_track_range() > 40.0) {
    return false;
  }
  if (esr_track01_500.can_tx_track_angle() > 25.0 ||
      esr_track01_500.can_tx_track_angle() < -25.0) {
    return false;
  }
*/
  if (esr_track01_500.can_tx_track_status() == ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET ||
    esr_track01_500.can_tx_track_status() == ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_COASTED_TARGET ) {
    return false;
  }
return true;
}

PerceptionObstacles L3Perception::FilterDelphiEsrPerceptionObstacles(
    const PerceptionObstacles& perception_obstacles,
    const DelphiESR& delphi_esr) {
  std::vector<apollo::drivers::Esr_trackmotionpower_540::Motionpower> motionpowers(64);
  for (const auto& esr_trackmotionpower_540 : delphi_esr.esr_trackmotionpower_540()) {
    const int& can_tx_track_can_id_group = esr_trackmotionpower_540.can_tx_track_can_id_group();
    for (int index = 0; index < (can_tx_track_can_id_group < 9 ? 7 : 1); ++index) {
      motionpowers[can_tx_track_can_id_group * 7 + index].CopyFrom(esr_trackmotionpower_540.can_tx_track_motion_power(index));
    }
  }

  PerceptionObstacles filtered_perception_obstacles;
  for (int index = 0; index < perception_obstacles.perception_obstacle_size(); ++index) {
    const auto& perception_obstacle = perception_obstacles.perception_obstacle(index);
    const auto& esr_track01_500 = delphi_esr.esr_track01_500(index);
    const auto& rcs = static_cast<double>(motionpowers[index].can_tx_track_power()) - 10.0;
    if (IsPreserved(perception_obstacle, esr_track01_500, rcs)) {
      auto* obstacle = filtered_perception_obstacles.add_perception_obstacle();
      obstacle->CopyFrom(perception_obstacle);
    }
  }
  return filtered_perception_obstacles;
}

void L3Perception::OnTimer(const ros::TimerEvent&) {
  AINFO << "publish PerceptionObstacles";

  PerceptionObstacles obstacles;

  // TODO(lizh): check timestamp before publish.
  // if (mobileye_.header().timestamp_sec() >= last_timestamp_) {
  PerceptionObstacles mobileye_obstacles =
      ConvertToPerceptionObstacles(mobileye_, localization_);
  obstacles.MergeFrom(mobileye_obstacles);
  // }

  // if (delphi_esr_.header().timestamp_sec() >= last_timestamp_) {
  PerceptionObstacles delphi_esr_obstacles =
      ConvertToPerceptionObstacles(delphi_esr_, localization_);
  PerceptionObstacles filtered_delphi_esr_obstacles =
      FilterDelphiEsrPerceptionObstacles(delphi_esr_obstacles, delphi_esr_);
  obstacles.MergeFrom(filtered_delphi_esr_obstacles);
  // }

  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_node_name, &obstacles);
  AdapterManager::PublishPerceptionObstacles(obstacles);
  last_timestamp_ = apollo::common::time::Clock::NowInSecond();
}

}  // namespace l3_perception
}  // namespace apollo
