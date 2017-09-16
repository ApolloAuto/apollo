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
using apollo::perception::Point;

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

    Point sl_point;
    sl_point.set_x(mob_pos_x);
    sl_point.set_y(mob_pos_y);
    Point xy_point = SLtoXY(sl_point, adc_theta);

    double converted_x = adc_x + xy_point.x();
    double converted_y = adc_y + xy_point.y();
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

    FillPerceptionPolygon(mob, mid_x, mid_y, mid_z, mob_l, mob_w, mob->height(),
                          heading);
  }

  return obstacles;
}

RadarObstacles L3Perception::ConvertToRadarObstacles(
    const DelphiESR& delphi_esr, const RadarObstacles& last,
    const LocalizationEstimate& localization, const double last_timestamp,
    const double current_timestamp) {
  RadarObstacles current;

  const auto adc_pos = localization.pose().position();
  const auto adc_vel = localization.pose().linear_velocity();
  const auto adc_quaternion = localization.pose().orientation();
  const double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  std::vector<apollo::drivers::Esr_trackmotionpower_540::Motionpower>
      motionpowers(64);
  for (const auto& esr_trackmotionpower_540 :
       delphi_esr.esr_trackmotionpower_540()) {
    const int& can_tx_track_can_id_group =
        esr_trackmotionpower_540.can_tx_track_can_id_group();
    for (int index = 0; index < (can_tx_track_can_id_group < 9 ? 7 : 1);
         ++index) {
      motionpowers[can_tx_track_can_id_group * 7 + index].CopyFrom(
          esr_trackmotionpower_540.can_tx_track_motion_power(index));
    }
  }

  for (int index = 0; index < delphi_esr.esr_track01_500_size(); ++index) {
    RadarObstacle radar_obstacle;

    const auto& data_500 = delphi_esr.esr_track01_500(index);
    //TODO(lizh): object id
    int id = index;
    radar_obstacle.set_id(id);
    radar_obstacle.set_relative_range(data_500.can_tx_track_range());
    radar_obstacle.set_relative_angle(data_500.can_tx_track_angle() * L3_PI / 180.0);

    radar_obstacle.set_length(GetDefaultObjectLength(4));
    radar_obstacle.set_width(GetDefaultObjectWidth(4));

    Point relative_pos_sl;
    relative_pos_sl.set_x(radar_obstacle.relative_range() *
                           std::cos(radar_obstacle.relative_angle()) +
                       FLAGS_delphi_esr_pos_adjust + // offset: imu <-> radar
                       radar_obstacle.length() / 2.0); // make x the middle point of the vehicle.
    relative_pos_sl.set_y(radar_obstacle.relative_range() *
                       std::sin(radar_obstacle.relative_angle()));

    Point relative_pos_xy = SLtoXY(relative_pos_sl, adc_theta);
    Point absolute_pos;
    absolute_pos.set_x(adc_pos.x() + relative_pos_xy.x());
    absolute_pos.set_y(adc_pos.y() + relative_pos_xy.y());
    radar_obstacle.set_position(absolute_pos);

    Point absolute_vel;
    const auto iter = last.find(id);
    if (iter == last.end()) {
      // new in the current frame
      absolute_vel.set_x(0.0);
      absolute_vel.set_y(0.0);
      radar_obstacle.set_movable(false);
    } else {
      // also appeared in the last frame
      absolute_vel.set_x((absolute_pos.x() - iter->second.position().x()) / (current_timestamp - last_timestamp));
      absolute_vel.set_y((absolute_pos.y() - iter->second.position().y()) / (current_timestamp - last_timestamp));
      double absolute_speed = std::sqrt(absolute_vel.x() * absolute_vel.x() +
                                        absolute_vel.y() * absolute_vel.y());
      if (absolute_speed > 5.0) {
        radar_obstacle.set_movable(true);
      }
    }

    radar_obstacle.set_velocity(absolute_vel);
    radar_obstacle.set_rcs(static_cast<double>(motionpowers[index].can_tx_track_power()) - 10.0);

    current[id] = radar_obstacle;
  }
  return current;
}

PerceptionObstacles L3Perception::ConvertToPerceptionObstacles(
    const RadarObstacles& radar_obstacles) {
  PerceptionObstacles obstacles;

  for (const auto& pair_id_radar_obstacle : radar_obstacles) {
    const int& id = pair_id_radar_obstacle.first;
    const RadarObstacle& radar_obstacle = pair_id_radar_obstacle.second;

    auto* mob = obstacles.add_perception_obstacle();
    mob->set_id(id);
    mob->mutable_position()->CopyFrom(radar_obstacle.position());

    mob->set_type(PerceptionObstacle::UNKNOWN);  // UNKNOWN

    mob->mutable_velocity()->CopyFrom(radar_obstacle.velocity());
    mob->set_length(radar_obstacle.length());
    mob->set_width(radar_obstacle.width());
    mob->set_theta(
        std::atan2(radar_obstacle.velocity().y(), radar_obstacle.velocity().x()));
    mob->set_height(3.0);

    mob->clear_polygon_point();

    FillPerceptionPolygon(
        mob, radar_obstacle.position().x(), radar_obstacle.position().y(),
        radar_obstacle.position().z(), mob->length(), mob->width(), mob->height(), mob->theta());
  }
  return obstacles;
}

bool IsPreserved(const RadarObstacle& radar_obstacle) {
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
  if (esr_track01_500.can_tx_track_status() == ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET ||
    esr_track01_500.can_tx_track_status() == ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_COASTED_TARGET ) {
    return false;
  }
  */
  return true;
}

RadarObstacles L3Perception::FilterRadarObstacles(
    const RadarObstacles& radar_obstacles) {
  RadarObstacles filtered_radar_obstacles;
  for (const auto& pair_id_radar_obstacle : radar_obstacles) {
    if (IsPreserved(pair_id_radar_obstacle.second)) {
      filtered_radar_obstacles[pair_id_radar_obstacle.first] = pair_id_radar_obstacle.second;
    }
  }
  return filtered_radar_obstacles;
}

void L3Perception::OnTimer(const ros::TimerEvent&) {
  AINFO << "publish PerceptionObstacles";

  double current_timestamp = apollo::common::time::Clock::NowInSecond();

  PerceptionObstacles obstacles;

  // TODO(lizh): check timestamp before publish.
  // if (mobileye_.header().timestamp_sec() >= last_timestamp_) {
  PerceptionObstacles mobileye_obstacles =
      ConvertToPerceptionObstacles(mobileye_, localization_);
  obstacles.MergeFrom(mobileye_obstacles);
  // }

  // if (delphi_esr_.header().timestamp_sec() >= last_timestamp_) {
  RadarObstacles curr_map = ConvertToRadarObstacles(delphi_esr_, last_map_, localization_,
                                      last_timestamp_, current_timestamp);
  RadarObstacles filtered_map = FilterRadarObstacles(curr_map);
  PerceptionObstacles filtered_delphi_esr_obstacles =
      ConvertToPerceptionObstacles(filtered_map);
  obstacles.MergeFrom(filtered_delphi_esr_obstacles);
  // }

  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_node_name, &obstacles);
  AdapterManager::PublishPerceptionObstacles(obstacles);

  last_timestamp_ = current_timestamp;
  last_map_ = curr_map;
}

}  // namespace l3_perception
}  // namespace apollo
