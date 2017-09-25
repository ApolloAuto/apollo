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

/**
 * @file
 */

#include "modules/l3_perception/convertion.h"
#include "modules/l3_perception/l3_perception_gflags.h"
#include "modules/l3_perception/l3_perception_util.h"

/**
 * @namespace apollo::l3_perception::convertion
 * @brief apollo::l3_perception
 */
namespace apollo {
namespace l3_perception {
namespace convertion {

using ::apollo::l3_perception::GetAngleFromQuaternion;
using ::apollo::l3_perception::FillPerceptionPolygon;
using ::apollo::l3_perception::GetDefaultObjectLength;
using ::apollo::l3_perception::GetDefaultObjectWidth;

PerceptionObstacles MobileyeToPerceptionObstacles(
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
    double mid_z = adc_z;
    double heading = mob->theta();

    FillPerceptionPolygon(mob, mid_x, mid_y, mid_z, mob_l, mob_w, mob->height(),
                          heading);

    mob->set_confidence(0.75);
  }

  return obstacles;
}

RadarObstacles DelphiToRadarObstacles(const DelphiESR& delphi_esr) {
  RadarObstacles obstacles;
  
  // assign motion power from 540
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
    const auto& data_500 = delphi_esr.esr_track01_500(index);

    // ignore invalid target
    if (data_500.can_tx_track_status() ==
        ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET) {
      continue;
    }

    auto* rob = obstacles.add_radar_obstacle();

    rob->set_id(index);

    double range = data_500.can_tx_track_range();
    double angle = data_500.can_tx_track_angle() * L3_PI / 180.0;
    rob->mutable_relative_position()->set_x(range * std::cos(angle));
    rob->mutable_relative_position()->set_y(range * std::sin(angle));

    rob->set_rcs(static_cast<double>(motionpowers[index].can_tx_track_power()) - 10.0);

    double range_vel = data_500.can_tx_track_range_rate();
    double lateral_vel = data_500.can_tx_track_lat_rate();
    rob->mutable_relative_velocity()->set_x(range_vel * std::cos(angle) - 
                                            lateral_vel * std::sin(angle)); 
    rob->mutable_relative_velocity()->set_y(range_vel * std::sin(angle) + 
                                            lateral_vel * std::cos(angle)); 
  }

  return obstacles;
}

PerceptionObstacles RadarObstaclesToPerceptionObstacles(
    const RadarObstacles& radar_obstacles, const LocalizationEstimate& localization) {
  PerceptionObstacles obstacles;
  double adc_x = localization.pose().position().x();
  double adc_y = localization.pose().position().y();
  double adc_z = localization.pose().position().z();
  auto adc_quaternion = localization.pose().orientation();
  double adc_vx = localization.pose().linear_velocity().x();
  double adc_vy = localization.pose().linear_velocity().y();

  double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < radar_obstacles.radar_obstacle_size(); ++index) {
    auto* pob = obstacles.add_perception_obstacle();
    const auto& radar_obstacle = radar_obstacles.radar_obstacle(index);

    pob->set_id(radar_obstacle.id());

    pob->set_type(PerceptionObstacle::UNKNOWN);  // UNKNOWN
    pob->set_length(GetDefaultObjectLength(4));
    pob->set_width(GetDefaultObjectWidth(4));
    pob->set_height(3.0);

    double pob_pos_x = radar_obstacle.relative_position().x();
    double pob_pos_y = radar_obstacle.relative_position().y();
    double pob_vel_x = radar_obstacle.relative_velocity().x();
    double pob_vel_y = radar_obstacle.relative_velocity().y();

    // TODO(lizh): calibrate mobileye and make those consts FLAGS
    pob_pos_x += FLAGS_delphi_esr_pos_adjust;  // offset: imu <-> mobileye
    pob_pos_x += pob->length() / 2.0; // make x the middle point of the vehicle.

    // position: sl->xy
    Point sl_point;
    sl_point.set_x(pob_pos_x);
    sl_point.set_y(pob_pos_y);
    Point xy_point = SLtoXY(sl_point, adc_theta);
    double converted_x = adc_x + xy_point.x();
    double converted_y = adc_y + xy_point.y();

    pob->mutable_position()->set_x(converted_x);
    pob->mutable_position()->set_y(converted_y);

    // velocity: sl->xy
    sl_point.set_x(pob_vel_x);
    sl_point.set_y(pob_vel_y);
    xy_point = SLtoXY(sl_point, adc_theta);
    double converted_vx = adc_vx + xy_point.x();
    double converted_vy = adc_vy + xy_point.y();

    pob->mutable_velocity()->set_x(converted_vx);
    pob->mutable_velocity()->set_y(converted_vy);
    
    pob->set_theta(adc_theta);

    // create polygon
    pob->clear_polygon_point();
    double mid_x = converted_x;
    double mid_y = converted_y;
    double mid_z = adc_z;
    double heading = pob->theta();

    FillPerceptionPolygon(pob, mid_x, mid_y, mid_z, 
                          pob->length(), pob->width(), pob->height(),
                          heading);

    pob->set_confidence(0.5);
  }

  return obstacles;
}

}  // namespace convertion 
}  // namespace l3_perception
}  // namespace apollo



