/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"

#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"
#include "modules/third_party_perception/tools/conversion_radar.h"

/**
 * @namespace apollo::third_party_perception::conversion_radar
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_radar {

using apollo::canbus::Chassis;
using apollo::drivers::DelphiESR;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using Point = apollo::common::Point3D;

RadarObstacles ContiToRadarObstacles(
    const apollo::drivers::ContiRadar& conti_radar,
    const apollo::localization::LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles, const Chassis& chassis) {
  RadarObstacles obstacles;

  const double last_timestamp = last_radar_obstacles.header().timestamp_sec();
  const double current_timestamp = conti_radar.header().timestamp_sec();

  const auto adc_pos = localization.pose().position();
  const auto adc_vel = localization.pose().linear_velocity();
  const auto adc_quaternion = localization.pose().orientation();
  const double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < conti_radar.contiobs_size(); ++index) {
    const auto& contiobs = conti_radar.contiobs(index);

    RadarObstacle rob;

    rob.set_id(contiobs.obstacle_id());
    rob.set_rcs(contiobs.rcs());
    rob.set_length(GetDefaultObjectLength(4));
    rob.set_width(GetDefaultObjectWidth(4));
    rob.set_height(3.0);

    Point relative_pos_sl;

    // TODO(QiL): load the radar configs here
    relative_pos_sl.set_x(contiobs.longitude_dist());
    relative_pos_sl.set_y(contiobs.lateral_dist());
    rob.mutable_relative_position()->CopyFrom(relative_pos_sl);

    Point relative_pos_xy = SLtoXY(relative_pos_sl, adc_theta);
    Point absolute_pos;
    absolute_pos.set_x(adc_pos.x() + relative_pos_xy.x());
    absolute_pos.set_y(adc_pos.y() + relative_pos_xy.y());
    absolute_pos.set_z(adc_pos.z());
    rob.mutable_absolute_position()->CopyFrom(absolute_pos);

    rob.mutable_relative_velocity()->set_x(contiobs.longitude_vel());
    rob.mutable_relative_velocity()->set_y(contiobs.lateral_vel());

    const auto iter = last_radar_obstacles.radar_obstacle().find(index);
    Point absolute_vel;
    if (iter == last_radar_obstacles.radar_obstacle().end()) {
      rob.set_count(0);
      rob.set_movable(false);
      rob.set_moving_frames_count(0);
      absolute_vel.set_x(0.0);
      absolute_vel.set_y(0.0);
      absolute_vel.set_z(0.0);
    } else if (!FLAGS_use_navigation_mode) {
      rob.set_count(iter->second.count() + 1);
      rob.set_movable(iter->second.movable());
      absolute_vel.set_x(
          (absolute_pos.x() - iter->second.absolute_position().x()) /
          (current_timestamp - last_timestamp));
      absolute_vel.set_y(
          (absolute_pos.y() - iter->second.absolute_position().y()) /
          (current_timestamp - last_timestamp));
      absolute_vel.set_z(0.0);
      double v_heading = std::atan2(absolute_vel.y(), absolute_vel.x());
      double heading_diff = HeadingDifference(v_heading, rob.theta());
      if (heading_diff < FLAGS_movable_heading_threshold &&
          Speed(absolute_vel) * std::cos(heading_diff) >
              FLAGS_movable_speed_threshold) {
        rob.set_moving_frames_count(iter->second.moving_frames_count() + 1);
      } else {
        rob.set_moving_frames_count(0);
      }
    } else {
      rob.set_count(iter->second.count() + 1);
      rob.set_movable(iter->second.movable());
      absolute_vel.set_x(contiobs.longitude_vel() + chassis.speed_mps());
      absolute_vel.set_y(contiobs.lateral_vel());
      absolute_vel.set_z(0.0);

      // Overwrite heading here with relative headings
      // TODO(QiL) : refind the logic here.
      if (contiobs.clusterortrack() == 0) {
        rob.set_theta(contiobs.oritation_angle() / 180 * M_PI);
      } else {
        // in FLU
        rob.set_theta(std::atan2(rob.relative_position().x(),
                                 rob.relative_position().y()));
      }
    }

    rob.mutable_absolute_velocity()->CopyFrom(absolute_vel);

    if (rob.moving_frames_count() >= FLAGS_movable_frames_count_threshold) {
      rob.set_movable(true);
    }
    (*obstacles.mutable_radar_obstacle())[index] = rob;
  }

  obstacles.mutable_header()->CopyFrom(conti_radar.header());
  return obstacles;
}

RadarObstacles DelphiToRadarObstacles(
    const DelphiESR& delphi_esr, const LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles) {
  RadarObstacles obstacles;

  const double last_timestamp = last_radar_obstacles.header().timestamp_sec();
  const double current_timestamp = delphi_esr.header().timestamp_sec();

  // assign motion power from 540
  std::vector<apollo::drivers::Esr_trackmotionpower_540::Motionpower>
      motionpowers(64);
  for (const auto& esr_trackmotionpower_540 :
       delphi_esr.esr_trackmotionpower_540()) {
    if (!esr_trackmotionpower_540.has_can_tx_track_can_id_group()) {
      AERROR << "ESR track motion power 540 does not have "
                "can_tx_track_can_id_group()";
      continue;
    }
    const int can_tx_track_can_id_group =
        esr_trackmotionpower_540.can_tx_track_can_id_group();
    const int can_tx_track_motion_power_size =
        esr_trackmotionpower_540.can_tx_track_motion_power_size();
    for (int index = 0; index < (can_tx_track_can_id_group < 9 ? 7 : 1) &&
                        index < can_tx_track_motion_power_size;
         ++index) {
      std::size_t motion_powers_index = can_tx_track_can_id_group * 7 + index;
      if (motion_powers_index < motionpowers.size()) {
        motionpowers[motion_powers_index].CopyFrom(
            esr_trackmotionpower_540.can_tx_track_motion_power(index));
      }
    }
  }

  const auto adc_pos = localization.pose().position();
  const auto adc_vel = localization.pose().linear_velocity();
  const auto adc_quaternion = localization.pose().orientation();
  const double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < delphi_esr.esr_track01_500_size() &&
                      index < static_cast<int>(motionpowers.size());
       ++index) {
    const auto& data_500 = delphi_esr.esr_track01_500(index);

    // ignore invalid target
    if (data_500.can_tx_track_status() ==
        apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET) {
      continue;
    }

    RadarObstacle rob;

    rob.set_id(index);
    rob.set_rcs(static_cast<double>(motionpowers[index].can_tx_track_power()) -
                10.0);
    rob.set_length(GetDefaultObjectLength(4));
    rob.set_width(GetDefaultObjectWidth(4));
    rob.set_height(3.0);

    const double range = data_500.can_tx_track_range();
    const double angle = data_500.can_tx_track_angle() * M_PI / 180.0;
    Point relative_pos_sl;
    relative_pos_sl.set_x(range * std::cos(angle) +
                          FLAGS_radar_pos_adjust +  // offset: imu <-> mobileye
                          rob.length() /
                              2.0);  // make x the middle point of the vehicle
    relative_pos_sl.set_y(range * std::sin(angle));
    rob.mutable_relative_position()->CopyFrom(relative_pos_sl);

    Point relative_pos_xy = SLtoXY(relative_pos_sl, adc_theta);
    Point absolute_pos;
    absolute_pos.set_x(adc_pos.x() + relative_pos_xy.x());
    absolute_pos.set_y(adc_pos.y() + relative_pos_xy.y());
    absolute_pos.set_z(adc_pos.z());
    rob.mutable_absolute_position()->CopyFrom(absolute_pos);

    double theta = GetNearestLaneHeading(rob.absolute_position());
    rob.set_theta(theta);

    const double range_vel = data_500.can_tx_track_range_rate();
    const double lateral_vel = data_500.can_tx_track_lat_rate();
    rob.mutable_relative_velocity()->set_x(range_vel * std::cos(angle) -
                                           lateral_vel * std::sin(angle));
    rob.mutable_relative_velocity()->set_y(range_vel * std::sin(angle) +
                                           lateral_vel * std::cos(angle));

    const auto iter = last_radar_obstacles.radar_obstacle().find(index);
    Point absolute_vel;
    if (iter == last_radar_obstacles.radar_obstacle().end()) {
      rob.set_count(0);
      rob.set_movable(false);
      rob.set_moving_frames_count(0);
      absolute_vel.set_x(0.0);
      absolute_vel.set_y(0.0);
      absolute_vel.set_z(0.0);
    } else {
      rob.set_count(iter->second.count() + 1);
      rob.set_movable(iter->second.movable());
      absolute_vel.set_x(
          (absolute_pos.x() - iter->second.absolute_position().x()) /
          (current_timestamp - last_timestamp));
      absolute_vel.set_y(
          (absolute_pos.y() - iter->second.absolute_position().y()) /
          (current_timestamp - last_timestamp));
      absolute_vel.set_z(0.0);
      double v_heading = std::atan2(absolute_vel.y(), absolute_vel.x());
      double heading_diff = HeadingDifference(v_heading, rob.theta());
      if (heading_diff < FLAGS_movable_heading_threshold &&
          Speed(absolute_vel) * std::cos(heading_diff) >
              FLAGS_movable_speed_threshold) {
        rob.set_moving_frames_count(iter->second.moving_frames_count() + 1);
      } else {
        rob.set_moving_frames_count(0);
      }
    }

    rob.mutable_absolute_velocity()->CopyFrom(absolute_vel);

    if (rob.moving_frames_count() >= FLAGS_movable_frames_count_threshold) {
      rob.set_movable(true);
    }
    (*obstacles.mutable_radar_obstacle())[index] = rob;
  }

  obstacles.mutable_header()->CopyFrom(delphi_esr.header());
  return obstacles;
}

PerceptionObstacles RadarObstaclesToPerceptionObstacles(
    const RadarObstacles& radar_obstacles) {
  PerceptionObstacles obstacles;

  for (const auto& iter : radar_obstacles.radar_obstacle()) {
    auto* pob = obstacles.add_perception_obstacle();
    const auto& radar_obstacle = iter.second;

    pob->set_id(radar_obstacle.id() + FLAGS_radar_id_offset);

    pob->set_type(PerceptionObstacle::VEHICLE);
    pob->set_length(radar_obstacle.length());
    pob->set_width(radar_obstacle.width());
    pob->set_height(radar_obstacle.height());

    if (!FLAGS_use_navigation_mode) {
      pob->mutable_position()->CopyFrom(radar_obstacle.absolute_position());
      pob->mutable_velocity()->CopyFrom(radar_obstacle.absolute_velocity());
    } else {
      pob->mutable_position()->CopyFrom(radar_obstacle.relative_position());
      pob->mutable_velocity()->CopyFrom(radar_obstacle.relative_velocity());
    }

    pob->set_theta(radar_obstacle.theta());

    // create polygon
    FillPerceptionPolygon(pob, pob->position().x(), pob->position().y(),
                          pob->position().z(), pob->length(), pob->width(),
                          pob->height(), pob->theta());
    pob->set_confidence(0.01);
  }

  obstacles.mutable_header()->CopyFrom(radar_obstacles.header());

  return obstacles;
}

}  // namespace conversion_radar
}  // namespace third_party_perception
}  // namespace apollo
