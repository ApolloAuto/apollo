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

#include <cmath>
#include <map>
#include <vector>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"

#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"
#include "modules/third_party_perception/conversion.h"

/**
 * @namespace apollo::third_party_perception::conversion
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion {

using apollo::canbus::Chassis;
using apollo::drivers::ContiRadar;
using apollo::drivers::DelphiESR;
using apollo::drivers::Mobileye;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::perception::Point;

std::map<std::int32_t, ::apollo::hdmap::LaneBoundaryType_Type>
    lane_conversion_map = {{0, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {1, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {2, apollo::hdmap::LaneBoundaryType::UNKNOWN},
                           {3, apollo::hdmap::LaneBoundaryType::CURB},
                           {4, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {5, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {6, apollo::hdmap::LaneBoundaryType::UNKNOWN}};

PerceptionObstacles MobileyeToPerceptionObstacles(
    const Mobileye& mobileye, const LocalizationEstimate& localization,
    const Chassis& chassis) {
  PerceptionObstacles obstacles;
  // retrieve position and velocity of the main vehicle from the localization
  // position

  obstacles.mutable_header()->CopyFrom(mobileye.header());

  // Fullfill lane_type information
  std::int32_t mob_left_lane_type = mobileye.lka_766().lane_type();
  std::int32_t mob_right_lane_type = mobileye.lka_768().lane_type();

  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_lane_type(
      lane_conversion_map[mob_left_lane_type]);
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_lane_type(
      lane_conversion_map[mob_right_lane_type]);

  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_quality(
      mobileye.lka_766().quality() / 4.0);
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_model_degree(
      mobileye.lka_766().model_degree());

  // Convert everything to FLU
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_c0_position(
      -mobileye.lka_766().position());
  obstacles.mutable_lane_marker()
      ->mutable_left_lane_marker()
      ->set_c1_heading_angle(-mobileye.lka_767().heading_angle());
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_c2_curvature(
      -mobileye.lka_766().curvature());
  obstacles.mutable_lane_marker()
      ->mutable_left_lane_marker()
      ->set_c3_curvature_derivative(-mobileye.lka_766().curvature_derivative());
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_view_range(
      mobileye.lka_767().view_range());

  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_quality(
      mobileye.lka_768().quality() / 4.0);
  obstacles.mutable_lane_marker()
      ->mutable_right_lane_marker()
      ->set_model_degree(mobileye.lka_768().model_degree());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_c0_position(
      -mobileye.lka_768().position());
  obstacles.mutable_lane_marker()
      ->mutable_right_lane_marker()
      ->set_c1_heading_angle(-mobileye.lka_769().heading_angle());
  obstacles.mutable_lane_marker()
      ->mutable_right_lane_marker()
      ->set_c2_curvature(-mobileye.lka_768().curvature());
  obstacles.mutable_lane_marker()
      ->mutable_right_lane_marker()
      ->set_c3_curvature_derivative(-mobileye.lka_768().curvature_derivative());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_view_range(
      mobileye.lka_769().view_range());

  double adc_x = localization.pose().position().x();
  double adc_y = localization.pose().position().y();
  double adc_z = localization.pose().position().z();
  // heading
  auto adc_quaternion = localization.pose().orientation();
  double adc_theta = GetAngleFromQuaternion(adc_quaternion);
  // velocity
  double adc_vx = localization.pose().linear_velocity().x();
  double adc_vy = localization.pose().linear_velocity().y();
  double adc_velocity = Speed(adc_vx, adc_vy);

  for (int index = 0; index < mobileye.details_738().num_obstacles() &&
                      index < mobileye.details_739_size();
       ++index) {
    auto* mob = obstacles.add_perception_obstacle();
    const auto& data_739 = mobileye.details_739(index);
    int mob_id = data_739.obstacle_id() + FLAGS_mobileye_id_offset;
    double mob_x = data_739.obstacle_pos_x();
    double mob_y = -data_739.obstacle_pos_y();
    double mob_vel_x = data_739.obstacle_rel_vel_x();
    int mob_type = data_739.obstacle_type();

    double mob_l = 0.0;
    double mob_w = 0.0;
    if (mobileye.details_73a_size() <= index) {
      mob_l = GetDefaultObjectLength(mob_type);
      mob_w = GetDefaultObjectWidth(mob_type);
    } else {
      if (mobileye.details_73a(index).obstacle_length() >
          FLAGS_max_mobileye_obstacle_length) {
        mob_l = GetDefaultObjectLength(mob_type);
      } else {
        mob_l = mobileye.details_73a(index).obstacle_length();
      }

      if (mobileye.details_73a(index).obstacle_width() >
          FLAGS_max_mobileye_obstacle_width) {
        mob_w = GetDefaultObjectWidth(mob_type);
      } else {
        mob_w = mobileye.details_73a(index).obstacle_width();
      }
    }

    mob_x += FLAGS_mobileye_pos_adjust;  // offset: imu <-> mobileye
    mob_x += mob_l / 2.0;  // make x the middle point of the vehicle.

    Point xy_point = SLtoXY(mob_x, mob_y, adc_theta);

    // TODO(QiL) : Clean this up after data collection and validation
    double converted_x = 0.0;
    double converted_y = 0.0;
    double converted_speed = 0.0;
    double converted_vx = 0.0;
    double converted_vy = 0.0;

    double path_c1 = 0.0;
    double path_c2 = 0.0;
    double path_c3 = 0.0;

    if (obstacles.lane_marker().left_lane_marker().quality() >=
        obstacles.lane_marker().right_lane_marker().quality()) {
      path_c1 = obstacles.lane_marker().left_lane_marker().c1_heading_angle();
      path_c2 = obstacles.lane_marker().left_lane_marker().c2_curvature();
      path_c3 =
          obstacles.lane_marker().left_lane_marker().c3_curvature_derivative();
    } else {
      path_c1 = obstacles.lane_marker().right_lane_marker().c1_heading_angle();
      path_c2 = obstacles.lane_marker().right_lane_marker().c2_curvature();
      path_c3 = obstacles.lane_marker().right_lane_marker().c2_curvature();
    }

    if (!FLAGS_use_navigation_mode) {
      converted_x = adc_x + xy_point.x();
      converted_y = adc_y + xy_point.y();
      mob->set_theta(GetNearestLaneHeading(converted_x, converted_y, adc_z));
      converted_speed = adc_velocity + mob_vel_x;
      converted_vx = converted_speed * std::cos(mob->theta());
      converted_vy = converted_speed * std::sin(mob->theta());
    } else {
      converted_x = mobileye.details_739(index).obstacle_pos_x() +
                    FLAGS_mobileye_pos_adjust;
      converted_y = mobileye.details_739(index).obstacle_pos_y();
      converted_vx = mob_vel_x + chassis.speed_mps();
      converted_vy = 0.0;

      if (mobileye.details_73b_size() <= index) {
        mob->set_theta(0.0);
      } else {
        if (!FLAGS_overwrite_mobileye_theta) {
          mob->set_theta(mobileye.details_73b(index).obstacle_angle() / 180 *
                         M_PI);
        } else {
          double nearest_lane_heading =
              converted_vx > 0
                  ? std::atan2(3 * path_c3 * converted_x * converted_x +
                                   2 * path_c2 * converted_x + path_c1,
                               1)
                  : std::atan2(3 * path_c3 * converted_x * converted_x +
                                   2 * path_c2 * converted_x + path_c1,
                               1) +
                        M_PI;
          AINFO << "nearest lane heading is" << nearest_lane_heading;
          mob->set_theta(nearest_lane_heading);
        }
      }
    }

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
    mob->set_height(FLAGS_default_height);

    mob->clear_polygon_point();

    // create polygon
    FillPerceptionPolygon(mob, mob->position().x(), mob->position().y(),
                          mob->position().z(), mob->length(), mob->width(),
                          mob->height(), mob->theta());

    mob->set_confidence(0.5);
  }

  return obstacles;
}

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
    const int& can_tx_track_can_id_group =
        esr_trackmotionpower_540.can_tx_track_can_id_group();
    for (int index = 0; index < (can_tx_track_can_id_group < 9 ? 7 : 1);
         ++index) {
      motionpowers[can_tx_track_can_id_group * 7 + index].CopyFrom(
          esr_trackmotionpower_540.can_tx_track_motion_power(index));
    }
  }

  const auto adc_pos = localization.pose().position();
  const auto adc_vel = localization.pose().linear_velocity();
  const auto adc_quaternion = localization.pose().orientation();
  const double adc_theta = GetAngleFromQuaternion(adc_quaternion);

  for (int index = 0; index < delphi_esr.esr_track01_500_size(); ++index) {
    const auto& data_500 = delphi_esr.esr_track01_500(index);

    // ignore invalid target
    if (data_500.can_tx_track_status() ==
        ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET) {
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

}  // namespace conversion
}  // namespace third_party_perception
}  // namespace apollo
