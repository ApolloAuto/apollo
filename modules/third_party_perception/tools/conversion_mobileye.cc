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
#include "modules/third_party_perception/tools/conversion_mobileye.h"

/**
 * @namespace apollo::third_party_perception::conversion_mobileye
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_mobileye {

using apollo::canbus::Chassis;
using apollo::drivers::Mobileye;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using Point = apollo::common::Point3D;

PerceptionObstacles MobileyeToPerceptionObstacles(
    const Mobileye& mobileye, const LocalizationEstimate& localization,
    const Chassis& chassis) {
  PerceptionObstacles obstacles;
  // retrieve position and velocity of the main vehicle from the localization
  // position
  std::map<std::int32_t, apollo::hdmap::LaneBoundaryType_Type>
    lane_conversion_map = {{0, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {1, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {2, apollo::hdmap::LaneBoundaryType::UNKNOWN},
                           {3, apollo::hdmap::LaneBoundaryType::CURB},
                           {4, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {5, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {6, apollo::hdmap::LaneBoundaryType::UNKNOWN}};

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
      path_c3 =
          obstacles.lane_marker().right_lane_marker().c3_curvature_derivative();
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

}  // namespace conversion_mobileye
}  // namespace third_party_perception
}  // namespace apollo
