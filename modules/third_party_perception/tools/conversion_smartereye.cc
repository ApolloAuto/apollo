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
#include "modules/third_party_perception/tools/conversion_smartereye.h"

/**
 * @namespace apollo::third_party_perception::conversion_smartereye
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_smartereye {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using Point = apollo::common::Point3D;

apollo::perception::PerceptionObstacles SmartereyeToPerceptionObstacles(
    const apollo::drivers::SmartereyeObstacles& smartereye_obstacles,
    const apollo::drivers::SmartereyeLanemark& smartereye_lanemark,
    const apollo::localization::LocalizationEstimate& localization,
    const apollo::canbus::Chassis& chassis) {
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

  obstacles.mutable_header()->CopyFrom(smartereye_obstacles.header());

  // Fullfill lane_type information
  std::int32_t sma_left_lane_type =
      smartereye_lanemark.lane_road_data().roadway().left_lane().style();
  std::int32_t sma_right_lane_type =
      smartereye_lanemark.lane_road_data().roadway().right_lane().style();

  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_lane_type(
      lane_conversion_map[sma_left_lane_type]);
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_lane_type(
      lane_conversion_map[sma_right_lane_type]);

  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_quality(
      smartereye_lanemark.lane_road_data().roadway()
            .left_lane().quality() / 4.0);
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()
      ->set_model_degree(smartereye_lanemark.lane_road_data().roadway()
            .left_lane().right_boundary().degree());

  // Convert everything to FLU
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()->set_c0_position(
      smartereye_lanemark.lane_road_data().roadway()
            .left_lane().right_boundary().c0_position());
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()
      ->set_c1_heading_angle(smartereye_lanemark.lane_road_data().roadway()
            .left_lane().right_boundary().c1_heading_angle());
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()
      ->set_c2_curvature(smartereye_lanemark.lane_road_data().roadway()
            .left_lane().right_boundary().c2_curvature());
  obstacles.mutable_lane_marker()->mutable_left_lane_marker()
      ->set_c3_curvature_derivative(
            smartereye_lanemark.lane_road_data().roadway()
            .left_lane().right_boundary().c3_curvature_derivative());
  // obstacles.mutable_lane_marker()->mutable_left_lane_marker()
  //      ->set_view_range(
  //     smartereye_lanemark.lane_road_data().roadway().left_lane().width());

  obstacles.mutable_lane_marker()->mutable_right_lane_marker()->set_quality(
      smartereye_lanemark.lane_road_data().roadway()
            .right_lane().quality() / 4.0);
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()
      ->set_model_degree(smartereye_lanemark.lane_road_data().roadway()
            .right_lane().left_boundary().degree());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()
      ->set_c0_position(smartereye_lanemark.lane_road_data().roadway()
            .right_lane().left_boundary().c0_position());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()
      ->set_c1_heading_angle(smartereye_lanemark.lane_road_data().roadway()
            .right_lane().left_boundary().c1_heading_angle());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()
      ->set_c2_curvature(smartereye_lanemark.lane_road_data().roadway()
            .right_lane().left_boundary().c2_curvature());
  obstacles.mutable_lane_marker()->mutable_right_lane_marker()
      ->set_c3_curvature_derivative(
            smartereye_lanemark.lane_road_data().roadway()
            .right_lane().left_boundary().c3_curvature_derivative());
  // obstacles.mutable_lane_marker()->mutable_right_lane_marker()
  //      ->set_view_range(
  //     smartereye_lanemark.lane_road_data().roadway().right_lane().width());

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

  for (int index = 0; index < smartereye_obstacles.num_obstacles() &&
                      index < smartereye_obstacles.output_obstacles_size();
       ++index) {
    auto* sma = obstacles.add_perception_obstacle();
    const auto& data_obstacle =
        smartereye_obstacles.output_obstacles().at(index);
    int sma_id = data_obstacle.trackid() + FLAGS_smartereye_id_offset;

    double sma_x = data_obstacle.avgdistancez();
    double sma_y = data_obstacle.real3dcenterx();
    double sma_z = (data_obstacle.real3dupy() +
        data_obstacle.real3dlowy()) / 2.0;
    // relative speed
    double sma_vel_x = data_obstacle.fuzzyrelativespeedz();
    int sma_type = data_obstacle.obstacletype();

    double sma_w = data_obstacle.real3drightx() - data_obstacle.real3dleftx();
    double sma_l = data_obstacle.real3dlowy() - data_obstacle.real3dupy();

    switch (sma_type) {
      case 1:
      case 6:
      case 7: {
        sma->set_type(PerceptionObstacle::VEHICLE);  // VEHICLE
        break;
      }
      case 4:
      case 5: {
        sma->set_type(PerceptionObstacle::BICYCLE);  // BIKE
        break;
      }
      case 2:
      case 3: {
        sma->set_type(PerceptionObstacle::PEDESTRIAN);  // PED
        break;
      }
      default: {
        sma->set_type(PerceptionObstacle::UNKNOWN);  // UNKNOWN
        break;
      }
    }

    if (sma_l > FLAGS_max_mobileye_obstacle_length) {
      sma_l = GetDefaultObjectLength(sma_type);
    }
    if (sma_w > FLAGS_max_mobileye_obstacle_width) {
      sma_w = GetDefaultObjectWidth(sma_type);
    }

    Point xy_point = SLtoXY(sma_x, sma_y, adc_theta);

    // TODO(QiL) : Clean this up after data collection and validation
    double converted_x = 0.0;
    double converted_y = 0.0;
    double converted_z = 0.0;
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
      converted_z = adc_z + sma_z;
      sma->set_theta(GetNearestLaneHeading(converted_x, converted_y, adc_z));
      converted_speed = adc_velocity + sma_vel_x;
      converted_vx = converted_speed * std::cos(sma->theta());
      converted_vy = converted_speed * std::sin(sma->theta());
    } else {
      converted_x = data_obstacle.real3dcenterx() - sma_l / 2.0;
      converted_y = sma_y;
      converted_vx = sma_vel_x + chassis.speed_mps();
      converted_vy = 0.0;
      converted_z = data_obstacle.avgdistancez();

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
      sma->set_theta(nearest_lane_heading);
    }

    sma->set_id(sma_id);
    sma->mutable_position()->set_x(converted_x);
    sma->mutable_position()->set_y(converted_y);
    sma->mutable_position()->set_z(converted_z);

    sma->mutable_velocity()->set_x(converted_vx);
    sma->mutable_velocity()->set_y(converted_vy);
    sma->set_length(sma_l);
    sma->set_width(sma_w);
    sma->set_height(FLAGS_default_height);

    sma->clear_polygon_point();

    // create polygon
    FillPerceptionPolygon(sma, sma->position().x(), sma->position().y(),
                          sma->position().z(), sma->length(), sma->width(),
                          sma->height(), sma->theta());

    sma->set_confidence(0.5);
  }

  return obstacles;
}

}  // namespace conversion_smartereye
}  // namespace third_party_perception
}  // namespace apollo
